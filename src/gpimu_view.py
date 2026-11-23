"""GPIMU monitor and independent-file analysis, without altering the active source."""
from bisect import bisect_left
import copy
import csv
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog
import weakref

import numpy as np

from gpimu_data import GP_MODELS, convert_rows, load_gpimu_recording, match_rawimu
from time_axis import TimeAxis


class GPIMUView(object):
    @property
    def frozen(self):
        return self.app.frozen

    def __init__(self, parent, app, plot_class):
        self.parent, self.app = parent, weakref.proxy(app)
        self.file_data = None
        self.loading = False
        self.closed = False
        self._cancel = threading.Event()
        self._messages = queue.Queue()
        self._after = None
        self._signature = None
        self._prepared = None
        self._current = None
        self._load_error = ''
        self._cursor_time = None
        self._updating_position = False
        self.model_var = tk.StringVar(master=parent, value=app.cfg['imu_model'] if app.cfg['imu_model'] in GP_MODELS else 'M1633')
        self.tolerance_var = tk.StringVar(master=parent, value='2')
        self.position_var = tk.DoubleVar(master=parent)
        self.status_var = tk.StringVar(master=parent, value='等待GPIMU；可跟随串口/网口/回放或单独载入文件')
        self.summary_var = tk.StringVar(master=parent)
        self.compare_var = tk.StringVar(master=parent, value='按GPS时间匹配RAWIMU，不插值、不自动拟合标度。')
        self.cursor_var = tk.StringVar(master=parent, value='移动图中游标读取GPIMU原始字段与物理量')
        bar = tk.Frame(parent, bg='white', padx=8, pady=5)
        bar.pack(fill='x')
        ttk.Button(bar, text='跟随当前数据', command=self.follow_current).pack(side='left')
        self.load_button = ttk.Button(bar, text='单独载入文件…', command=self.load_file)
        self.load_button.pack(side='left', padx=5)
        tk.Label(bar, text='GP型号', bg='white', fg=app.MUTED).pack(side='left', padx=(8, 4))
        model = ttk.Combobox(bar, textvariable=self.model_var, values=tuple(GP_MODELS), width=7, state='readonly')
        model.pack(side='left')
        model.bind('<<ComboboxSelected>>', lambda e: self._mark_dirty())
        ttk.Button(bar, text='导出当前窗CSV…', command=self.export_csv).pack(side='right')
        self.position_bar = tk.Frame(parent, bg='white', padx=8, pady=3)
        tk.Label(self.position_bar, text='文件位置(s)', bg='white', fg=app.MUTED).pack(side='left')
        self.position_scale = ttk.Scale(self.position_bar, from_=0, to=1, variable=self.position_var,
                                        command=lambda value: self._mark_dirty() if not self._updating_position else None)
        self.position_scale.pack(side='left', fill='x', expand=True, padx=8)
        self.status_label = tk.Label(parent, textvariable=self.status_var, bg=app.BG, fg=app.MUTED, anchor='w',
                                    font=('Microsoft YaHei', 9), padx=8, pady=3)
        self.status_label.pack(fill='x')
        tk.Label(parent, textvariable=self.summary_var, bg=app.BG, fg=app.INK, anchor='w',
                 font=('Consolas', 9), padx=8).pack(fill='x')
        notebook = ttk.Notebook(parent)
        notebook.pack(fill='both', expand=True, pady=3)
        self.notebook = notebook
        monitor = tk.Frame(notebook, bg=app.BG)
        compare = tk.Frame(notebook, bg=app.BG)
        details = tk.Frame(notebook, bg='white')
        notebook.add(monitor, text='GPIMU监测')
        notebook.add(compare, text='RAWIMU对比')
        notebook.add(details, text='原始字段 / 对比统计')
        notebook.bind('<<NotebookTabChanged>>', lambda e: self._mark_dirty())

        def make_panels(container, specifications, columns, compact):
            panels = []
            for row in range((len(specifications)+columns-1)//columns):
                container.rowconfigure(row, weight=1, uniform='gp_rows')
            for column in range(columns):
                container.columnconfigure(column, weight=1, uniform='gp_cols')
            for index, (title, unit, span) in enumerate(specifications):
                canvas = tk.Canvas(container, bg='white', width=100, height=100, highlightthickness=1,
                                   highlightbackground='#cfd8e4')
                canvas.grid(row=index//columns, column=index%columns, sticky='nsew', padx=2, pady=2)
                panel = plot_class(canvas, title, unit, ['GPIMU'] if compact else ['GPIMU', 'RAWIMU'],
                                   colors=('#237db6', '#dc792f'), min_span=span)
                panel.compact_header = compact
                panel.value_decimals = 5
                canvas.bind('<Configure>', lambda e, p=panel: self._resize(e, p))
                canvas.bind('<Map>', lambda e, p=panel: self._mapped(p))
                panel.bind_cursor(self.inspect_time)
                panel.bind_navigation(self)
                panels.append((canvas, panel))
            return tuple(panels)
        specs = [('采样数 / 语句', 'samples', 4.), ('GPIMU温度', '°C', .2), ('语句时间间隔', 'ms', .1)]
        specs += [(axis+' 角速度', 'deg/s', .0001) for axis in 'XYZ']
        specs += [(axis+' 加速度', 'm/s²', .001) for axis in 'XYZ']
        self.monitor_panels = make_panels(monitor, specs, 3, True)
        self.monitor_panels[0][1].integer_values = True
        self.monitor_panels[0][1].stepped = True
        matchbar = tk.Frame(compare, bg=app.BG, padx=6, pady=4)
        matchbar.pack(fill='x')
        tk.Label(matchbar, text='GPS匹配容差(ms)', bg=app.BG, fg=app.MUTED).pack(side='left')
        tolerance = ttk.Entry(matchbar, textvariable=self.tolerance_var, width=6)
        tolerance.pack(side='left', padx=5)
        tolerance.bind('<Return>', lambda e: self._mark_dirty())
        ttk.Button(matchbar, text='更新匹配', command=self._mark_dirty).pack(side='left')
        tk.Label(compare, textvariable=self.compare_var, bg=app.BG, fg=app.INK,
                 anchor='w', font=('Microsoft YaHei', 9), padx=6).pack(fill='x')
        comparison_grid = tk.Frame(compare, bg=app.BG)
        comparison_grid.pack(fill='both', expand=True)
        self.compare_panels = make_panels(comparison_grid, specs[3:], 3, False)
        self.panels = self.monitor_panels + self.compare_panels
        self.fields = ttk.Treeview(details, columns=('field', 'value'), show='headings', height=9)
        self.fields.heading('field', text='当前GPIMU字段')
        self.fields.heading('value', text='原始值 / 解释')
        self.fields.column('field', width=210)
        self.fields.column('value', width=440)
        self.fields.pack(fill='both', expand=True, padx=8, pady=6)
        self.statistics = ttk.Treeview(details, columns=('axis', 'mean', 'std', 'pp', 'unit'), show='headings', height=6)
        for key, title in zip(('axis', 'mean', 'std', 'pp', 'unit'), ('轴', 'GP−RAW均值', '差值标准差', '差值峰峰值', '单位')):
            self.statistics.heading(key, text=title)
            self.statistics.column(key, width=110, anchor='e')
        self.statistics.pack(fill='x', padx=8, pady=6)
        tk.Label(parent, textvariable=self.cursor_var, bg='white', fg=app.INK, anchor='w',
                 font=('Consolas', 9), padx=6, pady=3).pack(fill='x')
        parent.bind('<Destroy>', self._destroyed, add='+')
        self._after = parent.after(100, self._poll)

    def _mark_dirty(self):
        self._signature = None
        self.app._mark_dirty()

    def _resize(self, event, panel):
        panel.resize(event.width, event.height)
        if panel.c.winfo_ismapped():
            panel.render_enabled = True
            panel.update(panel._last_t, panel._last_ys)
        self._mark_dirty()

    def _mapped(self, panel):
        # Notebook visibility can change without a new data frame or size event.
        panel.render_enabled = True
        panel.update(panel._last_t, panel._last_ys)
        self._mark_dirty()

    def follow_current(self):
        self._cancel.set()
        self.loading = False
        self.load_button.configure(state='normal')
        self.file_data = None
        self._load_error = ''
        self.position_bar.pack_forget()
        self._cursor_time = None
        self._mark_dirty()

    def on_session_changed(self):
        if self.file_data is None:
            self._cursor_time = None
            for _, panel in self.panels:
                panel.cur = None
                panel.show_cursor(None)
            self._mark_dirty()

    def load_file(self, path=None):
        if self.loading:
            return
        if path is None:
            path = filedialog.askopenfilename(parent=self.parent.winfo_toplevel(), title='单独载入GPIMU / RAWIMU数据',
                        filetypes=(('IMU数据', '*.dat *.bin *.log *.txt'), ('所有文件', '*.*')))
        if not path:
            return
        cfg, formats = copy.deepcopy(self.app.sess.analysis_cfg), self.app.formats
        self._cancel = threading.Event()
        cancel, messages = self._cancel, self._messages
        self.loading = True
        self._load_error = ''
        self.load_button.configure(state='disabled')
        self.status_var.set('正在后台载入GPIMU和RAWIMU…')
        def run():
            try:
                result = load_gpimu_recording(path, cfg, cancel, formats)
                messages.put((cancel, result, None))
            except Exception as error:
                messages.put((cancel, None, str(error)))
        threading.Thread(target=run, name='gpimu-file-analysis', daemon=True).start()

    def _poll(self):
        if self.closed:
            return
        try:
            cancel, data, error = self._messages.get_nowait()
        except queue.Empty:
            pass
        else:
            if cancel is self._cancel and not cancel.is_set():
                self.loading = False
                self.load_button.configure(state='normal')
                if error:
                    self._load_error = error
                    self.status_var.set('文件载入失败：'+error)
                else:
                    self.file_data = data
                    self._updating_position = True
                    self.position_scale.configure(to=max(.001, float(data['rows'][-1, 0])))
                    self.position_var.set(float(data['rows'][-1, 0]))
                    self._updating_position = False
                    self.position_bar.pack(fill='x', before=self.status_label)
                    self._cursor_time = None
                    self._mark_dirty()
        self._after = self.parent.after(100, self._poll)

    def update(self, current, window_s):
        self._current = current
        if not self.parent.winfo_ismapped():
            self._signature = None
            return
        data = self.file_data if self.file_data is not None else current
        rows = data['rows']
        end = float(self.position_var.get()) if self.file_data is not None else (float(rows[-1, 0]) if len(rows) else 0.)
        try:
            tolerance = float(self.tolerance_var.get()) / 1000.
            if not np.isfinite(tolerance) or tolerance <= 0:
                raise ValueError()
        except ValueError:
            self.status_var.set('GPS匹配容差必须为正数，单位ms')
            return
        signature = (id(self.file_data) if self.file_data is not None else None, data['total'], data.get('raw_total', len(data['raw_rows'])),
                     data['checksum_bad'], data['malformed'], end, window_s, self.model_var.get(), tolerance,
                     self.app.axis_mode())
        if self._signature == signature:
            return
        self._signature = signature
        start = max(0., end - window_s)
        first = max(0, int(np.searchsorted(rows[:, 0], start)) - 1) if len(rows) else 0
        last = int(np.searchsorted(rows[:, 0], end + 1e-6, side='right')) if len(rows) else 0
        visible = rows[first:last]
        model = self.model_var.get()
        values, temperature = convert_rows(visible, model)
        raw_rows = data['raw_rows']
        gps = visible[:, 1]
        valid_gps = gps[np.isfinite(gps)]
        if len(raw_rows) and len(valid_gps):
            mask = (raw_rows[:, 0] >= valid_gps.min()-tolerance) & (raw_rows[:, 0] <= valid_gps.max()+tolerance)
            raw_rows = raw_rows[mask]
        else:
            raw_rows = np.empty((0, 7))
        matched, offsets = match_rawimu(gps, raw_rows, tolerance)
        times = visible[:, 0].tolist()
        axis = TimeAxis(self.app.axis_mode(), times, visible[:, 4], np.where(visible[:, 15], visible[:, 3], 0.))
        self._prepared = {'rows': visible, 'values': values, 'temperature': temperature, 'raw': matched,
                          'offsets': offsets, 'time': times, 'source': data['source'], 'model': model, 'axis': axis}
        time_range = (start, max(window_s, end))
        series = [visible[:, 5], temperature, visible[:, 13]] + [values[:, k] for k in range(6)]
        for (_, panel), column in zip(self.monitor_panels, series):
            panel.time_range = time_range
            panel.time_axis = axis
            panel.render_enabled = bool(panel.c.winfo_ismapped())
            panel.update(times, [column.tolist()])
        for k, (_, panel) in enumerate(self.compare_panels):
            panel.time_range = time_range
            panel.time_axis = axis
            panel.render_enabled = bool(panel.c.winfo_ismapped())
            panel.update(times, [values[:, k].tolist(), matched[:, k].tolist()])
        matched_count = int(np.isfinite(offsets).sum())
        self.compare_var.set('GPS一对一匹配 %d / %d · 容差 %.3gms · RAW型号 %s（未去零偏）' %
                             (matched_count, len(visible), tolerance*1000., data.get('raw_model', '')))
        self.statistics.delete(*self.statistics.get_children())
        for k, axis in enumerate(('GX','GY','GZ','AX','AY','AZ')):
            difference = values[:, k] - matched[:, k]
            finite = difference[np.isfinite(difference)]
            stats = [np.mean(finite), np.std(finite, ddof=1) if len(finite)>1 else np.nan, np.ptp(finite)] if len(finite) else [np.nan]*3
            self.statistics.insert('', 'end', values=[axis] + [self._number(v) for v in stats] + ['deg/s' if k<3 else 'm/s²'])
        if not self.loading:
            self.status_var.set(('文件载入失败：'+self._load_error+'；仍显示原数据' if self._load_error else
                                '独立文件：'+Path(data['source']).name if self.file_data is not None else '跟随当前串口 / 网口 / 回放') +
                                ' · GP型号 '+model+' · XYZ按MATLAB定义（不套用RAW的Y取反）')
        if len(visible):
            intervals = visible[1:, 13]
            positive = intervals[np.isfinite(intervals) & (intervals > 0)]
            rate = 1000./np.median(positive) if len(positive) else np.nan
            self.summary_var.set('GPIMU %d帧 · %sHz · 采样数/语句 %g · 温度 %.2f°C · XOR错误 %d · 格式错误 %d · %s' %
                (data['total'], self._number(rate), visible[-1, 5], temperature[-1], data['checksum_bad'], data['malformed'],
                 'GPS时间有效' if visible[-1, 15] else 'GPS未就绪，使用IMU时间；不作RAW时间匹配'))
            self._show_fields(len(visible)-1)
        else:
            self.summary_var.set('当前无GPIMU数据 · XOR错误 %d · 格式错误 %d' % (data['checksum_bad'], data['malformed']))
            self.fields.delete(*self.fields.get_children())
            self.cursor_var.set('等待GPIMU；采样数是语句内聚合数量，不按RAW计数器判断丢帧')
        if self._cursor_time is not None and times and times[0] <= self._cursor_time <= times[-1]:
            self.inspect_time(self._cursor_time)
        else:
            self._cursor_time = None
            for _, panel in self.panels:
                panel.show_cursor(None)
            if times:
                self.cursor_var.set('GP窗口 %.4f ~ %.4f s；移动图中游标读取同一时刻字段与六轴值' % (times[0], times[-1]))

    @staticmethod
    def _number(value):
        return '%.7g' % value if np.isfinite(value) else '—'

    def _show_fields(self, index):
        prepared = self._prepared
        row = prepared['rows'][index]
        fields = [('GPIMU分析时间(s)', '%.9f' % row[0]), ('IMU时间字段(s)', '%.9f' % row[2]), ('GPS周', row[3]), ('GPS周内秒', '%.9f' % row[4]),
                  ('采样数 / 语句（非递增帧计数）', row[5]), ('语句间隔(ms)', row[13]),
                  ('类型/标志字段（非IMU_Status）', row[14]), ('温度原始值', row[12])]
        fields += [(name+'原始计数', '0x%08X (%d)' % (int(row[6+k]) & 0xffffffff, int(row[6+k])))
                   for k, name in enumerate(('GX','GY','GZ','AX','AY','AZ'))]
        self.fields.delete(*self.fields.get_children())
        for name, value in fields:
            self.fields.insert('', 'end', values=(name, self._number(value) if isinstance(value, (float, np.floating)) else value))

    def inspect_time(self, target):
        if not self._prepared or not self._prepared['time']:
            return
        times = self._prepared['time']
        index = min(len(times)-1, bisect_left(times, target))
        if index and abs(times[index-1]-target) < abs(times[index]-target):
            index -= 1
        self._cursor_time = times[index]
        values = self._prepared['values'][index]
        self.cursor_var.set('GP ' + self._prepared['axis'].cursor_text(times[index]) +
                            ('  GXYZ=%+.5f/%+.5f/%+.5f deg/s  AXYZ=%+.5f/%+.5f/%+.5f m/s²') % tuple(values))
        for _, panel in self.panels:
            panel.show_cursor(times[index])
        self._show_fields(index)

    def export_csv(self, path=None):
        if self._prepared is None or not len(self._prepared['rows']):
            self.status_var.set('当前窗没有可导出的GPIMU数据')
            return
        data = self._prepared  # Snapshot the visible window before the modal file chooser.
        if path is None:
            path = filedialog.asksaveasfilename(parent=self.parent.winfo_toplevel(), title='导出当前可见GPIMU窗口',
                      initialfile='gpimu_window.csv', defaultextension='.csv', filetypes=(('CSV','*.csv'),))
        if not path:
            return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as output:
                writer = csv.writer(output)
                writer.writerow(['gp_model','t_s','gps_week','gps_sow','imu_time','samples_per_sentence','temperature_C','interval_ms']+
                                ['gp_'+axis for axis in ('gx_deg_s','gy_deg_s','gz_deg_s','ax_m_s2','ay_m_s2','az_m_s2')]+
                                ['raw_'+axis for axis in ('gx_deg_s','gy_deg_s','gz_deg_s','ax_m_s2','ay_m_s2','az_m_s2')]+['raw_minus_gp_time_ms'])
                for k, row in enumerate(data['rows']):
                    writer.writerow([data['model'], row[0], row[3], row[4], row[2], row[5], data['temperature'][k], row[13]]+
                                    list(data['values'][k])+list(data['raw'][k])+[data['offsets'][k]])
            self.status_var.set('当前窗口已导出：'+str(path))
        except (OSError, ValueError) as error:
            self.status_var.set('导出失败：'+str(error))

    def _destroyed(self, event):
        if event.widget is self.parent:
            self.closed = True
            self._cancel.set()
            if self._after is not None:
                self.parent.after_cancel(self._after)
