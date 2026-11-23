"""Manual static-segment IMU analysis; background jobs communicate through a queue."""
import copy
import math
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog

import numpy as np

from imu_performance import analyze_static, display_data, export_analysis, AXES
from performance_data import check_cancel, load_recording
from plot_navigation import PlotNavigation, clip_polyline
from time_axis import TimeAxis, AXIS_MODES

TIMING_MODES = {'连续采样（容许抖动）': 'continuous', '严格设备时间（2%）': 'strict'}


class LogAllanPlot(object):
    COLORS = ('#d34c4c', '#20956b', '#3c70d6')

    def __init__(self, canvas, title):
        self.canvas, self.title = canvas, title
        self.tau, self.values = [], None
        self.unit = ''
        self.message = '选择静态段并点击“开始分析”'
        self.navigation = PlotNavigation(canvas, self.redraw)
        canvas.bind('<Configure>', lambda e: self.redraw())
        canvas.bind('<Map>', lambda e: self.redraw())

    def set_data(self, tau, values, unit, message=''):
        if self.tau is not tau or self.unit != unit:
            self.navigation.base = self.navigation.view = None
        self.tau, self.values, self.unit, self.message = tau, values, unit, message
        self.redraw()

    def redraw(self):
        c = self.canvas
        w, h = c.winfo_width(), c.winfo_height()
        if w < 100 or h < 100 or not c.winfo_ismapped():
            return
        c.delete('all')
        c.create_text(16, 17, text=self.title, anchor='w', fill='#172b49', font=('Microsoft YaHei', 11, 'bold'))
        c.create_text(w - 16, 17, text=self.unit, anchor='e', fill='#63748b', font=('Consolas', 10))
        for k, axis in enumerate('XYZ'):
            c.create_line(18 + k*72, 40, 36 + k*72, 40, fill=self.COLORS[k], width=2)
            c.create_text(42 + k*72, 40, text=axis, anchor='w', fill='#44536b')
        if self.values is None:
            c.create_text(w/2, h/2, text=self.message, width=w-100, fill='#63748b', font=('Microsoft YaHei', 11))
            return
        tau, data = np.asarray(self.tau), np.asarray(self.values)
        valid = np.isfinite(data) & (data > 0)
        if not valid.any():
            c.create_text(w/2, h/2, text='结果为零或无正值，无法绘制对数曲线', fill='#63748b')
            return
        xmin, xmax = math.floor(np.log10(tau.min())), math.ceil(np.log10(tau.max()))
        ymin, ymax = math.floor(np.log10(data[valid].min())), math.ceil(np.log10(data[valid].max()))
        xmax, ymax = max(xmin + 1, xmax), max(ymin + 1, ymax)
        left, top, width, height = 78, 62, w - 102, h - 110
        xmin, xmax, ymin, ymax = self.navigation.configure(
            (xmin, xmax, ymin, ymax), (left, top, width, height))
        def x(value):
            return left + (np.log10(value) - xmin) / (xmax - xmin) * width
        def y(value):
            return top + height - (np.log10(value) - ymin) / (ymax - ymin) * height
        for exponent in np.linspace(xmin, xmax, 5):
            xx = float(x(10.**exponent))
            c.create_line(xx, top, xx, top+height, fill='#e8edf4')
            c.create_text(xx, top+height+14, text='%g' % 10.**exponent, fill='#63748b', font=('Consolas', 9))
        for exponent in np.linspace(ymin, ymax, 5):
            yy = float(y(10.**exponent))
            c.create_line(left, yy, left+width, yy, fill='#e8edf4')
            c.create_text(left-8, yy, text='%.2g' % 10.**exponent, anchor='e', fill='#63748b', font=('Consolas', 9))
        c.create_text(left+width/2, h-12, text='聚类时间 τ (s) · 对数坐标', fill='#63748b', font=('Microsoft YaHei', 10))
        for axis in range(3):
            segment = []
            def draw_segment():
                points = list(zip(segment[::2], segment[1::2]))
                for clipped in clip_polyline(points, left, left+width, top, top+height):
                    c.create_line(*[v for point in clipped for v in point], fill=self.COLORS[axis], width=2)
            for index in range(len(tau)):
                if valid[index, axis]:
                    segment.extend((float(x(tau[index])), float(y(data[index, axis]))))
                else:
                    if len(segment) >= 4:
                        draw_segment()
                    segment = []
            if len(segment) >= 4:
                draw_segment()
            elif len(segment) == 2:
                xx, yy = segment
                if left <= xx <= left+width and top <= yy <= top+height:
                    c.create_oval(xx-2, yy-2, xx+2, yy+2, fill=self.COLORS[axis], outline='')


class PerformanceWindow(object):
    def __init__(self, app, plot_class):
        self.app = app
        self.window = tk.Toplevel(app.root)
        self.window.title('静态 IMU 性能分析 · Allan / 平滑 / 统计')
        self.window.geometry('1320x920')
        self.window.minsize(1100, 760)
        self.window.configure(bg=app.BG)
        self.window.protocol('WM_DELETE_WINDOW', self.window.destroy)
        self.window.bind('<Destroy>', self._destroyed, add='+')
        self.window.bind('<Escape>', self._escape)
        self.source_data = None
        self.result = None
        self.busy = False
        self.closed = False
        self.cancel_event = threading.Event()
        self.messages = queue.Queue()
        self._after = None
        self._job_session = None
        self._display = None
        self.source_var = tk.StringVar(master=self.window)
        self.start_var = tk.StringVar(master=self.window, value='0')
        self.end_var = tk.StringVar(master=self.window)
        self.smooth_var = tk.StringVar(master=self.window, value='10')
        self.timing_var = tk.StringVar(master=self.window, value='连续采样（容许抖动）')
        self.confirmed_var = tk.BooleanVar(master=self.window, value=False)
        self.status_var = tk.StringVar(master=self.window, value='选取静态时间段后开始分析；结束留空表示快照末尾')
        self.quality_var = tk.StringVar(master=self.window, value='等待分析。静态条件由用户确认，采样连续性由程序检查。')
        self.metric_var = tk.StringVar(master=self.window, value='Allan偏差 σ')
        header = tk.Frame(self.window, bg=app.INK)
        header.pack(fill='x')
        tk.Label(header, text='静态 IMU 性能分析', bg=app.INK, fg='white',
                 font=('Microsoft YaHei', 14, 'bold')).pack(side='left', padx=16, pady=9)
        tk.Label(header, text='图表：滚轮缩放 · Shift 时间 / Ctrl 纵轴 · 拖动平移 · 双击恢复', bg=app.INK, fg='#b9cbe4',
                 font=('Microsoft YaHei', 10)).pack(side='right', padx=16)
        sourcebar = tk.Frame(self.window, bg='white', padx=12, pady=5)
        sourcebar.pack(fill='x')
        self.current_button = ttk.Button(sourcebar, text='使用当前回放 / 录制', command=self.use_current)
        self.current_button.pack(side='left')
        self.file_button = ttk.Button(sourcebar, text='载入 DAT / CSV…', command=self.load_file)
        self.file_button.pack(side='left', padx=8)
        tk.Label(sourcebar, textvariable=self.source_var, bg='white', fg=app.MUTED,
                 anchor='w', font=('Microsoft YaHei', 10)).pack(side='left', fill='x', expand=True)
        options = tk.Frame(self.window, bg='white', padx=12, pady=5)
        options.pack(fill='x')
        for caption, variable in (('开始(s)', self.start_var), ('结束(s)', self.end_var), ('平滑(s)', self.smooth_var)):
            tk.Label(options, text=caption, bg='white', fg=app.INK).pack(side='left', padx=(0, 6))
            ttk.Entry(options, textvariable=variable, width=10).pack(side='left', padx=(0, 16))
        tk.Label(options, text='时间检查', bg='white', fg=app.INK).pack(side='left', padx=(0, 5))
        ttk.Combobox(options, textvariable=self.timing_var, values=tuple(TIMING_MODES), width=22,
                     state='readonly').pack(side='left', padx=(0, 8))
        ttk.Checkbutton(options, text='已确认所选时间段静止（用于 Allan）',
                        variable=self.confirmed_var).pack(side='left', padx=8)
        actions = tk.Frame(self.window, bg=app.BG, padx=12, pady=6)
        actions.pack(fill='x')
        self.run_button = ttk.Button(actions, text='开始分析', style='Accent.TButton', command=self.start_analysis)
        self.run_button.pack(side='left')
        self.cancel_button = ttk.Button(actions, text='取消计算', command=self.cancel_work, state='disabled')
        self.cancel_button.pack(side='left', padx=8)
        self.export_button = ttk.Button(actions, text='导出结果…', command=self.export_result, state='disabled')
        self.export_button.pack(side='left')
        tk.Label(actions, textvariable=self.status_var, bg=app.BG, fg=app.MUTED, anchor='w',
                 font=('Microsoft YaHei', 10)).pack(side='left', padx=14, fill='x', expand=True)
        axisbar = tk.Frame(self.window, bg=app.BG, padx=12, pady=4)
        axisbar.pack(fill='x')
        tk.Label(axisbar, text='时序图横轴', bg=app.BG, fg=app.MUTED).pack(side='left', padx=(0, 8))
        axisbox = ttk.Combobox(axisbar, textvariable=app.time_axis_var, values=tuple(AXIS_MODES),
                               state='readonly', width=18)
        axisbox.pack(side='left')
        axisbox.bind('<<ComboboxSelected>>', app.on_axis_changed)
        tk.Label(axisbar, text='GPS 时间缺失时显示 —；Allan 横轴保持聚类时间 τ',
                 bg=app.BG, fg=app.MUTED).pack(side='left', padx=12)
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=(0, 8))
        stats_page = tk.Frame(self.notebook, bg='white', padx=12, pady=12)
        smooth_page = tk.Frame(self.notebook, bg=app.BG)
        allan_page = tk.Frame(self.notebook, bg=app.BG)
        self.notebook.add(stats_page, text='质量检查与统计')
        self.notebook.add(smooth_page, text='六轴平滑')
        self.notebook.add(allan_page, text='Allan 分析')
        self.notebook.bind('<<NotebookTabChanged>>', lambda e: self._redraw())
        self.quality_label = tk.Label(stats_page, textvariable=self.quality_var, bg='#eef4fb',
                                      fg=app.INK, anchor='w', justify='left', wraplength=1000,
                                      padx=12, pady=12, font=('Microsoft YaHei', 11))
        self.quality_label.pack(fill='x', pady=(0, 14))
        stats_page.bind('<Configure>', lambda e: self.quality_label.configure(wraplength=max(100, e.width-60)))
        columns = ('axis', 'unit', 'mean', 'std', 'pp', 'smooth_std', 'smooth_pp', 'allan_min', 'allan_min_tau')
        labels = ('轴', '单位', '均值', '标准差(样本)', '原始峰峰值', '平滑标准差', '平滑峰峰值', '最小Allan偏差', '对应τ(s)')
        tableframe = tk.Frame(stats_page, bg='white')
        tableframe.pack(fill='x')
        self.table = ttk.Treeview(tableframe, columns=columns, show='headings', height=7)
        for key, label in zip(columns, labels):
            self.table.heading(key, text=label)
            self.table.column(key, width=118 if key not in ('axis', 'unit') else 76, minwidth=70, anchor='e')
        scroll = ttk.Scrollbar(tableframe, orient='horizontal', command=self.table.xview)
        self.table.configure(xscrollcommand=scroll.set)
        self.table.pack(fill='x')
        scroll.pack(fill='x')
        self.details = tk.Text(stats_page, bg='white', fg=app.INK, relief='flat', wrap='word',
                               height=10, font=('Microsoft YaHei', 10), padx=8, pady=12)
        self.details.pack(fill='both', expand=True, pady=(12, 0))
        self._set_details('分析使用配置型号对应的原始物理量，不使用姿态解算的自动去零偏输出。\n'
                          '可直接分析当前会话完整历史，或单独载入录制文件；载入分析不会替换正在运行的数据源。\n'
                          '10秒平滑默认要求完整10秒数据；不足部分留空。\n'
                          '分析时间轴累计设备间隔；时间回退或超大跳变处按名义周期推进并标记异常。')
        panels = []
        for row in range(2):
            smooth_page.rowconfigure(row, weight=1, uniform='rows')
        for column in range(3):
            smooth_page.columnconfigure(column, weight=1, uniform='columns')
        for k, axis in enumerate(AXES):
            canvas = tk.Canvas(smooth_page, width=100, height=100, bg='white', highlightthickness=1,
                               highlightbackground='#cfd8e4')
            canvas.grid(row=k // 3, column=k % 3, sticky='nsew', padx=3, pady=3)
            panel = plot_class(canvas, axis.upper() + (' 角速度' if k < 3 else ' 加速度'),
                               'deg/s' if k < 3 else 'm/s²', ['原始', '10s均值'],
                               colors=('#237db6', '#dc792f'), min_span=.0001)
            panel.value_decimals = 6
            panel.bind_navigation()
            canvas.bind('<Configure>', lambda e, index=k, p=panel: self._resize_time(e, index, p))
            canvas.bind('<Map>', lambda e, index=k: self._draw_time_panel(index))
            panels.append((canvas, panel))
        self.time_panels = tuple(panels)
        metricbar = tk.Frame(allan_page, bg=app.BG, padx=8, pady=6)
        metricbar.pack(fill='x')
        metric = ttk.Combobox(metricbar, textvariable=self.metric_var, values=('Allan偏差 σ', 'Allan方差 σ²'),
                              state='readonly', width=18)
        metric.pack(side='left')
        metric.bind('<<ComboboxSelected>>', lambda e: self._redraw_allan())
        tk.Label(metricbar, text='重叠估计 · 原始未平滑数据 · 最大τ≤选段时长/10',
                 bg=app.BG, fg=app.MUTED, font=('Microsoft YaHei', 10)).pack(side='left', padx=14)
        charts = []
        for title in ('陀螺', '加速度计'):
            canvas = tk.Canvas(allan_page, width=100, height=100, bg='white', highlightthickness=1,
                               highlightbackground='#cfd8e4')
            canvas.pack(fill='both', expand=True, padx=3, pady=3)
            charts.append(LogAllanPlot(canvas, title))
        self.allan_plots = tuple(charts)
        tk.Label(self.window, text='陀螺均值包含地球自转及安装影响；单位置加速度均值包含重力。'
                 'Allan曲线最小值不直接等同于零偏不稳定性。', bg=app.BG, fg=app.MUTED,
                 anchor='w', font=('Microsoft YaHei', 9), padx=12, pady=4).pack(fill='x')
        self.refresh_source()
        self._after = self.window.after(100, self._poll)

    def _set_details(self, text):
        self.details.configure(state='normal')
        self.details.delete('1.0', 'end')
        self.details.insert('1.0', text)
        self.details.configure(state='disabled')

    def refresh_source(self):
        if self.closed:
            return
        if self.source_data is not None:
            self.source_var.set('文件：%s · %d样本' % (Path(self.source_data.source).name, len(self.source_data.records)))
        else:
            info = self.app.sess.analysis_archive.info()
            self.source_var.set('当前会话 · %d样本 · %.2fs%s' % (info['count'], info['duration'],
                                ' · 缓存错误：' + info['error'] if info['error'] else ''))

    def use_current(self):
        if self.busy:
            return
        self.source_data = None
        self.confirmed_var.set(False)
        self.start_var.set('0')
        self.end_var.set('')
        self._clear_result()
        self.refresh_source()

    def _clear_result(self):
        self.result, self._display = None, None
        self._axis_result = self._result_time_axis = None
        self.table.delete(*self.table.get_children())
        self.export_button.configure(state='disabled')
        self.quality_var.set('等待分析；请确认静态段和起止时间。')
        self._set_details('选择数据源及时间范围后，点击“开始分析”。')
        self._redraw()

    def on_session_changed(self):
        if self.source_data is None:
            if self.busy and self._job_session is not None:
                self.cancel_work()
            self._clear_result()
            self.start_var.set('0')
            self.end_var.set('')
            self.confirmed_var.set(False)
        self.refresh_source()

    def _start_job(self, kind, function, session=None):
        if self.busy or self.closed:
            return
        self.busy = True
        self.cancel_event = threading.Event()
        self._job_session = session
        self.status_var.set({'load': '正在载入完整文件…', 'analyze': '正在分析所选段…', 'export': '正在导出结果…'}[kind])
        if kind == 'analyze':
            self.quality_var.set('正在分析本次选段，请稍候。下方表格暂保留上一次结果，计算完成后更新。')
        for button in (self.run_button, self.current_button, self.file_button, self.export_button):
            button.configure(state='disabled')
        self.cancel_button.configure(state='normal')
        cancel = self.cancel_event
        messages = self.messages
        def run():
            try:
                value = function(cancel)
                check_cancel(cancel)
                messages.put((kind, value, None))
            except Exception as error:
                messages.put((kind, None, InterruptedError('分析已取消') if cancel.is_set() else error))
        threading.Thread(target=run, name='imu-performance-' + kind, daemon=True).start()

    def load_file(self, path=None):
        if self.busy:
            return
        if path is None:
            path = filedialog.askopenfilename(parent=self.window, title='载入静态IMU数据',
                        filetypes=(('IMU数据', '*.dat *.bin *.log *.csv'), ('所有文件', '*.*')))
        if path:
            cfg = copy.deepcopy(self.app.sess.analysis_cfg)
            formats = self.app.formats
            self._start_job('load', lambda cancel: load_recording(path, cfg, cancel, formats))

    def start_analysis(self):
        if self.busy:
            return
        try:
            start = float(self.start_var.get()) if self.start_var.get().strip() else None
            end = float(self.end_var.get()) if self.end_var.get().strip() else None
            smooth = float(self.smooth_var.get())
            if self.timing_var.get() not in TIMING_MODES:
                raise ValueError('请选择有效时间检查模式')
            timing_mode = TIMING_MODES[self.timing_var.get()]
            if ((start is not None and not math.isfinite(start)) or (end is not None and not math.isfinite(end))
                    or not math.isfinite(smooth) or smooth <= 0 or (start is not None and end is not None and start >= end)):
                raise ValueError('起止时间需有效且递增，平滑时间需大于0')
        except ValueError as error:
            self.status_var.set('参数错误：' + str(error))
            return
        confirmed = bool(self.confirmed_var.get())
        data = self.source_data
        session = self.app.sess if data is None else None
        metadata = {'crc_errors': self.app.sess.parser.n_crc + self.app.sess.parser.n_text_crc_bad,
                    'model': self.app.imu_var.get(),
                    'model_scales': {key: self.app.sess.analysis_cfg[key] for key in ('gyro_lsb', 'accel_lsb', 'freq')}}
        def calculate(cancel):
            source = session.analysis_archive.read(start, end, cancel, metadata) if session else data
            result = analyze_static(source, start=None if session else start, end=None if session else end,
                                    confirmed_static=confirmed, smooth_seconds=smooth, cancel=cancel, timing_mode=timing_mode)
            result['display'] = display_data(result)
            return result
        self._start_job('analyze', calculate, session)

    def export_result(self, prefix=None):
        if self.busy or self.result is None:
            return
        if prefix is None:
            path = filedialog.asksaveasfilename(parent=self.window, title='导出性能分析结果',
                        initialfile='imu_performance_summary.csv', defaultextension='.csv',
                        filetypes=(('统计CSV', '*.csv'),))
            if not path:
                return
            prefix = str(Path(path).with_suffix(''))
            if prefix.endswith('_summary'):
                prefix = prefix[:-8]
        result = self.result
        self._start_job('export', lambda cancel: export_analysis(result, prefix, cancel))

    def cancel_work(self):
        self.cancel_event.set()
        if self.busy:
            self.status_var.set('正在取消…')

    def _poll(self):
        if self.closed:
            return
        try:
            kind, value, error = self.messages.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            for button in (self.run_button, self.current_button, self.file_button):
                button.configure(state='normal')
            self.cancel_button.configure(state='disabled')
            if error is not None:
                self.status_var.set('已取消' if isinstance(error, InterruptedError) else '操作失败：' + str(error))
                if kind == 'analyze':
                    self.quality_var.set('本次分析已取消或失败；表格若有内容，为上一次分析结果。')
            elif kind == 'load':
                self.source_data = value
                self._clear_result()
                self.start_var.set('0')
                self.end_var.set('')
                self.confirmed_var.set(False)
                self.refresh_source()
                self.status_var.set('文件已载入；按当前IMU型号换算，请选择静态段')
            elif kind == 'analyze':
                if self._job_session is None or self._job_session is self.app.sess:
                    self._show_result(value)
            elif kind == 'export':
                self.status_var.set('已导出统计 / Allan / 完整平滑数据 / 质量报告（4个文件）')
            self.export_button.configure(state='normal' if self.result is not None else 'disabled')
            self._job_session = None
        self._after = self.window.after(100, self._poll)

    @staticmethod
    def _number(value):
        return '%.7g' % value if np.isfinite(value) else '—'

    def _show_result(self, result):
        self.result = result
        for _, panel in self.time_panels:
            panel.cur = None
            panel.navigation.base = panel.navigation.view = None
        self._display = result['display']
        quality = result['quality']
        issues = quality['issues']
        self.quality_var.set(('Allan已计算：'+quality['allan_time_basis'] if quality['allan_allowed'] else
                              'Allan未计算：' + '；'.join(issues)) +
                             '\n%d样本 · %.3fs · 实测采样率 %s Hz · 故障位样本 %d · 源累计CRC错误 %d'
                             % (quality['sample_count'], quality['duration_s'],
                                self._number(quality['sample_rate_hz'] or np.nan), quality['fault_samples'],
                                quality['crc_errors_source_total']) +
                             '\n时间不连续 %d处 · 时间戳抖动 %d处 · 间隔 %s～%s ms · 累计偏离 %s ms' %
                             (quality['interval_anomalies'], quality['timestamp_jitter_count'],
                              self._number(quality['interval_min_ms'] if quality['interval_min_ms'] is not None else np.nan),
                              self._number(quality['interval_max_ms'] if quality['interval_max_ms'] is not None else np.nan),
                              self._number(quality['phase_error_max_ms'] if quality['phase_error_max_ms'] is not None else np.nan)))
        self.table.delete(*self.table.get_children())
        for k, axis in enumerate(AXES):
            values = [axis.upper(), 'deg/s' if k < 3 else 'm/s²']
            values += [self._number(result['stats'][key][k]) for key in
                       ('mean', 'std', 'pp', 'smooth_std', 'smooth_pp', 'allan_min', 'allan_min_tau')]
            self.table.insert('', 'end', values=values)
        self._set_details('数据源：%s\n选段快照：%.4f ~ %.4f s（设备间隔累计时间）\n平滑时间：%g s，起始不满完整窗口的部分留空。\n'
                          '计数连续性：%s；时间不连续：%d处。\n时间口径：%s；时间戳抖动%d处；采用周期%ss。\n%s\n'
                          '统计标准差采用N-1分母；Allan使用未平滑数据，不拼接缺口或重采样。\n'
                          '最小Allan偏差及对应τ仅描述本选段；不据此直接认定零偏不稳定性。'
                          % (result['source'], result['time'][0], result['time'][-1], result['smooth_seconds'],
                             str(quality['counter_anomalies']) + '处异常' if quality['counter_known'] else '状态未知',
                             quality['interval_anomalies'], quality['allan_time_basis'], quality['timestamp_jitter_count'],
                             self._number(quality['sample_period_s'] if quality['sample_period_s'] is not None else np.nan),
                             '\n'.join(result['motion_notes']) or '运动提示：未触发预设阈值；不替代人工静态确认。'))
        self.status_var.set('分析完成 · %.3f ~ %.3f s · %d样本' %
                            (result['time'][0], result['time'][-1], quality['sample_count']))
        self._redraw()

    def _resize_time(self, event, index, panel):
        panel.resize(event.width, event.height)
        self._draw_time_panel(index)

    def _draw_time_panel(self, index):
        canvas, panel = self.time_panels[index]
        panel.render_enabled = bool(canvas.winfo_ismapped())
        if self._display is None:
            panel.update([], [[], []])
            return
        times, values, smooth = self._display
        if getattr(self, '_axis_result', None) is not self.result:
            self._axis_result = self.result
            device_time = self.result.get('device_time')
            known = self.result['metadata'].get('gps_time_known', False) and device_time is not None
            self._result_time_axis = TimeAxis(times=self.result['time'],
                seconds=np.remainder(device_time, 604800.) if known else (),
                weeks=np.floor(device_time/604800.) if known else ())
        self._result_time_axis.mode = self.app.axis_mode()
        panel.time_axis = self._result_time_axis
        panel.time_range = (float(times[0]), max(float(times[0]) + .01, float(times[-1])))
        panel.series = ['原始', '%gs均值' % self.result['smooth_seconds']]
        panel.update(times.tolist(), [values[:, index].tolist(), smooth[:, index].tolist()])

    def _redraw_allan(self):
        if not hasattr(self, 'allan_plots'):
            return
        variance = self.metric_var.get() == 'Allan方差 σ²'
        allan = self.result['allan'] if self.result is not None else None
        for k, plot in enumerate(self.allan_plots):
            plot.title = ('陀螺' if k == 0 else '加速度计') + ' · ' + self.metric_var.get()
            unit = 'deg/s' if k == 0 else 'm/s²'
            if variance:
                unit = '(' + unit + ')²'
            plot.set_data(allan['tau'] if allan is not None else [],
                          allan['variance' if variance else 'deviation'][:, k*3:k*3+3] if allan is not None else None,
                          unit, '所选段未计算Allan，请查看“质量检查与统计”中的原因')

    def _redraw(self):
        if hasattr(self, 'time_panels'):
            for k in range(6):
                self._draw_time_panel(k)
        self._redraw_allan()

    def _escape(self, event):
        self.window.destroy()
        return 'break'

    def _destroyed(self, event):
        if event.widget is self.window:
            self.closed = True
            self.cancel_event.set()
            if self._after is not None:
                self.window.after_cancel(self._after)
            if self.app.performance_window is self:
                self.app.performance_window = None
