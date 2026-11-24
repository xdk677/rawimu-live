"""Independent 3x3 IMU analysis window, driven by the main application's snapshot."""
import math
from bisect import bisect_left
from collections import deque
import tkinter as tk
from tkinter import ttk
from time_axis import AXIS_MODES


def trailing_average(times, values, window_s=5.0):
    """Sample mean over trailing elapsed seconds; partial startup windows are allowed."""
    points = deque()
    total = 0.
    previous = None
    output = []
    for stamp, value in zip(times, values):
        if not math.isfinite(stamp) or not math.isfinite(value) or (previous is not None and stamp < previous):
            points.clear()
            total = 0.
        previous = stamp
        if not math.isfinite(stamp) or not math.isfinite(value):
            output.append(float('nan'))
            continue
        points.append((stamp, value))
        total += value
        while points and points[0][0] < stamp - window_s:
            total -= points.popleft()[1]
        output.append(total / len(points))
    return output


def centered_sample_average(values, window=500):
    """MATLAB movmean(values,k): even k has k/2 previous and k/2-1 next."""
    import numpy as np
    data = np.asarray(values, dtype=float)
    size = len(data)
    if not size:
        return []
    index = np.arange(size)
    before, after = window//2, (window-1)//2
    left, right = np.maximum(0, index-before), np.minimum(size, index+after+1)
    finite = np.isfinite(data)
    sums = np.concatenate(([0.], np.cumsum(np.where(finite, data, 0.))))
    counts = np.concatenate(([0], np.cumsum(finite)))
    result = (sums[right]-sums[left])/(right-left)
    result[counts[right]-counts[left] != right-left] = np.nan
    return result.tolist()


class AxisAnalysisWindow(object):
    def __init__(self, app, plot_class):
        self.app = app
        self.window = tk.Toplevel(app.root)
        self.window.title('IMU 六轴独立分析 · 时间 / 温度 / 周计数')
        self.window.geometry('1260x900')
        self.window.minsize(1000, 720)
        self.window.configure(bg=app.BG)
        self.window.protocol('WM_DELETE_WINDOW', self.window.destroy)
        self.window.bind('<Destroy>', self._destroyed, add='+')
        self.window.bind('<Map>', lambda e: app._mark_dirty(), add='+')
        self.window.bind('<F6>', lambda e: app.toggle_freeze())
        self.window.bind('<Escape>', self._close)
        header = tk.Frame(self.window, bg=app.INK)
        header.pack(fill='x')
        tk.Label(header, text='IMU 六轴独立分析', bg=app.INK, fg='white',
                 font=('Microsoft YaHei', 13, 'bold')).pack(side='left', padx=16, pady=10)
        toolbar = tk.Frame(self.window, bg='white', padx=12, pady=6)
        toolbar.pack(fill='x')
        tk.Label(toolbar, text='时间窗', bg='white', fg=app.MUTED).pack(side='left', padx=(0, 6))
        box = ttk.Combobox(toolbar, textvariable=app.window_var, values=('10', '30', '60'),
                           state='readonly', width=4)
        box.pack(side='left')
        box.bind('<<ComboboxSelected>>', lambda e: app.on_window())
        tk.Label(toolbar, text='秒', bg='white', fg=app.MUTED).pack(side='left', padx=6)
        ttk.Button(toolbar, textvariable=app.freeze_text, command=app.toggle_freeze).pack(side='left', padx=8)
        axisbox = ttk.Combobox(toolbar, textvariable=app.time_axis_var, values=tuple(AXIS_MODES),
                               state='readonly', width=18)
        axisbox.pack(side='left', padx=6)
        axisbox.bind('<<ComboboxSelected>>', app.on_axis_changed)
        tk.Label(toolbar, text='蓝：原始 / 橙：5s均值 · 冻结后滚轮缩放、拖动平移、双击恢复', bg='white',
                 fg=app.MUTED, font=('Microsoft YaHei', 10)).pack(side='left', padx=10)
        options = tk.Frame(self.window, bg='white', padx=12, pady=4)
        options.pack(fill='x')
        box = ttk.Combobox(options, textvariable=app.measurement_var, values=('原始物理量', '解算补偿量'),
                           state='readonly', width=14)
        box.pack(side='left')
        box.bind('<<ComboboxSelected>>', app.on_measurement_changed)
        self.smoothing_var = tk.StringVar(master=self.window, value='MATLAB 居中5s' if app.cfg.get('imu_model') == 'M21' else '实时后向5s')
        smoothbox = ttk.Combobox(options, textvariable=self.smoothing_var, values=('MATLAB 居中5s', '实时后向5s'),
                                 state='readonly', width=16)
        smoothbox.pack(side='left', padx=10)
        smoothbox.bind('<<ComboboxSelected>>', lambda e: app._mark_dirty())
        tk.Label(options, text='居中均值在右端使用已有样本；新数据到达后更新，冻结后固定。',
                 bg='white', fg=app.MUTED).pack(side='left')
        grid = tk.Frame(self.window, bg=app.BG)
        grid.pack(fill='both', expand=True, padx=6, pady=6)
        for k in range(3):
            grid.rowconfigure(k, weight=1, uniform='rows')
            grid.columnconfigure(k, weight=1, uniform='columns')
        specs = [('时间间隔', 'ms'), ('温度', '°C'), ('周计数 · 原始 week', 'week')]
        specs += [(axis + ' 角速度 · 去零偏', 'deg/s') for axis in 'XYZ']
        specs += [(axis + ' 加速度', 'm/s²') for axis in 'XYZ']
        panels = []
        for k, (title, unit) in enumerate(specs):
            canvas = tk.Canvas(grid, bg='white', width=100, height=100, highlightthickness=1,
                               highlightbackground='#cfd8e4')
            canvas.grid(row=k // 3, column=k % 3, sticky='nsew', padx=3, pady=3)
            panel = plot_class(canvas, title, unit, ['原始', '5s均值'] if k >= 3 else ['Value'],
                               colors=('#237db6', '#dc792f'), min_span=.0001 if k >= 3 else .02)
            if k >= 3:
                panel.legend_values = False
            if k == 2:
                panel.integer_values, panel.stepped = True, True
                panel.min_span = 4.
            canvas.bind('<Configure>', lambda e, p=panel: app.on_plot_resize(e, p))
            panel.bind_cursor(app.inspect_time)
            panel.bind_navigation(app)
            panels.append((canvas, panel))
        self.panels = tuple(panels)
        self.cursor_var = tk.StringVar(master=self.window)
        self.reset_cursor()
        tk.Label(self.window, textvariable=self.cursor_var, bg='white', fg=app.INK,
                 anchor='w', font=('Consolas', 10), padx=12, pady=5).pack(fill='x')
        tk.Label(self.window, text='μ：当前时间窗所选物理量均值；PP：窗内5秒均值峰峰值。'
                 '居中模式与MATLAB movmean一致；后向模式按经过秒数统计。共用冻结和游标。',
                 bg=app.BG, fg=app.MUTED, anchor='w', font=('Microsoft YaHei', 9),
                 padx=12, pady=4).pack(fill='x')

    def _close(self, event):
        self.window.destroy()
        return 'break'

    def _destroyed(self, event):
        if event.widget is self.window and self.app.axes_window is self:
            self.app.axes_window = None

    def reset_cursor(self):
        self.cursor_var.set('游标：在任意图表内移动鼠标，读取同一时刻的六轴样本')

    def update(self, times, channels, status, start, time_range):
        if not self.window.winfo_ismapped():
            return
        visible_times = times[start:]
        sources = [status['time_interval_ms'], status['temperature'], status['week']]
        matlab_clock = (self.app.cfg.get('imu_model') == 'M21' and self.app.measurement_var.get() == '原始物理量')
        if matlab_clock and any(math.isfinite(value) for value in status.get('header_interval_ms', [])):
            sources[0] = status['header_interval_ms']
            self.panels[0][1].title = '时间间隔 · MATLAB 报文头时间'
        else:
            self.panels[0][1].title = '时间间隔 · IMU 测量时间'
        # Include five seconds before the displayed window to avoid restarting its mean at the left edge.
        context_start = max(0, bisect_left(times, visible_times[0] - 5.)) if visible_times else 0
        context_times = times[context_start:]
        for k, (canvas, panel) in enumerate(self.panels):
            panel.time_range = time_range
            panel.time_axis = self.app.chart_time_axis
            panel.render_enabled = bool(canvas.winfo_ismapped())
            if k < 3:
                ys = [sources[k]]
            else:
                values = channels[k][start:]
                if self.smoothing_var.get() == 'MATLAB 居中5s':
                    averaged = centered_sample_average(channels[k], max(1, int(round(self.app.cfg['freq']*5))))[start:]
                else:
                    averaged = trailing_average(context_times, channels[k][context_start:])[start - context_start:]
                # The preceding sample retained for line clipping is excluded from visible statistics.
                first_visible = bisect_left(visible_times, time_range[0])
                finite = [v for v in values[first_visible:] if math.isfinite(v)]
                smooth = [v for v in averaged[first_visible:] if math.isfinite(v)]
                mean = '%+.6g' % (math.fsum(finite) / len(finite)) if finite else '—'
                pp = '%.6g' % (max(smooth) - min(smooth)) if smooth else '—'
                panel.series = ['原始 μ=' + mean, '5s均值 PP=' + pp]
                panel.title = 'XYZ'[(k-3)%3] + (' 角速度' if k < 6 else ' 加速度') + ' · ' + self.app.measurement_var.get()
                ys = [values, averaged]
            panel.update(visible_times, ys)

    def inspect(self, index):
        times = self.panels[3][1]._last_t
        if not 0 <= index < len(times):
            return
        values = [panel._last_ys[0][index] for _, panel in self.panels[3:]]
        self.cursor_var.set(self.app.chart_time_axis.cursor_text(times[index]) +
                            ('    GX=%+.5f  GY=%+.5f  GZ=%+.5f deg/s    '
                             'AX=%+.5f  AY=%+.5f  AZ=%+.5f m/s²') % tuple(values))
