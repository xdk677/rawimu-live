"""Compact recording-time index and Tk replay transport controls (Python3.7)."""
from array import array
from bisect import bisect_right
import math
from pathlib import Path
import weakref
import tkinter as tk
from tkinter import ttk


def file_signature(path):
    path = Path(path).resolve()
    stat = path.stat()
    return str(path), stat.st_size, stat.st_mtime_ns


class ReplayIndex(object):
    def __init__(self, signature):
        self.signature = signature
        self.times, self.ends = array('d'), array('Q')
        self.gpimu_only = False

    @property
    def duration(self):
        return self.times[-1] if self.times else 0.

    def matches(self, path):
        try:
            return self.signature == file_signature(path)
        except OSError:
            return False

    def frame_at(self, seconds):
        return max(0, min(len(self.times)-1, bisect_right(self.times, seconds + 1e-6) - 1))


def build_replay_index(path, registry, frequency, stop=None, on_progress=None, parser_class=None):
    if not math.isfinite(frequency) or frequency <= 0:
        raise ValueError('回放采样率必须为正数')
    if parser_class is None:
        from rawimu_live import FrameParser
        parser_class = FrameParser
    signature = file_signature(path)
    index = ReplayIndex(signature)
    previous, elapsed = [None], [0.]
    def on_frame(spec, body):
        try:
            raw = spec['decode'](body)
            stamp = raw.get('week', 0) * 604800. + raw.get('sow', 0.)
        except (ValueError, TypeError, KeyError):
            return
        if index.times:
            delta = stamp - previous[0]
            elapsed[0] += delta if math.isfinite(delta) and 0 < delta <= 10. else 1. / frequency
        previous[0] = stamp
        index.times.append(elapsed[0])
        index.ends.append(parser.frame_end)
    parser = parser_class(on_frame, registry)
    gp_index = ReplayIndex(signature)
    gp_previous, gp_domain = [None], [None]
    def on_gpimu(raw):
        if gp_index.times:
            delta = raw['clock'] - gp_previous[0] if raw['gps_valid'] == gp_domain[0] else float('nan')
            elapsed_gp = gp_index.times[-1] + (delta if math.isfinite(delta) and 0 < delta <= 10. else 1./frequency)
        else:
            elapsed_gp = 0.
        gp_index.times.append(elapsed_gp)
        gp_index.ends.append(parser.frame_end)
        gp_previous[0], gp_domain[0] = raw['clock'], raw['gps_valid']
    parser.on_gpimu = on_gpimu
    with open(path, 'rb') as source:
        while True:
            if stop is not None and stop.is_set():
                raise InterruptedError('时间轴索引已取消')
            data = source.read(65536)
            if not data:
                parser.feed(b'', final=True)
                break
            parser.feed(data)
            if on_progress:
                on_progress(source.tell(), signature[1])
    if not index.matches(path):
        raise ValueError('回放文件正在变化，请停止录制后重新打开')
    if not index.times:
        if gp_index.times:
            gp_index.gpimu_only = True
            return gp_index
        raise ValueError('文件中未找到有效RAWIMU或GPIMU数据')
    return index


def format_elapsed(seconds):
    milliseconds = int(round(max(0., seconds) * 1000.))
    seconds, ms = divmod(milliseconds, 1000)
    minutes, sec = divmod(seconds, 60)
    hours, minute = divmod(minutes, 60)
    return ('%02d:%02d:%02d.%03d' % (hours, minute, sec, ms) if hours else
            '%02d:%02d.%03d' % (minute, sec, ms))


class ReplayTimeline(object):
    def __init__(self, parent, app):
        # Avoid an App -> transport -> App cycle: Tk variables must be released
        # on the UI thread, never by a playback-thread garbage collection.
        self.app = weakref.proxy(app)
        self.frame = tk.Frame(parent, bg='white')
        self.position_var = tk.DoubleVar(master=self.frame, value=0.)
        self.time_var = tk.StringVar(master=self.frame, value='00:00.000 / --:--.---')
        self.note_var = tk.StringVar(master=self.frame)
        self.dragging = False
        self.info = None
        tk.Label(self.frame, text='回放时间轴', bg='white', fg=app.INK,
                 font=('Microsoft YaHei', 10, 'bold')).pack(side='left', padx=(0, 10))
        self.pause_button = ttk.Button(self.frame, text='暂停回放', width=9, command=app.toggle_replay_pause)
        self.pause_button.pack(side='left', padx=(0, 8))
        self.scale = ttk.Scale(self.frame, from_=0., to=1., variable=self.position_var,
                               command=self._preview)
        self.scale.pack(side='left', fill='x', expand=True, padx=(0, 10))
        self.scale.bind('<ButtonPress-1>', self._begin_drag)
        self.scale.bind('<ButtonRelease-1>', self._commit_drag)
        for key in ('Left', 'Right', 'Home', 'End', 'Prior', 'Next'):
            self.scale.bind('<KeyRelease-' + key + '>', self._commit_keyboard)
        tk.Label(self.frame, textvariable=self.time_var, bg='white', fg=app.INK,
                 font=('Consolas', 10), width=27, anchor='w').pack(side='left')
        tk.Label(self.frame, textvariable=self.note_var, bg='white', fg=app.MUTED,
                 font=('Microsoft YaHei', 9), width=13, anchor='w').pack(side='left')

    def _enabled(self):
        return 'disabled' not in self.scale.state()

    def _begin_drag(self, event):
        if self._enabled():
            self.dragging = True

    def _preview(self, value):
        if self.dragging and self.info:
            self.time_var.set(format_elapsed(float(value)) + ' / ' + format_elapsed(self.info['duration']))

    def _commit_drag(self, event):
        if self.dragging:
            self.dragging = False
            if self._enabled():
                self.app.seek_replay(self.position_var.get())

    def _commit_keyboard(self, event):
        if self._enabled():
            self.app.seek_replay(self.position_var.get())

    def render(self, worker, recording=False):
        if worker is None:
            self.dragging = False
            self.frame.pack_forget()
            self.info = None
            self.position_var.set(0.)
            return
        if not self.frame.winfo_manager():
            self.frame.pack(fill='x', pady=(7, 0))
        info = worker.timeline()
        self.info = info
        ready = info['ready']
        self.scale.configure(to=max(.001, info['duration']))
        self.scale.state(['!disabled'] if ready and not recording else ['disabled'])
        self.pause_button.configure(state='normal' if worker.is_alive() or ready else 'disabled',
                    text='继续回放' if info['paused'] and worker.is_alive() else
                         '暂停回放' if worker.is_alive() else '从头播放')
        if not self.dragging:
            self.position_var.set(info['position'])
            self.time_var.set(format_elapsed(info['position']) + ' / ' +
                              (format_elapsed(info['duration']) if ready else '--:--.---'))
        self.note_var.set('CSV录制中·禁止跳转' if recording else '正在重算定位…' if info['seeking'] else
                          '索引 %d%%' % (100 * info['scan_progress']) if not ready else '拖动定位 / 可暂停')
