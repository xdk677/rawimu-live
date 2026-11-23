"""Label elapsed plotting coordinates with the original measured GPS clock."""
from bisect import bisect_left
import math

AXIS_MODES = {'相对时间 (s)': 'elapsed', 'GPS 周内秒 (s)': 'sow'}


class TimeAxis(object):
    def __init__(self, mode='elapsed', times=(), seconds=(), weeks=()):
        self.mode = mode
        self.times, self.seconds, self.weeks = times, seconds, weeks

    def _valid(self, index):
        return (0 <= index < min(len(self.times), len(self.seconds), len(self.weeks)) and
                math.isfinite(self.weeks[index]) and self.weeks[index] > 0 and
                math.isfinite(self.seconds[index]) and 0 <= self.seconds[index] < 604800.)

    def _clock_at(self, value):
        count = min(len(self.times), len(self.seconds), len(self.weeks))
        if not count or not math.isfinite(value):
            return float('nan'), None
        index = min(count-1, bisect_left(self.times, value))
        if self.times[index] == value:
            return (self.seconds[index], int(self.weeks[index])) if self._valid(index) else (float('nan'), None)
        if index == 0 or value > self.times[count-1]:
            index = 0 if index == 0 else count-1
            if not self._valid(index):
                return float('nan'), None
            seconds = self.seconds[index] + value-self.times[index]
            rollover = math.floor(seconds/604800.)
            return seconds % 604800., int(self.weeks[index])+rollover
        first, last = index-1, index
        if not self._valid(first) or not self._valid(last):
            return float('nan'), None
        elapsed = self.times[last]-self.times[first]
        clock = (self.weeks[last]-self.weeks[first])*604800. + self.seconds[last]-self.seconds[first]
        if elapsed <= 0 or clock <= 0 or abs(clock-elapsed) > max(.002, abs(elapsed)*.2):
            return float('nan'), None
        seconds = self.seconds[first] + (value-self.times[first])/elapsed*clock
        return seconds % 604800., int(self.weeks[first])+math.floor(seconds/604800.)

    def seconds_at(self, value):
        return self._clock_at(value)[0]

    def tick_text(self, value, step):
        seconds = self.seconds_at(value) if self.mode == 'sow' else value
        if not math.isfinite(seconds):
            return '—'
        digits = max(0, min(6, int(-math.floor(math.log10(max(step, 1e-6))))))
        return ('%.*f s' % (digits, seconds))

    def cursor_text(self, value):
        if self.mode != 'sow':
            return 't=%.4f s' % value
        seconds, week = self._clock_at(value)
        return ('GPS W%d / SOW=%.4f s' % (week, seconds)) if week is not None else 'GPS 周内秒不可用 · t=%.4f s' % value
