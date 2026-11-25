"""Full-session, disk-backed precalibration samples and offline recording import."""
import csv
import copy
from pathlib import Path
import math
import struct
import tempfile
import threading

ROW = struct.Struct('<13d')
# cumulative device intervals, device time, GX/GY/GZ/AX/AY/AZ, T, dt, nominal Hz, status, revision


def check_cancel(cancel):
    if cancel is not None and cancel.is_set():
        raise InterruptedError('分析已取消')


class AnalysisData(object):
    def __init__(self, records, source, metadata=None):
        self.records = records
        self.source = str(source)
        self.metadata = dict(metadata or {})


class SampleArchive(object):
    def __init__(self, source='当前会话'):
        self.source = source
        self.lock = threading.RLock()
        self.file = None
        self.count = 0
        self.elapsed = 0.
        self.last_time = 0.
        self.error = ''
        self.closed = False
        self.model_configs = {}

    def append_values(self, values, temperature, interval, rate, status, device_time, revision=0):
        with self.lock:
            if self.closed or self.error:
                return
            try:
                if self.file is None:
                    self.file = tempfile.TemporaryFile(mode='w+b')
                if self.file.tell() != self.count * ROW.size:
                    self.file.seek(self.count * ROW.size)
                if self.count:
                    # Preserve measured seconds even when the nominal model rate differs.
                    # Clock jumps use a nominal step for selection only; quality rejects them.
                    self.elapsed += interval if math.isfinite(interval) and 0 < interval <= 10. else 1. / rate
                data = ROW.pack(self.elapsed, device_time, *values, temperature, interval, rate, status, revision)
                if self.file.write(data) != len(data):
                    raise OSError('静态分析缓存写入不完整')
                self.last_time = self.elapsed
                self.count += 1
            except (OSError, ValueError, OverflowError, struct.error) as error:
                self.error = str(error)

    def append(self, sample, interval, cfg, revision=0):
        values = [v * cfg['gyro_lsb'] for v in sample['gyr_raw']]
        values += [v * cfg['accel_lsb'] for v in sample['acc_raw']]
        with self.lock:
            if revision not in self.model_configs:
                self.model_configs[revision] = {key: copy.deepcopy(cfg[key]) for key in
                        ('gyro_lsb', 'accel_lsb', 'freq', 'axis_map', 'axis_sign', 'invert_y')}
            self.append_values(values, sample['temp'], interval, cfg['freq'], sample['status'],
                               sample['week'] * 604800. + sample['sec'], revision)

    def info(self):
        with self.lock:
            return {'count': self.count, 'duration': self.last_time, 'error': self.error, 'closed': self.closed}

    def read(self, start=None, end=None, cancel=None, metadata=None):
        import numpy as np
        check_cancel(cancel)
        with self.lock:
            if self.closed:
                raise ValueError('数据源已关闭，请重新载入')
            if self.error:
                raise ValueError('分析缓存不完整：' + self.error)
            count = self.count  # A fixed snapshot even while acquisition continues.
            configurations = copy.deepcopy(self.model_configs)
            if not count:
                raise ValueError('当前没有可分析的 IMU 数据')
            def bound(value, right=False):
                lo, hi = 0, count
                while lo < hi:
                    mid = (lo + hi) // 2
                    self.file.seek(mid * ROW.size)
                    stamp = struct.unpack('<d', self.file.read(8))[0]
                    if stamp < value or (right and stamp == value):
                        lo = mid + 1
                    else:
                        hi = mid
                return lo
            first = bound(start) if start is not None else 0
            last = bound(end, right=True) if end is not None else count
        if last <= first:
            raise ValueError('所选时间段没有样本')
        result = np.empty((last - first, 13), dtype=np.float64)
        for offset in range(first, last, 32768):
            check_cancel(cancel)
            length = min(32768, last - offset)
            with self.lock:
                if self.closed:
                    raise ValueError('数据源已关闭')
                self.file.seek(offset * ROW.size)
                block = self.file.read(length * ROW.size)
            if len(block) != length * ROW.size:
                raise ValueError('分析缓存读取不完整')
            result[offset - first:offset - first + length] = np.frombuffer(block, dtype='<f8').reshape(-1, 13)
        metadata = dict(metadata or {})
        metadata.setdefault('gps_time_known', True)
        metadata['time_basis'] = 'cumulative device intervals; invalid or >10s clock jumps advance by one nominal period and fail quality checks'
        metadata['model_configurations'] = {str(k): v for k, v in configurations.items()
                                            if k in result[:, 12]}
        return AnalysisData(result, self.source, metadata)

    def close(self):
        with self.lock:
            self.closed = True
            if self.file is not None:
                try:
                    self.file.close()
                except OSError as error:
                    self.error = str(error)
                self.file = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def load_recording(path, cfg, cancel=None, formats=None):
    """Import without starting/replacing a serial source or running attitude calibration."""
    path = Path(path)
    archive = SampleArchive(str(path))
    metadata = {'model_scales': {k: cfg[k] for k in ('gyro_lsb', 'accel_lsb', 'freq')}, 'crc_errors': 0}
    try:
        if path.suffix.lower() == '.csv':
            metadata['gps_time_known'] = False  # Legacy t_s is elapsed, never GPS time.
            with path.open(newline='', encoding='utf-8-sig') as source:
                reader = csv.DictReader(source)
                fields = set(reader.fieldnames or [])
                raw = ('gx_raw', 'gy_raw', 'gz_raw', 'ax_raw', 'ay_raw', 'az_raw')
                physical = ('wx', 'wy', 'wz', 'ax', 'ay', 'az')
                has_raw = set(raw) <= fields
                if 't_s' not in fields or not (has_raw or set(physical) <= fields):
                    raise ValueError('CSV需包含t_s及六轴原始计数列或wx/wy/wz/ax/ay/az物理量列')
                metadata['physical_only'] = not has_raw
                previous = None
                for number, row in enumerate(reader, 2):
                    if number % 1024 == 0:
                        check_cancel(cancel)
                    try:
                        stamp = float(row['t_s'])
                        values = [float(row[k]) for k in (raw if has_raw else physical)]
                        if has_raw:
                            values = [v * (cfg['gyro_lsb'] if k < 3 else cfg['accel_lsb']) for k, v in enumerate(values)]
                        interval = stamp - previous if previous is not None else float('nan')
                        status = float(row['status']) if row.get('status') else float(row.get('counter') or 'nan')
                        archive.append_values(values, float(row.get('temp') or 'nan'), interval,
                                              cfg['freq'], status, stamp)
                        previous = stamp
                    except (ValueError, TypeError) as error:
                        raise ValueError('CSV第%d行格式错误：%s' % (number, error))
        else:
            from rawimu_live import FrameParser, FormatRegistry, to_physical
            previous = [None]
            def on_frame(spec, body):
                sample = to_physical(spec['decode'](body), spec, cfg)
                stamp = sample['week'] * 604800. + sample['sec']
                interval = stamp - previous[0] if previous[0] is not None else float('nan')
                archive.append(sample, interval, cfg)
                previous[0] = stamp
            parser = FrameParser(on_frame, formats or FormatRegistry())
            with path.open('rb') as source:
                while True:
                    check_cancel(cancel)
                    chunk = source.read(65536)
                    if not chunk:
                        parser.feed(b'', final=True)
                        break
                    parser.feed(chunk)
            metadata['crc_errors'] = parser.n_crc + parser.n_text_crc_bad
        check_cancel(cancel)
        return archive.read(cancel=cancel, metadata=metadata)
    finally:
        archive.close()
