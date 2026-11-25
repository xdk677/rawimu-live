"""GPIMU32 decoding and MATLAB-compatible models, kept separate from RAWIMU."""
from array import array
from collections import deque
import copy
import math
from pathlib import Path
import time

# deg/s, m/s² and °C. Sources: load_imu_model.m, IMU_ID_M21/M22 aliases.
GP_MODELS = {
    'M22P': {'gyro': (1/(100*65536.),)*3, 'accel': (9.80665/(5000*65536.),)*3,
             'temp': .005, 'offset': 40.},
    'M1633': {'gyro': (1/(100*65536.),)*3, 'accel': (1/(200*65536.),)*3, 'temp': .01, 'offset': 0.},
    'M21': {'gyro': (1/(100*65536.),)*3, 'accel': (9.80665/(2000*65536.),)*3, 'temp': 1/256., 'offset': 25.},
    'M22': {'gyro': (1/(100*65536.), 1/(100*65536.), -1/(16896*256.)),
            'accel': (9.80665/(2000*65536.),)*3, 'temp': 1/128., 'offset': 0.},
}


class GPIMUChecksumError(ValueError):
    pass


def _signed_word(text):
    if len(text) != 8 or any(c not in '0123456789abcdefABCDEF' for c in text):
        raise ValueError('GPIMU32传感器/温度字段须为8位十六进制')
    value = int(text, 16)
    return value - 2**32 if value >= 2**31 else value


def decode_gpimu(line):
    text = line.decode('ascii') if isinstance(line, bytes) else line
    text = text.strip()
    payload, star, checksum = text.rpartition('*')
    if not star or len(checksum) != 2 or not text.startswith('$GPIMU,'):
        raise ValueError('GPIMU需要完整语句和2位XOR校验')
    try:
        expected = int(checksum, 16)
    except ValueError:
        raise ValueError('GPIMU校验字段不是十六进制')
    actual = 0
    for value in payload[1:].encode('ascii'):
        actual ^= value
    if expected != actual:
        raise GPIMUChecksumError('GPIMU XOR校验失败')
    fields = [field.strip() for field in payload.split(',')]
    if len(fields) not in (18, 19):
        raise ValueError('GPIMU32周号格式需要18或19个逗号字段')
    imu_time, count = float(fields[1]), int(fields[2])
    week, sow = int(fields[9]), float(fields[10])
    if not math.isfinite(imu_time) or not math.isfinite(sow) or count < 0 or week < 0:
        raise ValueError('GPIMU时间/采样数无效')
    gyro = [_signed_word(field) for field in fields[3:6]]
    acc = [_signed_word(field) for field in fields[6:9]]
    temperature = _signed_word(fields[17])
    if len(fields) == 19 and fields[18] and int(fields[18], 16) == 12:
        raise ValueError('16位GPIMU需专用配置，当前视图支持32位格式')
    valid_gps = week > 0 and 0 <= sow < 604800.
    try:
        # M22P transmits the sensor ID as "1E" (IMU-30).
        type_value = int(fields[16], 16 if any(c in 'abcdefABCDEF' for c in fields[16]) else 10)
    except ValueError:
        type_value = float('nan')
    return {'imu_time': imu_time, 'sample_count': count, 'gyro': gyro, 'acc': acc,
            'week': week, 'sow': sow, 'temperature_raw': temperature, 'type_value': type_value,
            'type_text': fields[16], 'extensions': fields[18:],
            'gps_stamp': week*604800. + sow if valid_gps else float('nan'),
            'clock': week*604800. + sow if valid_gps else imu_time,
            'gps_valid': valid_gps}


def convert_gpimu(raw, model):
    import numpy as np
    profile = GP_MODELS[model]
    temperature = raw['temperature_raw']
    if model == 'M22':
        temperature = (temperature & 0xffff)
        temperature = temperature - 65536 if temperature >= 32768 else temperature
    return (np.asarray(raw['gyro'] + raw['acc']) * np.asarray(profile['gyro'] + profile['accel']),
            temperature * profile['temp'] + profile['offset'])


def convert_rows(rows, model):
    import numpy as np
    profile = GP_MODELS[model]
    temperature = rows[:, 12]
    if model == 'M22':
        temperature = temperature.astype(np.int64) & 0xffff
        temperature = np.where(temperature >= 32768, temperature-65536, temperature)
    return rows[:, 6:12] * np.asarray(profile['gyro'] + profile['accel']), temperature * profile['temp'] + profile['offset']


class GPIMUStore(object):
    # Columns: elapsed,gps_stamp,IMUtime,week,sow,samples,GXYZ,AXYZ,Traw,dt_ms,type,gps_valid.
    def __init__(self, capacity=6502, keep_all=False):
        self.keep_all = keep_all
        self.rows = array('d') if keep_all else deque(maxlen=capacity)
        self.total = 0
        self.previous = None
        self.previous_domain = None
        self.elapsed = 0.
        self.last_wall = None

    def append(self, raw, frequency=100., elapsed=None):
        interval = float('nan')
        if self.previous is not None and raw['gps_valid'] == self.previous_domain:
            interval = raw['clock'] - self.previous
        if self.total:
            self.elapsed += interval if math.isfinite(interval) and 0 < interval <= 10 else 1./frequency
        if elapsed is not None:
            self.elapsed = elapsed
        row = (self.elapsed, raw['gps_stamp'], raw['imu_time'], raw['week'], raw['sow'], raw['sample_count'],
               *raw['gyro'], *raw['acc'], raw['temperature_raw'], interval*1000., raw['type_value'], int(raw['gps_valid']))
        if self.keep_all:
            self.rows.extend(row)
        else:
            self.rows.append(row)
        self.total += 1
        self.previous, self.previous_domain = raw['clock'], raw['gps_valid']
        self.last_wall = time.monotonic()

    def resize(self, capacity):
        if not self.keep_all:
            self.rows = deque(self.rows, maxlen=capacity)

    def snapshot(self):
        import numpy as np
        return np.asarray(self.rows, dtype=float).reshape(-1, 16).copy()


def match_rawimu(gp_stamps, raw_rows, tolerance=.002):
    """Nearest valid GPS epochs, one-to-one, no interpolation or ambiguous duplicates."""
    import numpy as np
    gp_stamps = np.asarray(gp_stamps)
    output = np.full((len(gp_stamps), 6), np.nan)
    delta = np.full(len(gp_stamps), np.nan)
    if not len(raw_rows) or not len(gp_stamps):
        return output, delta
    raw_rows = np.asarray(raw_rows)
    raw_rows = raw_rows[np.isfinite(raw_rows).all(axis=1)]
    if not len(raw_rows):
        return output, delta
    raw_rows = raw_rows[np.argsort(raw_rows[:, 0], kind='stable')]
    stamps = raw_rows[:, 0]
    unique, counts = np.unique(stamps, return_counts=True)
    raw_rows = raw_rows[np.isin(stamps, unique[counts == 1])]
    if not len(raw_rows):
        return output, delta
    stamps = raw_rows[:, 0]
    unique_gp, gp_counts = np.unique(gp_stamps[np.isfinite(gp_stamps)], return_counts=True)
    valid_gp = np.flatnonzero(np.isfinite(gp_stamps) & np.isin(gp_stamps, unique_gp[gp_counts == 1]))
    pos = np.searchsorted(stamps, gp_stamps[valid_gp])
    left = np.clip(pos-1, 0, len(stamps)-1)
    right = np.clip(pos, 0, len(stamps)-1)
    left_distance, right_distance = np.abs(stamps[left]-gp_stamps[valid_gp]), np.abs(stamps[right]-gp_stamps[valid_gp])
    nearest = np.where(left_distance <= right_distance, left, right)
    distance = np.minimum(left_distance, right_distance)
    valid = (distance <= tolerance) & ~((left != right) & (np.abs(left_distance-right_distance) < 1e-8))
    candidates = zip(distance[valid].tolist(), valid_gp[valid].tolist(), nearest[valid].tolist())
    used = set()
    for _, k, nearest in sorted(candidates):
        if nearest not in used:
            used.add(nearest)
            output[k] = raw_rows[nearest, 1:7]
            delta[k] = (stamps[nearest] - gp_stamps[k]) * 1000.
    return output, delta


def load_gpimu_recording(path, cfg, cancel=None, formats=None):
    import numpy as np
    from rawimu_live import FrameParser, FormatRegistry, to_physical
    from performance_data import check_cancel
    store = GPIMUStore(keep_all=True)
    raw_rows = array('d')
    def raw_frame(spec, body):
        try:
            sample = to_physical(spec['decode'](body), spec, cfg)
            stamp = sample['week']*604800. + sample['sec'] if sample['week'] > 0 else float('nan')
            raw_rows.extend((stamp, *sample['gyr'], *sample['acc']))
        except (ValueError, TypeError, KeyError):
            pass
    parser = FrameParser(raw_frame, formats or FormatRegistry(), lambda raw: store.append(raw, cfg['freq']))
    with open(path, 'rb') as source:
        while True:
            check_cancel(cancel)
            block = source.read(65536)
            if not block:
                parser.feed(b'', final=True)
                break
            parser.feed(block)
    check_cancel(cancel)
    if not store.total:
        raise ValueError('未找到校验通过的GPIMU32语句（XOR错误%d，格式错误%d）' %
                         (parser.n_gpimu_crc_bad, parser.n_gpimu_malformed))
    return {'rows': store.snapshot(), 'raw_rows': np.asarray(raw_rows).reshape(-1, 7),
            'total': store.total, 'checksum_bad': parser.n_gpimu_crc_bad, 'malformed': parser.n_gpimu_malformed,
            'source': str(Path(path)), 'raw_model': cfg.get('imu_model', ''), 'raw_cfg': copy.deepcopy(cfg)}
