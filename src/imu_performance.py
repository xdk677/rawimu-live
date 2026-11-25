"""Static IMU statistics and overlapping Allan variance, without calibration or resampling."""
import csv
import json
import math
import os
from pathlib import Path

import numpy as np

from performance_data import check_cancel

AXES = ('gx', 'gy', 'gz', 'ax', 'ay', 'az')
UNITS = ('deg/s',) * 3 + ('m/s^2',) * 3


def overlapping_allan(values, dt, factors=None, cancel=None):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim == 1:
        values = values[:, None]
    n, axes = values.shape
    if not math.isfinite(dt) or dt <= 0 or n < 3 or not np.isfinite(values).all():
        raise ValueError('Allan计算需要至少3个有限、等间隔样本')
    if factors is None:
        maximum = max(1, (n - 1) // 10)
        factors = np.unique(np.clip(np.ceil(np.logspace(0, np.log10(maximum), 64)).astype(int), 1, maximum))
    factors = np.asarray(factors, dtype=int)
    if np.any(factors < 1) or np.any(2 * factors > n):
        raise ValueError('Allan聚类长度超出数据支持范围')
    variance = np.empty((len(factors), axes), dtype=float)
    for axis in range(axes):
        check_cancel(cancel)
        centered = values[:, axis] - np.mean(values[:, axis])
        integrated = np.concatenate(([0.], np.cumsum(centered)))
        for k, m in enumerate(factors):
            check_cancel(cancel)
            differences = (integrated[2*m:] - 2 * integrated[m:-m] + integrated[:-2*m]) / m
            variance[k, axis] = np.dot(differences, differences) / (2 * len(differences))
    return {'tau': factors * dt, 'variance': variance, 'deviation': np.sqrt(variance),
            'pairs': n - 2 * factors + 1, 'clusters': n // factors, 'factors': factors}


def full_window_mean(times, values, seconds=10., cancel=None):
    result = np.full_like(values, np.nan, dtype=float)
    if len(times) < 2:
        return result
    left = np.searchsorted(times, times - seconds, side='left')
    ready = times - times[0] >= seconds
    ends = np.arange(1, len(times) + 1)
    for axis in range(values.shape[1]):
        check_cancel(cancel)
        column = values[:, axis]
        finite = np.isfinite(column)
        origin = np.mean(column[finite]) if finite.any() else 0.
        sums = np.concatenate(([0.], np.cumsum(np.where(finite, column - origin, 0.))))
        counts = np.concatenate(([0], np.cumsum(finite)))
        sizes = ends - left
        complete = ready & (counts[ends] - counts[left] == sizes)
        result[complete, axis] = (sums[ends[complete]] - sums[left[complete]]) / sizes[complete] + origin
    return result


def _stats(values):
    means, deviations, pp = [], [], []
    for column in values.T:
        finite = column[np.isfinite(column)]
        means.append(float(np.mean(finite)) if len(finite) else np.nan)
        deviations.append(float(np.std(finite, ddof=1)) if len(finite) > 1 else np.nan)
        pp.append(float(np.ptp(finite)) if len(finite) else np.nan)
    return np.array(means), np.array(deviations), np.array(pp)


def inspect_timing(intervals, mode):
    """Separate bounded timestamp jitter from breaks; thresholds are acceptance policy."""
    if mode not in ('continuous', 'strict'):
        raise ValueError('未知Allan时间模式')
    positive = intervals[np.isfinite(intervals) & (intervals > 0)]
    median = float(np.median(positive)) if len(positive) else np.nan
    invalid = ~np.isfinite(intervals) | (intervals <= 0)
    strict_mask = invalid | (np.abs(intervals-median) > max(2e-6, median*.02) if np.isfinite(median) else True)
    severe_mask = invalid | (np.abs(intervals-median) > max(2e-6, median*.10) if np.isfinite(median) else True)
    period = float(np.mean(intervals)) if not invalid.any() else median
    phase = float(np.max(np.abs(np.cumsum(intervals-period)))) if not invalid.any() else None
    phase_limit = max(2e-6, period*.25) if np.isfinite(period) else 0.
    finite = intervals[np.isfinite(intervals)]
    strict_count, severe_count = int(strict_mask.sum()), int(severe_mask.sum())
    jitter_count = int(np.count_nonzero(strict_mask & ~severe_mask))
    used_period = period if mode == 'continuous' else median
    return {'strict_mask': strict_mask, 'severe_mask': severe_mask,
            'strict_count': strict_count, 'severe_count': severe_count, 'jitter_count': jitter_count,
            'period': used_period, 'phase_bad': phase is not None and phase > phase_limit,
            'phase_error_max_ms': phase*1000 if phase is not None else None,
            'phase_limit_ms': phase_limit*1000,
            'interval_min_ms': float(finite.min()*1000) if len(finite) else None,
            'interval_max_ms': float(finite.max()*1000) if len(finite) else None}


def analyze_static(data, start=None, end=None, confirmed_static=False, smooth_seconds=10., cancel=None,
                   timing_mode='continuous'):
    check_cancel(cancel)
    if not math.isfinite(smooth_seconds) or smooth_seconds <= 0:
        raise ValueError('平滑时间必须为正数')
    records = data.records
    if records.ndim != 2 or records.shape[1] != 13:
        raise ValueError('IMU数据列数不正确')
    if (start is not None and not math.isfinite(start)) or (end is not None and not math.isfinite(end)):
        raise ValueError('起止时间必须为有限数值')
    if start is not None and end is not None and start >= end:
        raise ValueError('结束时间必须大于开始时间')
    first = np.searchsorted(records[:, 0], start, side='left') if start is not None else 0
    last = np.searchsorted(records[:, 0], end, side='right') if end is not None else len(records)
    records = records[first:last]
    if len(records) < 3:
        raise ValueError('所选时间段至少需要3个样本')
    times, values = records[:, 0], records[:, 2:8]
    intervals = records[1:, 9]
    timing = inspect_timing(intervals, timing_mode)
    dt = timing['period']
    interval_mask = timing['severe_mask']
    interval_bad = timing['severe_count']
    status = records[:, 11]
    known = np.isfinite(status) & (status >= 0) & (status <= 0xffffffff) & (status == np.floor(status))
    counter_bad = 0
    counter_mask = np.zeros(len(records) - 1, dtype=bool)
    fault_samples = 0
    words = np.where(known, status, 0).astype(np.uint32)
    pair_known = known[1:] & known[:-1]
    counter_mask = pair_known & ((np.diff((words & 255).astype(np.int64)) % 256) != 1)
    counter_bad = int(np.count_nonzero(counter_mask))
    fault_samples = int(np.count_nonzero((words[known] & 0xff00)))
    revisions = len(np.unique(records[:, 12]))
    finite = bool(np.isfinite(values).all())
    issues = []
    if timing_mode == 'strict' and timing['strict_count']:
        issues.append('严格时间模式：%d个间隔超出2%%容差（最小容差2µs）' % timing['strict_count'])
    elif interval_bad:
        issues.append('采样时间不连续%d处（回退/重复/缺口/偏差超过10%%）' % interval_bad)
    if timing['phase_bad']:
        issues.append('累计时间偏离等间隔网格%.3fms，超过%.3fms，不能视为小抖动' %
                      (timing['phase_error_max_ms'], timing['phase_limit_ms']))
    if counter_bad:
        issues.append('IMU计数不连续%d处' % counter_bad)
    if not known.all():
        issues.append('部分状态字/计数缺失或无效，无法完整校验连续性')
    if revisions > 1 or len(np.unique(records[:, 10])) > 1:
        issues.append('所选段跨越IMU配置变更')
    if not finite or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        issues.append('含缺失值或无效分析时间')
    if not confirmed_static:
        issues.append('尚未确认所选段静止')
    if len(records) < 32:
        issues.append('Allan计算至少需要32个样本')
    quality = {'allan_allowed': not issues, 'issues': issues, 'sample_count': len(records),
               'duration_s': float(times[-1] - times[0]), 'sample_rate_hz': 1/dt if dt > 0 else None,
               'interval_anomalies': interval_bad, 'counter_anomalies': counter_bad,
               'timing_mode': timing_mode, 'timestamp_jitter_count': timing['jitter_count'],
               'strict_interval_anomalies': timing['strict_count'],
               'sample_period_s': dt if np.isfinite(dt) else None,
               'phase_error_max_ms': timing['phase_error_max_ms'], 'phase_limit_ms': timing['phase_limit_ms'],
               'interval_min_ms': timing['interval_min_ms'], 'interval_max_ms': timing['interval_max_ms'],
               'uniform_sampling_approximation': timing_mode == 'continuous' and timing['jitter_count'] > 0,
               'allan_time_basis': '连续样本 / 平均周期（等间隔近似）' if timing_mode == 'continuous' else '严格设备时间 / 中位周期',
               'counter_known': bool(known.all()), 'fault_samples': fault_samples,
               'crc_errors_source_total': int(data.metadata.get('crc_errors', 0)),
               'confirmed_static': bool(confirmed_static)}
    mean, std, pp = _stats(values)
    boundary_mask = interval_mask | counter_mask | (np.diff(records[:, 12]) != 0) | (np.diff(records[:, 10]) != 0)
    if known.any():
        boundary_mask |= ~pair_known
    boundaries = np.concatenate(([0], np.flatnonzero(boundary_mask) + 1, [len(records)]))
    smooth = np.full_like(values, np.nan)
    for first_segment, last_segment in zip(boundaries[:-1], boundaries[1:]):
        check_cancel(cancel)
        smooth[first_segment:last_segment] = full_window_mean(times[first_segment:last_segment],
                    values[first_segment:last_segment], smooth_seconds, cancel)
    _, smooth_std, smooth_pp = _stats(smooth)
    motion = []
    if timing_mode == 'continuous' and timing['jitter_count']:
        motion.append('时间戳小抖动%d处，仅作质量提示；Allan按原始连续样本和实测平均周期作等间隔近似，未插值或修改测值。' % timing['jitter_count'])
    if np.any(std[:3] > .05) or np.linalg.norm(mean[:3]) > 3.:
        motion.append('角速度变化较大，建议复核所选段是否静止')
    if np.any(std[3:] > .05) or abs(np.linalg.norm(mean[3:]) - 9.80665) > .5:
        motion.append('加速度波动或重力模长异常，建议复核静态条件')
    if fault_samples:
        motion.append('选段内有IMU故障位，性能结果仅作诊断')
    if not known.all():
        motion.append('源数据无完整状态字/采样计数，无法检查计数连续性')
    if data.metadata.get('physical_only'):
        motion.append('CSV仅含物理列，可能已做零偏或标度补偿')
    if times[-1] - times[0] < smooth_seconds:
        motion.append('所选时长不足%gs，平滑统计暂不可用' % smooth_seconds)
    allan = overlapping_allan(values, dt, cancel=cancel) if quality['allan_allowed'] else None
    minimum = np.min(allan['deviation'], axis=0) if allan is not None else np.full(6, np.nan)
    minimum_tau = allan['tau'][np.argmin(allan['deviation'], axis=0)] if allan is not None else np.full(6, np.nan)
    return {'time': times, 'device_time': records[:, 1], 'values': values, 'smooth': smooth, 'smooth_seconds': smooth_seconds,
            'source': data.source, 'metadata': data.metadata, 'quality': quality, 'motion_notes': motion,
            'stats': {'mean': mean, 'std': std, 'pp': pp, 'smooth_std': smooth_std, 'smooth_pp': smooth_pp,
                      'allan_min': minimum, 'allan_min_tau': minimum_tau}, 'allan': allan}


def display_data(result, target=8000):
    """Reduce only display data, keeping extrema from every axis and missing-data breaks."""
    times, values, smooth = result['time'], result['values'], result['smooth']
    if len(times) <= target:
        return times, values, smooth
    indices = {0, len(times) - 1}
    for data in (values, smooth):
        for column in data.T:
            # Every missing/finite transition survives, including multiple runs in a bucket.
            changes = np.flatnonzero(np.isfinite(column[1:]) != np.isfinite(column[:-1])) + 1
            indices.update(changes.tolist())
            indices.update((changes - 1).tolist())
    width = max(1, int(math.ceil(len(times) * 26. / target)))
    for start in range(0, len(times), width):
        stop = min(len(times), start + width)
        for data in (values, smooth):
            for column in data[start:stop].T:
                good = np.flatnonzero(np.isfinite(column))
                if len(good):
                    indices.add(start + int(good[np.argmin(column[good])]))
                    indices.add(start + int(good[np.argmax(column[good])]))
                missing = np.flatnonzero(~np.isfinite(column))
                if len(missing):
                    indices.add(start + int(missing[0]))
    selection = np.array(sorted(indices))
    return times[selection], values[selection], smooth[selection]


def export_analysis(result, prefix, cancel=None):
    prefix = str(prefix)
    paths = [prefix + suffix for suffix in ('_summary.csv', '_allan.csv', '_timeseries.csv', '_report.json')]
    temporary = [path + '.tmp' for path in paths]
    try:
        with open(temporary[0], 'w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            keys = ('mean', 'std', 'pp', 'smooth_std', 'smooth_pp', 'allan_min', 'allan_min_tau')
            writer.writerow(['axis', 'unit'] + list(keys))
            for k, axis in enumerate(AXES):
                writer.writerow([axis, UNITS[k]] + [result['stats'][key][k] for key in keys])
        with open(temporary[1], 'w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            header = ['tau_s', 'overlapping_pairs', 'clusters']
            for axis, unit in zip(AXES, UNITS):
                header += [axis + '_variance_(' + unit + ')^2', axis + '_deviation_' + unit]
            writer.writerow(header)
            allan = result['allan']
            if allan is not None:
                for k, tau in enumerate(allan['tau']):
                    check_cancel(cancel)
                    row = [tau, allan['pairs'][k], allan['clusters'][k]]
                    for j in range(6):
                        row += [allan['variance'][k, j], allan['deviation'][k, j]]
                    writer.writerow(row)
        with open(temporary[2], 'w', newline='', encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            writer.writerow(['t_s'] + [axis + '_' + unit for axis, unit in zip(AXES, UNITS)] +
                            [axis + '_smooth_' + unit for axis, unit in zip(AXES, UNITS)])
            for k, stamp in enumerate(result['time']):
                if k % 4096 == 0:
                    check_cancel(cancel)
                writer.writerow([stamp] + list(result['values'][k]) + list(result['smooth'][k]))
        with open(temporary[3], 'w', encoding='utf-8') as handle:
            json.dump({key: result[key] for key in ('source', 'metadata', 'quality', 'motion_notes', 'smooth_seconds')},
                      handle, ensure_ascii=False, indent=2, allow_nan=False)
        check_cancel(cancel)
        for temp, path in zip(temporary, paths):
            os.replace(temp, path)
    finally:
        for path in temporary:
            if os.path.exists(path):
                os.remove(path)
    return paths
