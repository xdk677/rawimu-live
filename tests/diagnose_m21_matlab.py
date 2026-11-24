"""Compare an actual M21 DAT to nav_data.imu_raw produced by main_of_dpx.m.

Usage: python tests/diagnose_m21_matlab.py <dat> <nav.mat> <report.json>
Requires h5py only for the verification tool; it is not an app dependency.
"""
import json
from pathlib import Path
import sys
import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
import rawimu_live as live
from axis_analysis import centered_sample_average


def main():
    dat, mat, output = map(Path, sys.argv[1:])
    with h5py.File(str(mat), 'r') as source:
        raw = np.asarray(source['nav_data/imu_raw']).T
    cfg = dict(live.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False)
    live.apply_model(cfg, live.IMU_MODELS, 'M21')
    session = live.Session(cfg)
    try:
        session.feed(dat.read_bytes())
        history = session.status_snapshot()
        actual = np.column_stack([history[key] for key in ('gx','gy','gz','ax','ay','az')])
        # MATLAB cvt_rawimu2imudata sets delta flags then multiplies by 100 Hz.
        expected = raw[:, [4,5,6,1,2,3]].copy()
        expected[:, :3] *= 1./65536.
        expected[:, 3:] *= 9.80665/(20*65536.)
        np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-14)
        np.testing.assert_allclose(history['temperature'], raw[:,7]/256.+25., atol=1e-12)
        np.testing.assert_allclose(history['sow'], raw[:,0], atol=1e-9)
        np.testing.assert_array_equal(history['counter'], raw[:,9])
        np.testing.assert_allclose(history['header_interval_ms'][1:], np.diff(raw[:,0])*1000., atol=1e-9)
        smooth = np.column_stack([centered_sample_average(expected[:,i], 500) for i in range(6)])
        report = {'source': str(dat), 'matlab_reference': str(mat), 'samples': len(raw),
                  'max_physical_error': np.max(np.abs(actual-expected),axis=0).tolist(),
                  'mean': np.mean(actual,axis=0).tolist(), 'std_sample': np.std(actual,axis=0,ddof=1).tolist(),
                  'centered_5s_pp': np.ptp(smooth,axis=0).tolist(),
                  'temperature_min_max': [min(history['temperature']),max(history['temperature'])],
                  'gyro_lsb': cfg['gyro_lsb'], 'accel_lsb': cfg['accel_lsb'],
                  'temperature_conversion': 'signed raw16 / 256 + 25',
                  'crc_errors': session.integrity_snapshot()['crc_errors']}
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=True,indent=2))
    finally:
        session.close_analysis()


if __name__ == '__main__':
    main()
