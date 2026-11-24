import csv
import math
from pathlib import Path
import tempfile
import threading
import unittest

import numpy as np
from test_rawimu_live import ROOT, live, session


def static_data(count=4000, dt=.01):
    from performance_data import AnalysisData
    rows = np.zeros((count, 13), dtype=np.float64)
    rows[:, 0] = np.arange(count) * dt
    rows[:, 1] = 1400000000. + rows[:, 0]
    rng = np.random.RandomState(42)
    rows[:, 2:8] = rng.normal(0, .001, (count, 6))
    rows[:, 7] += 9.80665
    rows[:, 8] = 25.
    rows[:, 9] = dt
    rows[0, 9] = np.nan
    rows[:, 10] = 1 / dt
    rows[:, 11] = np.arange(count) % 256
    return AnalysisData(rows, 'synthetic stationary')


class AllanTests(unittest.TestCase):
    def test_default_logarithmic_factors_do_not_round_above_supported_tail(self):
        from imu_performance import overlapping_allan
        result = overlapping_allan(np.ones((12000, 1)), .01)
        self.assertLessEqual(int(result['factors'][-1]), 1199)
        self.assertGreaterEqual(int(result['clusters'].min()), 10)

    def test_balanced_timestamp_jitter_warns_but_continuous_sampling_can_compute_allan(self):
        from imu_performance import analyze_static, overlapping_allan
        data = static_data(6000)
        phase = np.zeros(len(data.records))
        phase[100::200] = .0005
        measured = np.arange(len(phase)) * .01 + phase
        data.records[:, 0] = measured
        data.records[1:, 9] = np.diff(measured)
        original = data.records.copy()
        result = analyze_static(data, confirmed_static=True)
        self.assertIsNotNone(result['allan'])
        self.assertGreater(result['quality']['timestamp_jitter_count'], 0)
        self.assertEqual(result['quality']['interval_anomalies'], 0)
        self.assertTrue(result['quality']['uniform_sampling_approximation'])
        self.assertTrue(np.isfinite(result['smooth'][1001:]).all())
        reference = overlapping_allan(data.records[:, 2:8], .01)
        np.testing.assert_allclose(result['allan']['variance'], reference['variance'], rtol=1e-12)
        np.testing.assert_allclose(result['allan']['tau'], reference['tau'], rtol=1e-12)
        np.testing.assert_equal(data.records, original)

    def test_strict_timing_mode_still_rejects_the_same_soft_jitter(self):
        from imu_performance import analyze_static
        data = static_data()
        data.records[100, 9], data.records[101, 9] = .0095, .0105
        result = analyze_static(data, confirmed_static=True, timing_mode='strict')
        self.assertIsNone(result['allan'])
        self.assertEqual(result['quality']['strict_interval_anomalies'], 2)
        self.assertTrue(any('严格' in text for text in result['quality']['issues']))

    def test_small_per_interval_drift_is_not_mistaken_for_bounded_timestamp_jitter(self):
        from imu_performance import analyze_static
        data = static_data()
        data.records[1:2000, 9] = .0099
        data.records[2000:, 9] = .0101
        result = analyze_static(data, confirmed_static=True)
        self.assertIsNone(result['allan'])
        self.assertGreater(result['quality']['phase_error_max_ms'], 100.)
        self.assertTrue(any('累计' in text for text in result['quality']['issues']))

    def test_overlapping_variance_matches_direct_adjacent_means_reference(self):
        from imu_performance import overlapping_allan
        values = np.random.RandomState(3).normal(size=(300, 6))
        result = overlapping_allan(values, .01, factors=[1, 2, 7, 20])
        for i, m in enumerate((1, 2, 7, 20)):
            diffs = [values[k + m:k + 2*m].mean(axis=0) - values[k:k + m].mean(axis=0)
                     for k in range(len(values) - 2*m + 1)]
            expected = np.mean(np.square(diffs), axis=0) / 2
            np.testing.assert_allclose(result['variance'][i], expected, rtol=1e-11)
            self.assertEqual(result['pairs'][i], len(diffs))
        np.testing.assert_allclose(result['deviation'] ** 2, result['variance'])

    def test_white_noise_and_constant_offset(self):
        from imu_performance import overlapping_allan
        values = np.random.RandomState(5).normal(0, .02, (100000, 1))
        result = overlapping_allan(values, .01, factors=[1, 10, 100])
        np.testing.assert_allclose(result['variance'][:, 0], .02**2 / np.array([1, 10, 100]), rtol=.12)
        shifted = overlapping_allan(values + 5000, .01, factors=[1, 10, 100])
        np.testing.assert_allclose(shifted['variance'], result['variance'], rtol=1e-8)

    def test_analysis_uses_unsmoothed_samples_and_complete_ten_second_windows(self):
        from imu_performance import analyze_static
        result = analyze_static(static_data(), confirmed_static=True)
        self.assertTrue(result['quality']['allan_allowed'])
        self.assertTrue(np.isnan(result['smooth'][:999]).all())
        self.assertTrue(np.isfinite(result['smooth'][1000:]).all())
        self.assertLess(result['stats']['smooth_std'][0], result['stats']['std'][0] / 5)
        self.assertAlmostEqual(result['allan']['variance'][0, 0], result['stats']['std'][0]**2, delta=1e-7)
        self.assertGreater(result['allan']['pairs'][-1], 100)

    def test_quality_rejects_gaps_counter_changes_and_unconfirmed_static(self):
        from imu_performance import analyze_static
        data = static_data()
        self.assertIsNone(analyze_static(data, confirmed_static=False)['allan'])
        for column, value in ((9, .02), (11, 200), (12, 1), (2, np.nan)):
            bad = static_data()
            bad.records[120, column] = value
            result = analyze_static(bad, confirmed_static=True)
            self.assertFalse(result['quality']['allan_allowed'], column)
            self.assertIsNone(result['allan'])
            self.assertTrue(result['quality']['issues'])

    def test_smoothing_does_not_bridge_device_time_gaps(self):
        from imu_performance import analyze_static
        data = static_data()
        data.records[1500, 9] = .5
        result = analyze_static(data, confirmed_static=True)
        self.assertTrue(np.isnan(result['smooth'][1500:2500]).all())
        self.assertTrue(np.isfinite(result['smooth'][2501:]).all())
        self.assertIsNone(result['allan'])

    def test_missing_status_does_not_hide_other_counter_or_fault_errors(self):
        from imu_performance import analyze_static
        data = static_data()
        data.records[100, 11] = np.nan
        data.records[200, 11] = data.records[199, 11]
        data.records[300, 11] += 256
        result = analyze_static(data, confirmed_static=True)
        self.assertFalse(result['quality']['allan_allowed'])
        self.assertGreater(result['quality']['counter_anomalies'], 0)
        self.assertEqual(result['quality']['fault_samples'], 1)

    def test_display_preserves_multiple_nan_gaps_inside_one_reduction_bucket(self):
        from imu_performance import display_data
        data = static_data(10000)
        values = data.records[:, 2:8].copy()
        smooth = values.copy()
        smooth[100:110, 0] = np.nan
        smooth[400:410, 0] = np.nan
        smooth[250, 0] = -1000
        smooth[500, 0] = 10000
        times, _, shown = display_data({'time': np.arange(10000), 'values': values, 'smooth': smooth}, target=100)
        self.assertTrue(np.isnan(shown[(times >= 250) & (times <= 500), 0]).any())

    def test_cancellation_and_short_data(self):
        from imu_performance import analyze_static
        cancel = threading.Event()
        cancel.set()
        with self.assertRaises(InterruptedError):
            analyze_static(static_data(), confirmed_static=True, cancel=cancel)
        result = analyze_static(static_data(100), confirmed_static=True)
        self.assertTrue(np.isnan(result['stats']['smooth_std']).all())


class ArchiveTests(unittest.TestCase):
    def test_smoothing_uses_measured_time_when_configured_rate_differs(self):
        from performance_data import SampleArchive
        from imu_performance import analyze_static
        archive = SampleArchive()
        try:
            for k in range(1101):
                archive.append_values([0., 0., 0., 0., 0., 9.80665], 25., .001, 100., k % 256, k * .001)
            result = analyze_static(archive.read(), confirmed_static=True)
            self.assertAlmostEqual(result['quality']['duration_s'], 1.1)
            self.assertTrue(np.isnan(result['smooth']).all())
        finally:
            archive.close()

    def test_archive_io_failure_does_not_stop_imu_processing(self):
        from unittest.mock import patch
        s = session()
        with patch('performance_data.tempfile.TemporaryFile', side_effect=OSError('disk unavailable')):
            s.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:216])
        self.assertEqual(s.n_imu, 3)
        self.assertIn('disk unavailable', s.analysis_archive.info()['error'])
        s.close_analysis()

    def test_full_session_archive_survives_ring_rollover_and_freezes_scale(self):
        from test_status_monitor import frames_with_status
        s = session()
        try:
            s.feed(frames_with_status([k % 256 for k in range(6800)]))
            data = s.analysis_archive.read()
            self.assertEqual(len(data.records), 6800)
            self.assertGreater(len(data.records), len(s.t))
            self.assertAlmostEqual(data.records[0, 0], 0)
            s.cfg['accel_lsb'] *= 1.1
            s.feed(frames_with_status([0]))
            np.testing.assert_allclose(s.analysis_archive.read().records[-1, 5:8], data.records[-1, 5:8])
        finally:
            s.close_analysis()

    def test_file_import_binary_ascii_and_csv_uses_raw_columns(self):
        from performance_data import load_recording
        cfg = dict(live.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False)
        binary = load_recording(ROOT / 'examples/rawimu_test.dat', cfg)
        self.assertEqual(len(binary.records), 2000)
        ascii_data = load_recording(ROOT / 'examples/rawimua_ascii.dat', cfg)
        self.assertEqual(len(ascii_data.records), 797)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'test.csv'
            path.write_text('t_s,gx_raw,gy_raw,gz_raw,ax_raw,ay_raw,az_raw,wx,wy,wz,ax,ay,az,temp,counter\n'
                            '0,1,2,3,4,5,6,999,999,999,999,999,999,25,0\n'
                            '0.01,2,3,4,5,6,7,999,999,999,999,999,999,25,1\n', encoding='utf-8')
            loaded = load_recording(path, cfg)
            self.assertAlmostEqual(loaded.records[0, 2], cfg['gyro_lsb'])
            self.assertAlmostEqual(loaded.records[0, 5], 4*cfg['accel_lsb'])

    def test_range_snapshot_is_inclusive_and_export_has_variance_and_deviation(self):
        from imu_performance import analyze_static, export_analysis
        result = analyze_static(static_data(), start=5., end=25., confirmed_static=True)
        self.assertAlmostEqual(result['time'][0], 5.)
        self.assertAlmostEqual(result['time'][-1], 25.)
        with tempfile.TemporaryDirectory() as folder:
            paths = export_analysis(result, Path(folder) / 'result')
            self.assertTrue(all(Path(path).is_file() for path in paths))
            with open(str(Path(folder) / 'result_allan.csv'), newline='', encoding='utf-8-sig') as handle:
                row = next(csv.DictReader(handle))
            self.assertIn('gx_variance_(deg/s)^2', row)
            self.assertIn('gx_deviation_deg/s', row)


if __name__ == '__main__':
    unittest.main()
