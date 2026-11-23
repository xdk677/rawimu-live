import math
import unittest

from test_rawimu_live import ROOT, session


class GPSTimeAxisTests(unittest.TestCase):
    def test_session_histories_keep_measured_sow_when_replay_time_is_normalized(self):
        s = session()
        try:
            s.replay_elapsed = 123.
            s.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
            history = s.status_snapshot()
            self.assertEqual(history['time'], [123., 123.])
            self.assertEqual(history['sow'][-1], s.smp['sec'])
            self.assertAlmostEqual(history['sow'][1]-history['sow'][0], .01, places=5)
            s.set_model({'FAST': {'freq': 1000}}, 'FAST')
            self.assertEqual(s.seconds.maxlen, s.t.maxlen)
            self.assertEqual(len(s.seconds), 2)
        finally:
            s.close_analysis()

    def test_sow_mapping_uses_measured_times_and_rolls_at_week_boundary(self):
        from time_axis import TimeAxis
        axis = TimeAxis('sow', [0., 1., 2.], [604799., 0., 1.], [2445, 2446, 2446])
        self.assertEqual(axis.tick_text(1., .1), '0.0 s')
        self.assertEqual(axis.tick_text(.5, .1), '604799.5 s')
        self.assertIn('W2446', axis.cursor_text(1.))
        self.assertIn('0.0000', axis.cursor_text(1.))
        self.assertAlmostEqual(axis.seconds_at(-1.), 604798.)

    def test_clock_jump_is_not_interpolated_into_fake_gps_time(self):
        from time_axis import TimeAxis
        axis = TimeAxis('sow', [0., 1., 2.], [100., 101., 200.], [2446]*3)
        self.assertTrue(math.isnan(axis.seconds_at(1.5)))
        self.assertEqual(axis.seconds_at(2.), 200.)
        self.assertEqual(axis.tick_text(1.5, .1), '—')

    def test_unavailable_gps_is_explicit_and_elapsed_mode_does_not_change(self):
        from time_axis import TimeAxis
        axis = TimeAxis('sow', [0., 1.], [100., 101.], [0, 0])
        self.assertEqual(axis.tick_text(.5, .1), '—')
        self.assertIn('不可用', axis.cursor_text(.5))
        self.assertEqual(TimeAxis('elapsed').tick_text(.5, .1), '0.5 s')

    def test_performance_result_retains_device_time_and_does_not_treat_csv_elapsed_as_gps(self):
        from imu_performance import analyze_static
        from performance_data import load_recording
        from test_rawimu_live import live
        data = load_recording(ROOT/'examples/rawimu_test.dat', dict(live.DEFAULT_CFG))
        result = analyze_static(data)
        self.assertEqual(result['device_time'][0], data.records[0, 1])
        self.assertTrue(result['metadata']['gps_time_known'])

    def test_legacy_csv_time_is_not_misreported_as_gps(self):
        import tempfile
        from pathlib import Path
        from performance_data import load_recording
        from time_axis import TimeAxis
        from test_rawimu_live import live
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'elapsed.csv'
            # An elapsed timestamp may be large; magnitude cannot establish GPS origin.
            path.write_text('t_s,wx,wy,wz,ax,ay,az\n1480000000,0,0,0,0,0,9.8\n1480000001,0,0,0,0,0,9.8\n')
            data = load_recording(path, dict(live.DEFAULT_CFG))
            self.assertFalse(data.metadata['gps_time_known'])
            self.assertEqual(TimeAxis('sow', data.records[:, 0]).tick_text(0., 1.), '—')
