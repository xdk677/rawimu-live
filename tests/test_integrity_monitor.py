import struct
import math
import unittest

from test_rawimu_live import live, session, reference_crc
from test_status_monitor import frames_with_status


class IntegrityTests(unittest.TestCase):
    def test_interval_histories_preserve_raw_gaps_and_counter_resets(self):
        data = frames_with_status([254, 255, 0, 3, 2])
        for k, seconds in enumerate((100., 100.01, 100.02, 100.62, 100.61)):
            struct.pack_into('<d', data, k * 72 + 32, seconds)
            struct.pack_into('<I', data, k * 72 + 68, reference_crc(data[k * 72:k * 72 + 68]))
        s = session()
        s.parser.feed(data)
        history = s.status_snapshot()
        self.assertTrue(math.isnan(history['time_interval_ms'][0]))
        self.assertTrue(math.isnan(history['counter_interval'][0]))
        for actual, expected in zip(history['time_interval_ms'][1:], [10, 10, 600, -10]):
            self.assertAlmostEqual(actual, expected, places=3)
        self.assertEqual(history['counter_interval'][1:], [1, 1, 3, -1])
        s.set_model({'FAST': {'freq': 1000}}, 'FAST')
        self.assertGreaterEqual(s.time_intervals.maxlen, 60000)
        self.assertGreaterEqual(s.counter_intervals.maxlen, 60000)

    def test_counter_wrap_is_normal_but_duplicates_and_jumps_are_counted(self):
        s = session()
        s.parser.feed(frames_with_status([254, 255, 0, 0, 3]))
        metrics = s.integrity_snapshot()
        self.assertEqual(metrics['counter_jumps'], 1)
        self.assertEqual(metrics['counter_duplicates'], 1)
        self.assertEqual(metrics['crc_errors'], 0)

    def test_bad_interval_is_measured_before_attitude_fallback(self):
        data = frames_with_status([0, 1, 2, 3])
        for k, seconds in enumerate((100., 100.01, 100.61, 100.60)):
            struct.pack_into('<d', data, k * 72 + 32, seconds)
            struct.pack_into('<I', data, k * 72 + 68, reference_crc(data[k * 72:k * 72 + 68]))
        s = session()
        s.parser.feed(data)
        metrics = s.integrity_snapshot()
        self.assertEqual(metrics['interval_anomalies'], 2)
        self.assertAlmostEqual(metrics['last_interval_ms'], -10, places=3)
        self.assertAlmostEqual(s.dt_sum, .04, places=5)

    def test_crc_only_input_is_visible_without_valid_imu_samples(self):
        s = session()
        frame = frames_with_status([0])
        frame[-1] ^= 1
        s.parser.feed(frame)
        self.assertEqual(s.n_imu, 0)
        self.assertEqual(s.integrity_snapshot()['crc_errors'], 1)

