import struct
import unittest

from test_rawimu_live import ROOT, live, reference_crc, session


def frames_with_status(words):
    frame = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72])
    data = bytearray()
    for k, word in enumerate(words):
        struct.pack_into('<d', frame, 32, 100. + k * .01)
        struct.pack_into('<I', frame, 40, word)
        struct.pack_into('<I', frame, 68, reference_crc(frame[:68]))
        data.extend(frame)
    return data


class StatusHistoryTests(unittest.TestCase):
    def test_fault_names_follow_matlab_status_byte_mapping(self):
        expected = ('GyroX', 'GyroY', 'GyroZ', 'AccX', 'AccY', 'AccZ', 'Temp', 'FUSA')
        for bit, name in enumerate(expected):
            self.assertEqual(live.imu_fault_names(1 << (8 + bit)), [name])
        self.assertEqual(live.imu_fault_names(0xffff00ff), [])
        self.assertEqual(live.imu_fault_names(0xffffa5ff), ['GyroX', 'GyroZ', 'AccZ', 'FUSA'])

    def test_status_history_decodes_all_fields_on_the_same_samples(self):
        s = session()
        s.parser.feed(frames_with_status([0xfff69afe, 0x000101ff, 0x00020000]))
        history = s.status_snapshot()
        self.assertEqual(history['status'], [0xfff69afe, 0x000101ff, 0x00020000])
        self.assertEqual(history['stat_byte'], [0x9a, 1, 0])
        self.assertEqual(history['counter'], [254, 255, 0])
        self.assertEqual(history['temperature'], [-.1, .01, .02])
        self.assertEqual(history['time'], s.snapshot()[0])
        history['status'][0] = 0
        self.assertEqual(s.status_snapshot()['status'][0], 0xfff69afe)

    def test_status_history_rollover_and_model_resize_remain_aligned(self):
        s = session()
        s.parser.feed(frames_with_status([0x01000200 + k for k in range(s.t.maxlen + 10)]))
        history = s.status_snapshot()
        self.assertEqual(len(history['status']), len(history['time']))
        self.assertEqual(history['status'][0], 0x0100020a)
        s.set_model({'FAST': {'freq': 1000}}, 'FAST')
        self.assertEqual(s.status_snapshot(), history)
        self.assertGreaterEqual(s.imu_status.maxlen, 60000)
        self.assertGreaterEqual(s.temperature.maxlen, 60000)

    def test_step_points_never_interpolate_between_digital_values(self):
        points = live.step_points([(0, 0), (1, 1), (2, 1), (3, 0)])
        self.assertEqual(points, [(0, 0), (1, 0), (1, 1), (2, 1), (3, 1), (3, 0)])
        self.assertTrue(all(a[0] == b[0] or a[1] == b[1] for a, b in zip(points, points[1:])))

    def test_held_state_is_clipped_to_the_left_window_boundary(self):
        points = live.step_points([(0, 1), (25, 1), (40, 0)])
        self.assertEqual(live.clip_points(points, 10, 40), [(10, 1), (25, 1), (40, 1), (40, 0)])


if __name__ == '__main__':
    unittest.main()
