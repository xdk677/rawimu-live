import unittest
from test_rawimu_live import live, session, ROOT


class MixedStreamRecoveryTests(unittest.TestCase):
    def test_binary_noise_ascii_markers_do_not_swallow_valid_rawimu(self):
        frames = (ROOT/'examples/rawimu_test.dat').read_bytes()[:144]
        for marker in (b'#', b'$'):
            data = marker+b'FAKE,\x01\x00\xff'+frames+b'\r\n'
            for block in (1, 7, 16, 65536):
                s = session()
                try:
                    for first in range(0, len(data), block):
                        s.feed(data[first:first+block])
                    self.assertEqual(s.n_imu, 2, (marker,block))
                    self.assertEqual(s.integrity_snapshot()['counter_jumps'], 0)
                    self.assertEqual(s.integrity_snapshot()['interval_anomalies'], 0)
                finally:
                    s.close_analysis()

    def test_printable_unterminated_ascii_prefix_before_binary_sync_recovers(self):
        frame = (ROOT/'examples/rawimu_test.dat').read_bytes()[:72]
        s = session()
        try:
            s.feed(b'#UNFINISHED,log,')
            s.feed(frame[:2])
            s.feed(frame[2:])
            self.assertEqual(s.n_imu, 1)
            self.assertEqual(s.parser.n_text, 0)
        finally:
            s.close_analysis()

    def test_valid_fragmented_ascii_and_binary_are_both_preserved(self):
        raw = (ROOT/'examples/rawimu_test.dat').read_bytes()[:72]
        ascii_line = next(line for line in (ROOT/'examples/rawimua_ascii.dat').read_bytes().splitlines()
                          if line.startswith(b'#RAWIMUA,')) + b'\r\n'
        s = session()
        try:
            data = ascii_line + raw
            for first in range(0, len(data), 7):
                s.feed(data[first:first+7])
            self.assertEqual(s.n_imu, 2)
            self.assertEqual(s.parser.n_text_crc_bad, 0)
        finally:
            s.close_analysis()
