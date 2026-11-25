import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from test_rawimu_live import ROOT, live, session


class RawRecordingTests(unittest.TestCase):
    def test_raw_dat_keeps_every_byte_including_unknown_and_bad_frames(self):
        s = session()
        valid = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:144]
        bad = bytearray(valid[:72])
        bad[-1] ^= 1
        data = b'noise\x00\xff' + valid + bytes(bad) + b'#UNKNOWN,text\r\n'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'raw.dat'
            self.assertTrue(s.start_raw_record(str(path)))
            for index in range(0, len(data), 7):
                s.feed(data[index:index + 7])
            s.stop_raw_record()
            self.assertEqual(path.read_bytes(), data)
            self.assertEqual(s.raw_bytes, len(data))
        self.assertEqual(s.n_imu, 2)
        self.assertEqual(s.parser.n_crc, 1)

    def test_raw_write_error_does_not_stop_parsing_or_csv(self):
        class BrokenFile(io.BytesIO):
            def write(self, data):
                raise OSError('disk full')
        s = session()
        s.raw_file, s.raw_recording = BrokenFile(), True
        s.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
        self.assertEqual(s.n_imu, 2)
        self.assertFalse(s.raw_recording)
        self.assertIn('disk full', s.raw_error)
        self.assertEqual(s.error, '')

    def test_serial_worker_records_and_flushes_on_disconnect(self):
        import serial
        s = session()
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:144]
        worker = live.SerialWorker(s)
        class Port:
            in_waiting = len(data)
            def read(self, size):
                worker.stop()
                return data
            def close(self):
                pass
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'serial.dat'
            s.start_raw_record(str(path))
            with patch.object(serial, 'Serial', return_value=Port()):
                worker.run()
            self.assertEqual(path.read_bytes(), data)
        self.assertFalse(s.raw_recording)
        self.assertEqual(s.n_imu, 2)

    def test_dat_and_csv_can_record_together_and_stop_independently(self):
        s = session()
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:216]
        with tempfile.TemporaryDirectory() as folder:
            raw, csv = Path(folder) / 'raw.dat', Path(folder) / 'parsed.csv'
            s.start_record(str(csv))
            s.start_raw_record(str(raw))
            s.feed(data[:144])
            s.stop_raw_record()
            s.feed(data[144:])
            s.stop_record()
            self.assertEqual(raw.read_bytes(), data[:144])
            self.assertEqual(len(csv.read_text().splitlines()), 4)

    def test_failed_open_and_close_are_reported(self):
        s = session()
        with tempfile.TemporaryDirectory() as folder:
            self.assertFalse(s.start_raw_record(str(Path(folder) / 'missing' / 'raw.dat')))
        self.assertTrue(s.raw_error)
        self.assertFalse(s.raw_recording)
        class BadClose(io.BytesIO):
            def close(self):
                super().close()
                raise OSError('flush failed')
        s.raw_file, s.raw_recording = BadClose(), True
        s.stop_raw_record()
        self.assertIn('flush failed', s.raw_error)
        self.assertIsNone(s.raw_file)


if __name__ == '__main__':
    unittest.main()
