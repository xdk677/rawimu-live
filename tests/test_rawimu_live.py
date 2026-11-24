import csv
import importlib.util
import io
import os
from pathlib import Path
import random
import struct
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rawimu_live', str(ROOT / 'src/rawimu_live.py'))
live = importlib.util.module_from_spec(spec)
spec.loader.exec_module(live)


def session():
    return live.Session(dict(live.DEFAULT_CFG, auto_bias=False, auto_accel_scale=False))


def reference_crc(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xedb88320 if crc & 1 else 0)
    return crc


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def read(self):
        return self.now

    def wait(self, seconds):
        self.now += seconds
        return False


class BackendTests(unittest.TestCase):
    def test_cli_speed_zero_is_preserved(self):
        cfgs = []
        with patch.object(live, 'App', side_effect=lambda root, cfg, **kw: cfgs.append(cfg)), \
                patch('tkinter.Tk'):
            self.assertEqual(live.main(['--speed', '0', '--no-autostart']), 0)
        self.assertEqual(cfgs[0]['replay_speed'], 0)

    def test_ascii_crc_error_does_not_enter_attitude_filter(self):
        line = next(line for line in (ROOT / 'examples/rawimua_ascii.dat').read_bytes().splitlines()
                    if line.startswith(b'#RAWIMUA,'))
        s = session()
        s.parser.feed(line[:-8] + b'00000000\r\n')
        self.assertEqual(s.parser.n_text_crc_bad, 1)
        self.assertEqual(s.n_imu, 0)

    def test_record_close_waits_for_inflight_write(self):
        entered, release, closed = threading.Event(), threading.Event(), threading.Event()
        class SlowFile(io.StringIO):
            def write(self, text):
                entered.set()
                release.wait(2)
                return super().write(text)

            def close(self):
                closed.set()
                super().close()

        s = session()
        s.csv_file, s.recording = SlowFile(), True
        writer = threading.Thread(target=s.parser.feed,
                                  args=((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72],))
        closer = threading.Thread(target=s.stop_record)
        writer.start()
        try:
            self.assertTrue(entered.wait(1))
            closer.start()
            self.assertFalse(closed.wait(.05))
        finally:
            release.set()
            writer.join(2)
            if closer.ident:
                closer.join(2)
        self.assertTrue(closed.is_set())
        self.assertEqual(s.record_error, '')

    def test_crc_matches_bitwise_reference(self):
        rng = random.Random(19)
        for size in (0, 1, 7, 68, 255, 8192):
            data = bytes(rng.randrange(256) for _ in range(size))
            self.assertEqual(live.crc32_raw(data), reference_crc(data))

    def test_switching_to_fast_model_keeps_sixty_seconds_of_history(self):
        s = session()
        s.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
        before = s.snapshot()[:4]
        self.assertTrue(s.set_model({'FAST': {'freq': 1000}}, 'FAST'))
        for values in [s.t] + s.ch + [s.rollA, s.pitchA]:
            self.assertGreaterEqual(values.maxlen, 60000)
        self.assertEqual(s.snapshot()[:4], before)

    def test_fragmented_samples_keep_all_frames(self):
        for name, expected in (('rawimu_test.dat', 2000), ('rawimua_ascii.dat', 797)):
            data = (ROOT / 'examples' / name).read_bytes()
            for step in (7, 8192):
                s = session()
                for pos in range(0, len(data), step):
                    s.parser.feed(data[pos:pos + step])
                self.assertEqual(s.n_imu, expected)
                self.assertEqual(s.parser.n_crc + s.parser.n_text_crc_bad, 0)

    def test_replay_one_x_obeys_data_duration(self):
        s = session()
        worker = live.ReplayWorker(s, str(ROOT / 'examples/rawimu_test.dat'), 1.0)
        clock = FakeClock()
        with patch.object(live.time, 'monotonic', clock.read), \
                patch.object(live.time, 'time', clock.read), \
                patch.object(live.time, 'sleep', clock.wait), \
                patch.object(threading.Event, 'wait', lambda event, timeout=None: clock.wait(timeout)):
            worker.run()
        self.assertEqual(s.n_imu, 2000)
        self.assertAlmostEqual(clock.now, 19.99, places=3)

    def test_stop_before_replay_opens_never_reads_frames(self):
        s = session()
        worker = live.ReplayWorker(s, str(ROOT / 'examples/rawimu_test.dat'), 0)
        worker.stop()
        worker.run()
        self.assertEqual(s.n_imu, 0)
        self.assertFalse(s.running)

    def test_stop_during_serial_open_closes_without_read(self):
        import serial
        s = session()
        worker = live.SerialWorker(s)
        closed = []

        class Port:
            @property
            def in_waiting(self):
                raise AssertionError('read after stop')

            def close(self):
                closed.append(True)

        def open_port(*args, **kwargs):
            worker.stop()
            return Port()

        with patch.object(serial, 'Serial', open_port):
            worker.run()
        self.assertEqual(closed, [True])
        self.assertFalse(s.running)
        self.assertEqual(s.error, '')


class StreamAndPlotTests(unittest.TestCase):
    def test_truth_timestamps_must_match_frame_cadence(self):
        original = (ROOT / 'examples/rawimu_truth.csv').read_text().splitlines()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'truth.csv'
            for mode in ('repeated', 'offset', 'nonfinite_initial'):
                lines = [original[0]]
                for k, row in enumerate(original[1:]):
                    parts = row.split(',')
                    if mode == 'repeated':
                        parts[0] = '0'
                    elif mode == 'offset' and k > 0:
                        parts[0] = str(float(parts[0]) + 1.)
                    elif mode == 'nonfinite_initial' and k == 0:
                        parts[0] = 'nan'
                    lines.append(','.join(parts))
                path.write_text('\n'.join(lines))
                with redirect_stdout(io.StringIO()):
                    result = live.run_selftest(dict(live.DEFAULT_CFG), str(ROOT / 'examples/rawimu_test.dat'), str(path))
                self.assertNotEqual(result, 0, mode)

    def test_empty_selftest_fails_cleanly(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'empty.dat'
            path.write_bytes(b'')
            with redirect_stdout(io.StringIO()):
                result = live.run_selftest(dict(live.DEFAULT_CFG), str(path))
        self.assertNotEqual(result, 0)

    def test_truth_with_only_header_fails_without_division_by_zero(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'truth.csv'
            path.write_text('t,roll,pitch,yaw\n')
            with redirect_stdout(io.StringIO()):
                result = live.run_selftest(dict(live.DEFAULT_CFG), str(ROOT / 'examples/rawimu_test.dat'), str(path))
        self.assertNotEqual(result, 0)

    def test_truth_comparison_is_not_shifted_by_ring_buffer(self):
        # Independent constant 30 deg/s yaw, 8000 frames exceeds the display ring.
        with tempfile.TemporaryDirectory() as folder:
            data_path = Path(folder) / 'long.dat'
            truth_path = Path(folder) / 'truth.csv'
            template = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72])
            with data_path.open('wb') as f:
                for k in range(8000):
                    struct.pack_into('<d', template, 32, 100.0 + k * .01)
                    struct.pack_into('<iii', template, 56, 30 * 65536, 0, 0)
                    struct.pack_into('<I', template, 68, reference_crc(template[:68]))
                    f.write(template)
            with truth_path.open('w') as f:
                f.write('t,roll,pitch,yaw\n')
                for k in range(8001):
                    f.write('%s,0,0,%s\n' % (k * .01, (k * .3 + 180) % 360 - 180))
            cfg = dict(live.DEFAULT_CFG, mode='gyro', fast_align=False)
            output = io.StringIO()
            with redirect_stdout(output):
                result = live.run_selftest(cfg, str(data_path), str(truth_path))
            self.assertEqual(result, 0)
            self.assertIn('8000', output.getvalue())
            self.assertIn('roll=0.00000 pitch=0.00000 yaw=0.00000', output.getvalue())

    def test_plot_preserves_spikes_and_limits_draw_points(self):
        times = [k * .001 for k in range(10000)]
        values = [0.] * 10000
        values[5001], values[5002] = 80., -90.
        segments = live.plot_segments(times, values, 100)
        self.assertLessEqual(sum(len(s) for s in segments), 402)
        flattened = [point for segment in segments for point in segment]
        self.assertIn((5.001, 80.), flattened)
        self.assertIn((5.002, -90.), flattened)
        self.assertEqual(flattened[0], (0., 0.))
        self.assertEqual(flattened[-1], (9.999, 0.))

    def test_irregular_sampling_keeps_visible_alternating_extrema(self):
        times = [k / 10000. for k in range(9900)] + [1 + k * .59 for k in range(100)]
        values = [0.] * 9900 + [float((k % 2) * 2 - 1) for k in range(100)]
        segments = live.plot_segments(times, values, 100)
        visible = set(point for segment in segments for point in segment)
        self.assertTrue(set(zip(times[9900:], values[9900:])).issubset(visible))

    def test_plot_does_not_bridge_nan_or_angle_wrap(self):
        segments = live.plot_segments(list(range(8)), [1, 2, float('nan'), 3, 4, 179, -179, -178], 100, wrap=True)
        self.assertEqual(segments, [[(0, 1), (1, 2)], [(3, 3), (4, 4), (5, 179)], [(6, -179), (7, -178)]])

    def test_snapshot_attitude_cannot_change_after_return(self):
        s = session()
        att = s.snapshot()[4]
        s.att.bias[0] = 42
        s.att.q = (0, 1, 0, 0)
        self.assertEqual(att.bias, [0, 0, 0])
        self.assertEqual(att.q, (1, 0, 0, 0))

    def test_csv_has_no_extra_windows_cr_and_retains_columns(self):
        s = session()
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, 'record.csv')
            s.start_record(path)
            s.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72])
            s.stop_record()
            data = Path(path).read_bytes()
        self.assertNotIn(b'\r\r\n', data)
        rows = list(csv.reader(io.StringIO(data.decode())))
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(rows[0]), 20)
        self.assertEqual(len(rows[1]), 20)

    def test_record_error_does_not_stop_imu_updates(self):
        class BrokenFile(io.StringIO):
            def write(self, value):
                raise OSError('disk full')

        s = session()
        s.recording = True
        s.csv_file = BrokenFile()
        s.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
        self.assertEqual(s.n_imu, 2)
        self.assertFalse(s.recording)
        self.assertIn('disk full', s.record_error)
        self.assertEqual(s.error, '')


if __name__ == '__main__':
    unittest.main()
