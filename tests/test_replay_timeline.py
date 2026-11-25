import copy
from pathlib import Path
import struct
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from test_rawimu_live import ROOT, live, session, reference_crc


class ReplayIndexTests(unittest.TestCase):
    def test_malformed_recognized_ascii_does_not_consume_valid_index_entries(self):
        data = b'#RAWIMUA,garbage\r\n' + (ROOT / 'examples/rawimua_ascii.dat').read_bytes()
        expected = session()
        expected.parser.feed(data)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'mixed.dat'
            path.write_bytes(data)
            actual = session()
            worker = live.ReplayWorker(actual, str(path), 0)
            worker.run()
            self.assertEqual(actual.n_imu, expected.n_imu)
            self.assertFalse(actual.error)
            self.assertAlmostEqual(actual.t[0], 0.)
            self.assertEqual(actual.smp, expected.smp)
            actual.close_analysis()
        expected.close_analysis()

    def test_mutation_while_paused_reports_error_instead_of_normal_eof(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'changing.dat'
            data = (ROOT / 'examples/rawimu_test.dat').read_bytes()
            path.write_bytes(data[:720])
            s = session()
            worker = live.ReplayWorker(s, str(path), 0, paused=True)
            worker.start()
            try:
                deadline = time.monotonic() + 2
                while s.state != 'paused' and time.monotonic() < deadline:
                    time.sleep(.01)
                with path.open('ab') as output:
                    output.write(data[720:792])
                worker.set_paused(False)
                worker.join(2)
                self.assertEqual(s.state, 'error')
                self.assertIn('变化', s.error)
            finally:
                worker.stop()
                worker.join(2)
                s.close_analysis()

    def test_frame_offsets_survive_chunk_splits_and_mixed_ascii(self):
        frame = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:72]
        line = next(line for line in (ROOT / 'examples/rawimua_ascii.dat').read_bytes().splitlines(True)
                    if line.startswith(b'#RAWIMUA,'))
        data = b'noise' + frame + b'junk' + line + frame
        offsets = []
        parser = live.FrameParser(lambda spec, body: offsets.append(parser.frame_end), live.FormatRegistry())
        for start in range(0, len(data), 17):
            parser.feed(data[start:start + 17])
        self.assertEqual(offsets, [77, 81 + len(line), len(data)])

    def test_index_duration_offsets_and_clock_regression_normalization(self):
        from replay_timeline import build_replay_index
        frame = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72])
        data = bytearray()
        for k, seconds in enumerate((100., 100.01, 90., 90.01)):
            struct.pack_into('<d', frame, 32, seconds)
            struct.pack_into('<I', frame, 68, reference_crc(frame[:68]))
            data.extend(frame)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'replay.dat'
            path.write_bytes(data)
            index = build_replay_index(path, live.FormatRegistry(), 100.)
            self.assertEqual(list(index.ends), [72, 144, 216, 288])
            self.assertAlmostEqual(index.duration, .03, places=5)
            self.assertTrue(all(a < b for a, b in zip(index.times, index.times[1:])))
            path.write_bytes(data + b'changed')
            self.assertFalse(index.matches(path))

    def test_seek_rebuilds_exact_prefix_attitude_and_stop_interrupts_pause(self):
        expected = session()
        expected.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:721 * 72])
        actual = session()
        worker = live.ReplayWorker(actual, str(ROOT / 'examples/rawimu_test.dat'), 1., start_position=7.2, paused=True)
        worker.start()
        try:
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                info = worker.timeline()
                if info['ready'] and not info['seeking'] and actual.n_imu >= 721:
                    break
                time.sleep(.01)
            self.assertEqual(actual.n_imu, 721)
            self.assertAlmostEqual(worker.timeline()['position'], 7.2, places=5)
            for observed, wanted in zip(actual.att.q, expected.att.q):
                self.assertAlmostEqual(observed, wanted, places=10)
            count = actual.n_imu
            time.sleep(.06)
            self.assertEqual(actual.n_imu, count)
        finally:
            worker.stop()
            worker.join(2)
            expected.close_analysis()
            actual.close_analysis()
        self.assertFalse(worker.is_alive())

    def test_ascii_index_and_playback_reach_same_last_sample(self):
        s = session()
        worker = live.ReplayWorker(s, str(ROOT / 'examples/rawimua_ascii.dat'), 0)
        worker.run()
        self.assertEqual(s.n_imu, 797)
        self.assertTrue(worker.timeline()['ready'])
        self.assertAlmostEqual(worker.timeline()['position'], worker.timeline()['duration'])
        self.assertEqual(s.parser.n_text_crc_bad, 0)
        s.close_analysis()

    def test_resume_after_pause_reaches_end_without_processing_while_paused(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'short.dat'
            path.write_bytes((ROOT / 'examples/rawimu_test.dat').read_bytes()[:720])
            s = session()
            worker = live.ReplayWorker(s, str(path), 1., paused=True)
            worker.start()
            try:
                deadline = time.monotonic() + 2
                while s.state != 'paused' and time.monotonic() < deadline:
                    time.sleep(.01)
                self.assertEqual(s.n_imu, 1)
                time.sleep(.04)
                self.assertEqual(s.n_imu, 1)
                worker.set_paused(False)
                worker.join(2)
                self.assertFalse(worker.is_alive())
                self.assertEqual(s.n_imu, 10)
                self.assertEqual(s.state, 'finished')
            finally:
                worker.stop()
                worker.join(2)
                s.close_analysis()


class ReplayTimelineGuiTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.failures = []
        failures = self.failures
        self.root.report_callback_exception = lambda *args: failures.append(args)
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
        self.root.update_idletasks()

    def tearDown(self):
        if self.app.worker:
            self.app.worker.stop()
            if self.app.worker.ident is not None:
                self.app.worker.join(2)
        self.app.on_close()
        self.assertEqual(self.failures, [])

    def finish_seek(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            self.app.tick_replay_seek()
            self.app.refresh()
            self.root.update()
            worker = self.app.worker
            if self.app._pending_replay_seek is None and worker.timeline()['ready'] and not worker.timeline()['seeking']:
                self.app._dirty = True
                self.app.refresh()
                return
            time.sleep(.01)
        self.fail('seek did not finish')

    def test_timeline_after_eof_seek_pauses_and_clears_on_serial_session(self):
        self.app.start_replay(str(ROOT / 'examples/rawimu_test.dat'), speed=0)
        self.app.worker.join(3)
        self.app.refresh()
        self.assertAlmostEqual(self.app.replay_timeline.position_var.get(), 19.99, places=4)
        self.assertIn('00:19.990', self.app.replay_timeline.time_var.get())
        self.app.seek_replay(7.2)
        self.finish_seek()
        self.assertTrue(self.app.worker.timeline()['paused'])
        self.assertAlmostEqual(self.app.sess.t_last, 7.2, places=5)
        self.assertEqual(len(self.app.p_gyr._last_t), 721)
        self.app.worker.stop()
        self.app.worker.join(2)
        self.app._new_session()
        self.app.worker = None
        self.app.refresh()
        self.assertFalse(self.app.replay_timeline.frame.winfo_manager())

    def test_csv_recording_blocks_seek_but_pause_is_separate_from_display_freeze(self):
        self.app.start_replay(str(ROOT / 'examples/rawimu_test.dat'), speed=0)
        self.app.worker.join(3)
        with tempfile.TemporaryDirectory() as folder:
            self.app.sess.start_record(str(Path(folder) / 'out.csv'))
            original = self.app.sess
            try:
                self.app.seek_replay(4.)
                self.assertIsNone(self.app._pending_replay_seek)
                self.assertIs(self.app.sess, original)
                self.assertTrue(original.recording)
            finally:
                original.stop_record()
        self.assertFalse(self.app.frozen)

    def test_csv_cannot_start_during_seek_or_if_seek_begins_in_save_dialog(self):
        from replay_timeline import build_replay_index
        path = ROOT / 'examples/rawimu_test.dat'
        index = build_replay_index(path, self.app.formats, 100.)
        self.app.worker = live.ReplayWorker(self.app.sess, str(path), 1., index=index, start_position=10.)
        with tempfile.TemporaryDirectory() as folder:
            output = str(Path(folder) / 'output.csv')
            try:
                with patch('tkinter.filedialog.asksaveasfilename', return_value=output):
                    self.app.toggle_record()
                self.assertFalse(self.app.sess.recording)
                self.app.worker._seeking = False
                def select(**kwargs):
                    self.app.worker._seeking = True
                    return output
                with patch('tkinter.filedialog.asksaveasfilename', side_effect=select):
                    self.app.toggle_record()
                self.assertFalse(self.app.sess.recording)
            finally:
                self.app.sess.stop_record()


if __name__ == '__main__':
    unittest.main()
