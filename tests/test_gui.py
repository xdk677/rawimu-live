import sys
import time
import tkinter as tk
import unittest
import tempfile
from pathlib import Path

from test_rawimu_live import ROOT, live


class GuiTests(unittest.TestCase):
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
            self.app.worker.join(2)
        self.app.on_close()
        self.assertEqual(self.failures, [])

    def test_idle_view_has_placeholders_and_source_status(self):
        self.app.refresh()
        self.assertEqual([v.get() for v in self.app.angle_vars], ['—', '—', '—'])
        self.assertIn('未连接', self.app.state_var.get())
        self.assertTrue(callable(self.app.open_replay))

    def test_aircraft_renders_extreme_attitudes_without_new_canvas_items(self):
        import math
        scene = self.app.scene
        scene.resize(420, 320)
        count = len(self.app.cv3d.find_all())
        for angles in ((0, 0, 0), (math.pi / 2, 0, math.pi), (0, math.pi / 2, 0), (math.pi, -.5, 1.2)):
            matrix = live.quat_to_dcm(live.euler_to_quat(*angles))
            scene.update(matrix, *angles, text='F-22')
            for item in scene.faces:
                self.assertTrue(all(math.isfinite(value) for value in self.app.cv3d.coords(item)))
        self.assertEqual(len(self.app.cv3d.find_all()), count)

    def test_dat_button_saves_exact_serial_input(self):
        from unittest.mock import patch
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:144]
        worker = live.SerialWorker(self.app.sess)
        self.app.worker = worker
        self.app.sess.state = 'serial'
        self.app.sess.running = True
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = str(Path(folder) / 'raw.dat')
                with patch.object(worker, 'is_alive', return_value=True), \
                        patch('tkinter.filedialog.asksaveasfilename', return_value=path):
                    self.app.toggle_raw_record()
                    self.app.sess.feed(data)
                    self.app.refresh()
                    self.assertIn('DAT', self.app.record_var.get())
                    self.assertTrue(self.app.sess.raw_recording)
                    self.app.toggle_raw_record()
                    self.assertEqual(Path(path).read_bytes(), data)
        finally:
            self.app.sess.stop_raw_record()
            self.app.worker = None

    def test_disconnect_while_choosing_dat_path_does_not_leave_open_file(self):
        from unittest.mock import patch
        worker = live.SerialWorker(self.app.sess)
        self.app.worker = worker
        self.app.sess.state, self.app.sess.running = 'serial', True
        try:
            with tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'disconnected.dat'
                def choose(**kwargs):
                    worker.stop()
                    self.app.sess.state = 'disconnected'
                    return str(path)
                with patch.object(worker, 'is_alive', return_value=True), \
                        patch('tkinter.filedialog.asksaveasfilename', side_effect=choose):
                    self.app.toggle_raw_record()
                self.assertFalse(path.exists())
                self.assertFalse(self.app.sess.raw_recording)
        finally:
            self.app.sess.stop_raw_record()
            self.app.worker = None

    def test_closing_window_closes_raw_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'close.dat')
            self.app.sess.start_raw_record(path)
            self.app.sess.feed(b'raw input')
            self.app.on_close()
            self.assertEqual(Path(path).read_bytes(), b'raw input')
            self.assertFalse(self.app.sess.raw_recording)

    def test_shutdown_keeps_bytes_from_an_inflight_serial_read(self):
        import serial
        import threading
        from unittest.mock import patch
        entered, release = threading.Event(), threading.Event()
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:72]
        class Port:
            in_waiting = len(data)
            def read(self, size):
                entered.set()
                release.wait(2)
                return data
            def close(self):
                pass
        worker = live.SerialWorker(self.app.sess)
        self.app.worker = worker
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'tail.dat'
            self.app.sess.start_raw_record(str(path))
            with patch.object(serial, 'Serial', return_value=Port()):
                worker.start()
                try:
                    self.assertTrue(entered.wait(1))
                    self.app.on_close()
                finally:
                    release.set()
                    worker.join(2)
            time.sleep(.03)
            self.root.update()
            self.assertEqual(path.read_bytes(), data)
            self.assertEqual(self.app.sess.n_imu, 1)

    def test_status_fields_are_plotted_and_cursor_reads_exact_word(self):
        from test_status_monitor import frames_with_status
        self.app.sess.parser.feed(frames_with_status([0xfff69afe, 0x000101ff, 0x00020000]))
        self.app.refresh()
        self.assertEqual(self.app.status_word_var.get(), '0x00020000')
        self.assertEqual(self.app.p_status_raw._last_ys[0], [0xfff69afe, 0x000101ff, 0x00020000])
        self.assertEqual(self.app.p_counter._last_ys[0], [254, 255, 0])
        self.app.inspect_time(.01)
        self.assertIn('0x000101FF', self.app.cursor_var.get())
        self.assertGreaterEqual(len(self.app.notebook.tabs()), 2)

    def test_fault_lanes_show_matlab_names_word_bits_and_current_state(self):
        from test_status_monitor import frames_with_status
        self.root.deiconify()
        self.app.notebook.select(1)
        self.root.update()
        self.app.sess.parser.feed(frames_with_status([0, 0xa500]))
        self.app.refresh()
        expected = ('GyroX', 'GyroY', 'GyroZ', 'AccX', 'AccY', 'AccZ', 'Temp', 'FUSA')
        for row, name in enumerate(expected):
            label = self.app.cv_status.itemcget(self.app.p_bits.decor[('bitlabel', row)], 'text')
            self.assertEqual(label, '%s [B%d] %d' % (name, row + 8, (0xa5 >> row) & 1))

    def test_freezing_display_keeps_sampling_and_unfreezing_catches_up(self):
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()
        self.app.sess.parser.feed(data[:144])
        self.app.refresh()
        self.app.toggle_freeze()
        self.app.sess.parser.feed(data[144:216])
        self.app.refresh()
        self.assertEqual(self.app.sess.n_imu, 3)
        self.assertEqual(len(self.app.p_eul._last_t), 2)
        self.app.toggle_freeze()
        self.app.refresh()
        self.assertEqual(len(self.app.p_eul._last_t), 3)

    def test_source_reset_removes_old_cursor(self):
        self.root.deiconify()
        self.root.update()
        self.app.sess.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:216])
        self.app.refresh()
        self.app.inspect_time(.01)
        cursor = self.app.p_eul._cursor
        self.assertEqual(self.app.cv_eul.itemcget(cursor, 'state'), 'normal')
        self.app._new_session()
        self.app.refresh()
        self.assertEqual(self.app.cv_eul.itemcget(cursor, 'state'), 'hidden')

    def test_compact_angles_fit_their_panels(self):
        from tkinter.font import Font
        self.root.deiconify()
        self.root.geometry('1100x720')
        self.root.update()
        for var in self.app.angle_vars:
            var.set('-180.00')
        self.root.update_idletasks()
        def labels(widget):
            for child in widget.winfo_children():
                if child.winfo_class() == 'Label' and str(child.cget('textvariable')) in [str(v) for v in self.app.angle_vars]:
                    yield child
                yield from labels(child)
        values = list(labels(self.root))
        self.assertEqual(len(values), 3)
        for label in values:
            available = min(label.winfo_width(), label.master.winfo_width() - 8)
            self.assertLessEqual(Font(font=label.cget('font')).measure('-180.00'), available)

    def test_digital_plot_preserves_held_state_at_window_start(self):
        import struct
        from test_status_monitor import frames_with_status, reference_crc
        data = frames_with_status([0x100, 0x100, 0])
        for index, seconds in enumerate((100., 125., 140.)):
            struct.pack_into('<d', data, index * 72 + 32, seconds)
            struct.pack_into('<I', data, index * 72 + 68, reference_crc(data[index * 72:index * 72 + 68]))
        self.root.deiconify()
        self.app.notebook.select(1)
        self.root.update()
        self.app.sess.parser.feed(data)
        self.app.refresh()
        line = self.app.p_bits.pool[(0, 0)]
        coordinates = self.app.cv_status.coords(line)
        self.assertAlmostEqual(coordinates[0], self.app.p_bits._bounds[0])

    def test_integrity_is_in_overview_and_faults_are_in_status_analysis(self):
        self.assertTrue(str(self.app.cv_integrity).startswith(str(self.app.overview_page)))
        self.assertTrue(str(self.app.cv_status).startswith(str(self.app.status_page)))
        self.assertEqual(self.app.notebook.tab(0, 'text'), '监控总览')
        self.assertEqual(int(self.app.cv_status.grid_info()['row']), 0)
        self.assertEqual(int(self.app.cv_status.grid_info()['column']), 1)
        self.assertEqual(self.app.cv_status.master, self.app.status_page)

    def test_integrity_panel_plots_time_and_counter_intervals(self):
        from test_status_monitor import frames_with_status
        self.app.sess.parser.feed(frames_with_status([254, 255, 0, 3]))
        self.app.refresh()
        panel = self.app.p_integrity
        self.assertEqual(panel.counter_plot._last_ys[0][1:], [1, 1, 3])
        self.assertAlmostEqual(panel.time_plot._last_ys[0][1], 10, places=3)
        self.assertEqual(panel.time_plot.time_range, self.app.p_eul.time_range)

    def test_integrity_refreshes_when_only_crc_errors_arrive(self):
        from test_status_monitor import frames_with_status
        self.app.refresh()
        frame = frames_with_status([0])
        frame[-1] ^= 1
        self.app.sess.parser.feed(frame)
        self.app.refresh()
        self.assertEqual(self.app.p_integrity.metrics['crc_errors'], 1)

    def test_diagnostics_keep_status_and_bias_accessible(self):
        self.app.open_details()
        self.app.sess.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
        self.app.refresh()
        content = self.app.details_text.get('1.0', 'end')
        self.assertIn('RAWIMU', content)
        self.assertIn('Bias', content)
        self.assertIn('Status', content)

    def test_replay_finishes_and_new_source_resets_samples(self):
        path = str(ROOT / 'examples/rawimu_test.dat')
        self.app.start_replay(path, speed=0)
        self.app.worker.join(2)
        self.app.refresh()
        self.assertFalse(self.app.worker.is_alive())
        self.assertEqual(self.app.sess.n_imu, 2000)
        self.assertIn('结束', self.app.state_var.get())
        self.assertNotEqual(self.app.angle_vars[0].get(), '—')
        self.app.start_replay(path, speed=0)
        self.app.worker.join(2)
        self.app.refresh()
        self.assertEqual(self.app.sess.n_imu, 2000)

    def test_connection_error_remains_visible_with_and_without_samples(self):
        self.app.sess.state = 'error'
        self.app.sess.error = 'TEST PORT ERROR'
        self.app.refresh()
        self.assertIn('TEST PORT ERROR', self.app.message_var.get())
        self.app.sess.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:144])
        self.app.refresh()
        self.assertIn('TEST PORT ERROR', self.app.message_var.get())

    def test_resize_does_not_create_unbounded_canvas_objects(self):
        self.root.deiconify()
        self.root.geometry('1100x720')
        self.root.update()
        self.app.sess.parser.feed((ROOT / 'examples/rawimu_test.dat').read_bytes())
        self.app.refresh()
        initial = len(self.app.cv_eul.find_all())
        for _ in range(8):
            self.app._dirty = True
            self.app.refresh()
        self.assertLessEqual(len(self.app.cv_eul.find_all()), initial + 5)
        self.assertGreater(self.app.cv3d.winfo_width(), 250)
        self.assertGreater(self.app.cv_eul.winfo_height(), 100)

    def test_scrolling_axis_reuses_tick_items(self):
        self.root.deiconify()
        self.root.update()
        panel = self.app.p_gyr
        panel.resize(650, 220)
        for k in range(3):
            panel.update([k, k + 1], [[k * 1000., k * 1000. + 5.]] * 3)
        count = len(panel.c.find_all())
        for k in range(3, 30):
            panel.update([k, k + 1], [[k * 1000., k * 1000. + 5.]] * 3)
        self.assertLessEqual(len(panel.c.find_all()), count + 10)

    def test_screenshot_uses_explicit_window_when_titles_match(self):
        if not sys.platform.startswith('win'):
            self.skipTest('PrintWindow and WM frame handles are Windows-only')
        try:
            from PIL import Image
        except ImportError:
            self.skipTest('Pillow is optional')
        self.root.title('RAWIMU screenshot regression')
        self.root.deiconify()
        other = tk.Toplevel(self.root)
        other.title(self.root.title())
        other.geometry('320x200')
        self.root.update()
        other.lift()
        self.root.update()
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder) / 'window.png')
            self.assertTrue(live.grab_window(self.root.title(), path, int(self.root.wm_frame(), 16)))
            with Image.open(path) as captured:
                self.assertGreater(captured.width, 1000)
        other.destroy()


if __name__ == '__main__':
    unittest.main()
