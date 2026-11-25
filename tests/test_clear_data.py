import csv
import tempfile
from pathlib import Path
import tkinter as tk
import unittest

from test_rawimu_live import live, session, ROOT
from test_gpimu import gp_line


class ClearDataTests(unittest.TestCase):
    def test_clear_keeps_parser_connection_and_recordings_but_starts_new_display(self):
        s = session()
        data = (ROOT/'examples/rawimu_test.dat').read_bytes()
        try:
            with tempfile.TemporaryDirectory() as folder:
                dat, output = Path(folder)/'raw.dat', Path(folder)/'data.csv'
                s.start_raw_record(str(dat))
                s.start_record(str(output))
                s.state, s.running = 'serial', True
                s.feed(data[:72*10+20])
                parser, attitude = s.parser, s.att
                s.clear_data()
                self.assertIs(s.parser, parser)
                self.assertIs(s.att, attitude)
                self.assertTrue(s.recording and s.raw_recording and s.running)
                self.assertEqual(s.state, 'serial')
                self.assertEqual(s.n_imu, 0)
                self.assertEqual(s.snapshot()[0], [])
                self.assertEqual(s.analysis_archive.info()['count'], 0)
                s.feed(data[72*10+20:72*12])
                self.assertEqual(s.n_imu, 2)
                self.assertEqual(s.total_imu_received, 12)
                self.assertEqual(s.snapshot()[0][0], 0.)
                self.assertEqual(len(s.status_snapshot()['header_interval_ms']), 2)
                s.stop_record()
                s.stop_raw_record()
                self.assertEqual(dat.read_bytes(), data[:72*12])
                with output.open() as source:
                    rows = list(csv.DictReader(source))
                self.assertEqual(len(rows), 12)
                times = [float(row['t_s']) for row in rows]
                self.assertTrue(all(b>a for a,b in zip(times,times[1:])))
        finally:
            s.close_analysis()

    def test_clear_gpimu_and_replay_lifetime_counts(self):
        s = session()
        try:
            s.replay_elapsed = 100.
            s.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[:72])
            s.feed(gp_line())
            s.clear_data()
            s.replay_elapsed = 101.
            s.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[72:144])
            s.feed(gp_line(100.01))
            self.assertEqual(s.total_imu_received, 2)
            self.assertEqual(s.total_gpimu_received, 2)
            self.assertEqual(s.t[0], 0.)
            self.assertEqual(s.gpimu.total, 1)
            self.assertEqual(s.gpimu.rows[0][0], 0.)
        finally:
            s.close_analysis()

    def test_clear_button_exits_freeze_and_redraws_hidden_and_visible_charts(self):
        root = tk.Tk()
        root.withdraw()
        app = live.App(root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
        try:
            app.sess.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[:7200])
            app.open_axes_window()
            root.update()
            app.toggle_freeze()
            app.inspect_time(.5)
            app.clear_data()
            self.assertFalse(app.frozen)
            self.assertEqual(app.p_gyr._last_t, [])
            self.assertEqual(app.axes_window.panels[3][1]._last_t, [])
            self.assertIsNone(app._cursor_time)
            app.sess.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[7200:7344])
            app.refresh()
            self.assertEqual(app.p_gyr._last_t[0], 0.)
            self.assertEqual(app.sess.n_imu, 2)
        finally:
            app.on_close()

    def test_active_replay_keeps_index_valid_after_clear_and_does_not_restart_file(self):
        s = session()
        try:
            process = s.parser.on_frame
            def callback(spec, body):
                process(spec, body)
                if s.total_imu_received == 10:
                    s.clear_data()
            s.parser.on_frame = callback
            worker = live.ReplayWorker(s, str(ROOT/'examples/rawimu_test.dat'), speed=0)
            worker.run()
            self.assertEqual(s.error, '')
            self.assertEqual(s.state, 'finished')
            self.assertEqual(s.total_imu_received, 2000)
            self.assertEqual(s.n_imu, 1990)
            self.assertEqual(s.t[0], 0.)
        finally:
            s.close_analysis()
