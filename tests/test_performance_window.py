from pathlib import Path
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from test_rawimu_live import ROOT, live
from test_imu_performance import static_data


class PerformanceWindowTests(unittest.TestCase):
    def test_timing_mode_is_explicit_and_report_distinguishes_jitter_from_gaps(self):
        import numpy as np
        view = self.open_window()
        data = static_data()
        data.records[100, 9], data.records[101, 9] = .0095, .0105
        view.source_data = data
        view.confirmed_var.set(True)
        view.start_analysis()
        self.finish(view)
        self.assertIsNotNone(view.result['allan'], view.status_var.get())
        self.assertIn('抖动', view.quality_var.get())
        self.assertIn('近似', view.quality_var.get())
        view.timing_var.set('严格设备时间（2%）')
        view.start_analysis()
        self.finish(view)
        self.assertIsNone(view.result['allan'])
        self.assertIn('严格', view.quality_var.get())

    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.failures = []
        failures = self.failures
        self.root.report_callback_exception = lambda *args: failures.append(args)
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)

    def tearDown(self):
        self.app.on_close()
        self.assertEqual(self.failures, [])

    def open_window(self):
        self.app.open_performance_window()
        self.root.update()
        return self.app.performance_window

    def finish(self, view):
        deadline = time.monotonic() + 8.
        while view.busy and time.monotonic() < deadline:
            self.root.update()
            time.sleep(.01)
        self.assertFalse(view.busy, view.status_var.get())
        self.root.update()

    def test_manual_segment_background_analysis_and_six_axis_display(self):
        view = self.open_window()
        view.source_data = static_data()
        view.start_var.set('5')
        view.end_var.set('35')
        view.confirmed_var.set(True)
        view.start_analysis()
        self.finish(view)
        self.assertIsNotNone(view.result, view.status_var.get())
        self.assertAlmostEqual(view.result['time'][0], 5.)
        self.assertAlmostEqual(view.result['time'][-1], 35.)
        self.assertEqual(len(view.table.get_children()), 6)
        self.assertEqual(len(view.time_panels), 6)
        self.assertEqual(len(view.allan_plots), 2)
        self.assertIsNotNone(view.result['allan'])
        view.notebook.select(1)
        self.root.update()
        for canvas, panel in view.time_panels:
            self.assertTrue(panel.pool)
            self.assertTrue(any(len(canvas.coords(item)) >= 4 for item in panel.pool.values()))
        view.notebook.select(2)
        self.root.update()
        for plot in view.allan_plots:
            self.assertGreater(len(plot.canvas.find_all()), 20)
        self.app.open_performance_window()
        self.assertIs(view, self.app.performance_window)
        view.window.destroy()
        self.assertIsNone(self.app.performance_window)

    def test_analysis_does_not_stop_csv_recording_or_replace_live_session(self):
        view = self.open_window()
        source_session = self.app.sess
        with tempfile.TemporaryDirectory() as folder:
            self.assertTrue(source_session.start_record(str(Path(folder) / 'live.csv')))
            source_session.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:7200])
            view.start_analysis()
            self.finish(view)
            self.assertIsNotNone(view.result, view.status_var.get())
            self.assertIs(self.app.sess, source_session)
            self.assertTrue(source_session.recording)
            self.assertIsNone(view.result['allan'])
            self.assertIn('静止', view.quality_var.get())
            source_session.stop_record()

    def test_session_change_cancels_old_snapshot_job_without_tk_worker_calls(self):
        view = self.open_window()
        self.app.sess.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:7200])
        entered = threading.Event()
        def wait_for_cancel(*args, **kwargs):
            entered.set()
            kwargs['cancel'].wait(3.)
            raise InterruptedError('cancelled')
        with patch('performance_window.analyze_static', side_effect=wait_for_cancel):
            view.start_analysis()
            self.assertTrue(entered.wait(2.))
            self.app._new_session()
            self.finish(view)
        self.assertIsNone(view.result)
        self.assertEqual(self.app.sess.analysis_archive.info()['count'], 0)
        view.window.destroy()
        self.root.update()

    def test_file_load_and_export_are_background_jobs(self):
        view = self.open_window()
        view.load_file(str(ROOT / 'examples/rawimu_test.dat'))
        self.finish(view)
        self.assertEqual(len(view.source_data.records), 2000)
        view.confirmed_var.set(True)
        view.start_analysis()
        self.finish(view)
        self.assertIsNotNone(view.result)
        with tempfile.TemporaryDirectory() as folder:
            prefix = Path(folder) / 'performance'
            view.export_result(str(prefix))
            self.finish(view)
            self.assertTrue(Path(str(prefix) + '_report.json').is_file())
            self.assertTrue(Path(str(prefix) + '_timeseries.csv').is_file())

    def test_switching_sources_requires_new_static_confirmation(self):
        view = self.open_window()
        view.source_data = static_data()
        view.confirmed_var.set(True)
        view.use_current()
        self.assertFalse(view.confirmed_var.get())


if __name__ == '__main__':
    unittest.main()
