from pathlib import Path
import tempfile
import time
import tkinter as tk
import unittest

from test_rawimu_live import ROOT, live
from test_gpimu import gp_line


class GPIMUViewTests(unittest.TestCase):
    def test_hidden_monitor_draws_cached_data_immediately_when_shown(self):
        view = self.render()
        view.notebook.select(1)
        self.root.update()
        self.app.sess.feed(gp_line() + gp_line(100.01))
        self.app._dirty = True
        self.app.refresh()
        self.assertFalse(view.monitor_panels[0][1].render_enabled)
        self.assertFalse(view.monitor_panels[0][1].pool)
        view.notebook.select(0)
        self.root.update_idletasks()
        self.assertTrue(view.monitor_panels[0][1].pool)

    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.failures = []
        failures = self.failures
        self.root.report_callback_exception = lambda *args: failures.append(args)
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False,
                                          auto_accel_scale=False), autostart=False)
        self.root.update_idletasks()

    def tearDown(self):
        self.app.on_close()
        self.assertEqual(self.failures, [])

    def render(self):
        self.root.deiconify()
        self.app.notebook.select(self.app.gpimu_page)
        self.root.update()
        self.app._dirty = True
        self.app.refresh()
        return self.app.gpimu_view

    def test_live_nine_plots_sample_count_temperature_and_freeze(self):
        self.app.sess.feed(gp_line() + gp_line(100.01))
        view = self.render()
        self.assertEqual(len(view.monitor_panels), 9)
        self.assertEqual(view.monitor_panels[0][1]._last_ys[0], [13., 13.])
        self.assertEqual(view.monitor_panels[1][1]._last_ys[0], [31.45, 31.45])
        self.assertAlmostEqual(view.monitor_panels[2][1]._last_ys[0][1], 10., places=3)
        self.assertEqual(view.monitor_panels[3][1]._last_ys[0], [1., 1.])
        self.assertTrue(view.monitor_panels[3][1].pool)
        self.app.toggle_freeze()
        self.app.sess.feed(gp_line(100.02))
        self.app.refresh()
        self.assertEqual(len(view.monitor_panels[0][1]._last_t), 2)
        self.app.toggle_freeze()
        self.assertEqual(len(view.monitor_panels[0][1]._last_t), 3)

    def test_profile_reinterprets_saved_counts_without_changing_rawimu_or_source(self):
        self.app.sess.feed(gp_line())
        view = self.render()
        raw_count = self.app.sess.n_imu
        view.model_var.set('M22')
        self.app._dirty = True
        self.app.refresh()
        self.assertAlmostEqual(view.monitor_panels[5][1]._last_ys[0][-1], 1.)
        self.assertEqual(self.app.sess.n_imu, raw_count)
        self.assertEqual(self.app.cfg['imu_model'], 'M1633')

    def test_m22p_recorded_sample_identification_and_zero_counts(self):
        self.app.sess.feed(b'$GPIMU,339.840967000,00020,00000000,00000000,00000000,00000000,'
                           b'00000000,00000000,1356,354.913671000,101,297,86443,0,0,1E,FFFFF9AC,00000000*49\r\n')
        view = self.render()
        p = view._prepared
        self.assertEqual(p['model'], 'M22P')
        self.assertEqual(p['rows'][0, 14], 30)
        self.assertAlmostEqual(p['temperature'][0], 31.9)
        self.assertEqual(list(p['values'][0]), [0.] * 6)
        self.assertIn('全零 1/1', view.compare_var.get())
        view.model_var.set('M21')
        self.app._dirty = True
        self.app.refresh()
        self.assertEqual(view._prepared['model'], 'M21')

    def test_independent_file_background_load_survives_new_main_session_and_exports(self):
        view = self.render()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'gpimu.dat'
            path.write_bytes(b''.join(gp_line(100+k*.01) for k in range(100)))
            main_source = self.app.sess
            view.load_file(str(path))
            deadline = time.monotonic() + 5
            while view.loading and time.monotonic() < deadline:
                self.root.update()
                time.sleep(.01)
            self.assertFalse(view.loading)
            self.assertIsNotNone(view.file_data, view.status_var.get())
            self.assertIs(self.app.sess, main_source)
            self.app._new_session()
            self.app.refresh()
            self.assertIsNotNone(view.file_data)
            view.inspect_time(.8)
            view.position_var.set(.2)
            view._mark_dirty()
            self.app.refresh()
            self.assertNotIn('t=0.8000', view.cursor_var.get())
            output = Path(folder) / 'gp.csv'
            view.export_csv(str(output))
            self.assertTrue(output.is_file())
            self.assertIn('samples_per_sentence', output.read_text(encoding='utf-8-sig').splitlines()[0])
            view.follow_current()
            self.app.refresh()
            self.assertIsNone(view.file_data)
            self.assertEqual(view.monitor_panels[0][1]._last_t, [])


if __name__ == '__main__':
    unittest.main()
