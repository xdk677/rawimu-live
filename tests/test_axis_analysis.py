import math
import struct
import tkinter as tk
import unittest

from test_rawimu_live import ROOT, live, session, reference_crc


class AxisDataTests(unittest.TestCase):
    def test_week_history_preserves_raw_week_and_stays_aligned_when_resized(self):
        s = session()
        data = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes()[:216])
        for k, week in enumerate((2446, 1356, 2446)):
            struct.pack_into('<I', data, k * 72 + 28, week)
            struct.pack_into('<I', data, k * 72 + 68, reference_crc(data[k * 72:k * 72 + 68]))
        s.parser.feed(data)
        self.assertEqual(s.status_snapshot()['week'], [2446, 1356, 2446])
        s.set_model({'FAST': {'freq': 1000}}, 'FAST')
        self.assertEqual(s.status_snapshot()['week'], [2446, 1356, 2446])
        self.assertEqual(s.weeks.maxlen, s.t.maxlen)

    def test_average_uses_elapsed_time_and_resets_at_missing_data_or_time_regression(self):
        from axis_analysis import trailing_average
        result = trailing_average([0, 1, 4, 7, 8, 9, 0, 1], [1, 3, 5, 9, float('nan'), 11, 20, 30])
        self.assertEqual(result[:4], [1, 2, 3, 7])
        self.assertTrue(math.isnan(result[4]))
        self.assertEqual(result[5:], [11, 20, 25])


class AxisWindowTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.failures = []
        failures = self.failures
        self.root.report_callback_exception = lambda *args: failures.append(args)
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False,
                                          auto_accel_scale=False), autostart=False)

    def tearDown(self):
        self.app.on_close()
        self.assertEqual(self.failures, [])

    def open_window(self):
        self.app.open_axes_window()
        self.root.update()
        self.app.refresh()
        return self.app.axes_window

    def test_nine_plots_match_reference_layout_and_main_axis_values(self):
        self.app.sess.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:14400])
        view = self.open_window()
        self.assertEqual(len(view.panels), 9)
        for k, (canvas, panel) in enumerate(view.panels):
            self.assertEqual(int(canvas.grid_info()['row']), k // 3)
            self.assertEqual(int(canvas.grid_info()['column']), k % 3)
            self.assertEqual(panel.time_range, self.app.p_gyr.time_range)
        self.assertAlmostEqual(view.panels[0][1]._last_ys[0][1], 10., places=3)
        self.assertEqual(view.panels[1][1]._last_ys[0], self.app._status_view['temperature'])
        self.assertEqual(view.panels[2][1]._last_ys[0], self.app._status_view['week'])
        for k, (_, panel) in enumerate(view.panels[3:]):
            expected = (self.app.p_gyr if k < 3 else self.app.p_acc)._last_ys[k % 3]
            self.assertEqual(panel._last_ys[0], expected)
            self.assertEqual(len(panel._last_ys[1]), len(expected))
            self.assertIn('PP=', panel.series[1])
        self.assertEqual(len({id(panel) for _, panel in view.panels}), 9)

    def test_individual_axis_range_can_show_small_sensor_noise(self):
        view = self.open_window()
        panel = view.panels[3][1]
        low, high, _ = panel._y_range([[.00001, .00003, -.00001]])
        self.assertLess(high - low, .001)
        self.assertLess(low, -.00001)
        self.assertGreater(high, .00003)

    def test_average_keeps_context_before_visible_window_and_restores_after_unmap(self):
        from axis_analysis import trailing_average
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()
        self.app.sess.feed(data[:72000])
        view = self.open_window()
        view.window.withdraw()
        self.root.update()
        self.app.sess.feed(data[72000:])
        self.app.window_var.set('10')
        self.app.on_window()
        view.window.deiconify()
        self.root.update()
        self.app.refresh()
        times, channels = self.app._display_data[0][:2]
        panel = view.panels[3][1]
        first = times.index(panel._last_t[0])
        expected = trailing_average(times, channels[3])[first:]
        for actual, wanted in zip(panel._last_ys[1], expected):
            self.assertAlmostEqual(actual, wanted, places=8)
        self.assertGreater(first, 0)
        self.assertEqual(panel.time_range, (self.app.p_gyr.time_range))

    def test_sixty_second_window_keeps_five_second_average_context_after_ring_rollover(self):
        frame = bytearray((ROOT / 'examples/rawimu_test.dat').read_bytes()[:72])
        data = bytearray()
        for k in range(7101):
            struct.pack_into('<d', frame, 32, 100. + k * .01)
            struct.pack_into('<i', frame, 64, int(round(k * .01 * 65536)))
            struct.pack_into('<I', frame, 68, reference_crc(frame[:68]))
            data.extend(frame)
        self.app.sess.feed(data)
        self.app.window_var.set('60')
        self.app.on_window()
        view = self.open_window()
        panel = view.panels[3][1]
        raw, smooth = panel._last_ys
        self.assertAlmostEqual(smooth[0], raw[0] - 2.5, delta=.02)
        self.assertAlmostEqual(smooth[-1] - smooth[1], 60., delta=.03)

    def test_frozen_window_reopens_on_same_snapshot_and_cursor_is_synchronized(self):
        data = (ROOT / 'examples/rawimu_test.dat').read_bytes()
        self.app.sess.feed(data[:7200])
        view = self.open_window()
        before = list(view.panels[3][1]._last_ys[0])
        self.app.toggle_freeze()
        self.app.sess.feed(data[7200:14400])
        self.app.open_axes_window()
        self.assertIs(self.app.axes_window, view)
        self.app.refresh()
        self.assertEqual(view.panels[3][1]._last_ys[0], before)
        self.app.inspect_time(.5)
        self.assertIn('GX=', view.cursor_var.get())
        for canvas, panel in view.panels:
            self.assertEqual(canvas.itemcget(panel._cursor, 'state'), 'normal')
        view.window.destroy()
        self.assertIsNone(self.app.axes_window)
        view = self.open_window()
        self.assertEqual(view.panels[3][1]._last_ys[0], before)
        self.app.toggle_freeze()
        self.assertGreater(len(view.panels[3][1]._last_ys[0]), len(before))

    def test_new_session_clears_old_data_and_closed_window_does_not_break_refresh(self):
        self.app.sess.feed((ROOT / 'examples/rawimu_test.dat').read_bytes()[:720])
        view = self.open_window()
        self.app.inspect_time(.05)
        self.app._new_session()
        self.app.refresh()
        self.assertEqual(view.panels[3][1]._last_t, [])
        self.assertNotIn('GX=', view.cursor_var.get())
        view.window.destroy()
        self.app.refresh()
        self.assertIsNone(self.app.axes_window)


if __name__ == '__main__':
    unittest.main()
