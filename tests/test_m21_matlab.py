import unittest
import numpy as np
import tkinter as tk
from test_rawimu_live import live, ROOT


class M21MatlabTests(unittest.TestCase):
    def config(self, calibration=False):
        cfg = dict(live.DEFAULT_CFG, auto_bias=calibration, auto_accel_scale=calibration)
        live.apply_model(cfg, live.IMU_MODELS, 'M21')
        return cfg

    def test_m21_scale_and_temperature_follow_b240_matlab_model(self):
        raw = {'acc': [21668., 66844., 1304019.], 'gyr': [-3220., 7993., -14498.],
               'status': (2685 << 16) | 48, 'week': 2446, 'sow': 197882.6129}
        value = live.to_physical(raw, {'name': 'RAWIMU'}, self.config())
        np.testing.assert_allclose(value['acc'], np.array([21668., -66844., 1304019.])*9.80665/(20*65536))
        np.testing.assert_allclose(value['gyr'], np.array([-3220., -7993., -14498.])/65536)
        self.assertAlmostEqual(value['temp'], 2685/256.+25.)

    def test_physical_histories_stay_uncalibrated_while_attitude_calibrates(self):
        s = live.Session(self.config(True))
        try:
            s.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[:36000])
            history = s.status_snapshot()
            archive = s.analysis_archive.read().records
            actual = np.column_stack([history[key] for key in ('gx','gy','gz','ax','ay','az')])
            np.testing.assert_allclose(actual, archive[:, 2:8], rtol=0, atol=1e-12)
            self.assertTrue(s.att.bias_done)
            self.assertTrue(s.att.acc_cal_done)
            self.assertGreater(np.max(np.abs(actual[:, :3]-np.asarray(s.snapshot()[1][3:6]).T)), .01)
        finally:
            s.close_analysis()

    def test_matlab_centered_movmean_even_window_and_shrinking_edges(self):
        from axis_analysis import centered_sample_average
        self.assertEqual(centered_sample_average([1., 2., 3., 4., 5., 6.], 4),
                         [1.5, 2., 2.5, 3.5, 4.5, 5.])

    def test_m21_ui_defaults_to_raw_and_freeze_can_switch_to_compensated(self):
        root = tk.Tk()
        root.withdraw()
        app = live.App(root, dict(self.config(True), renderer='canvas'), autostart=False)
        try:
            app.sess.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[:36000])
            app.open_axes_window()
            root.update()
            app.toggle_freeze()
            raw = list(app.p_gyr._last_ys[0])
            self.assertEqual(raw, app._status_view['gx'])
            self.assertEqual(app.axes_window.smoothing_var.get(), 'MATLAB 居中5s')
            self.assertEqual(app.axes_window.panels[3][1]._last_ys[0], raw)
            self.assertEqual(app.axes_window.panels[0][1]._last_ys[0], app._status_view['header_interval_ms'])
            app.measurement_var.set('解算补偿量')
            app.on_measurement_changed()
            self.assertNotEqual(app.p_gyr._last_ys[0], raw)
            self.assertEqual(app.p_gyr._last_ys[0], app._display_data[0][1][3])
            app.measurement_var.set('原始物理量')
            app.on_measurement_changed()
            self.assertEqual(app.p_gyr._last_ys[0], raw)
            self.assertTrue(app.frozen)
        finally:
            app.on_close()

    def test_header_time_interval_matches_matlab_without_hiding_body_jitter(self):
        import struct
        from test_rawimu_live import reference_crc
        s = live.Session(self.config())
        data = bytearray((ROOT/'examples/rawimu_test.dat').read_bytes()[:144])
        for index in range(2):
            struct.pack_into('<I', data, index*72+16, 100000+index*10)
            struct.pack_into('<d', data, index*72+32, 100.+index*.01005)
            struct.pack_into('<I', data, index*72+68, reference_crc(data[index*72:index*72+68]))
        try:
            s.feed(data)
            history = s.status_snapshot()
            self.assertAlmostEqual(history['header_interval_ms'][1], 10., places=6)
            self.assertAlmostEqual(history['time_interval_ms'][1], 10.05, places=3)
        finally:
            s.close_analysis()

