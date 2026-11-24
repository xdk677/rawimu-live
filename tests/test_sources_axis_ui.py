import socket
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

from test_rawimu_live import ROOT, live
from test_network_source import wait_for


class SourceAndAxisUITests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas', auto_bias=False,
                                          auto_accel_scale=False), autostart=False)
        self.root.update_idletasks()

    def tearDown(self):
        if self.app.worker:
            self.app.worker.stop()
            self.app.worker.join(2)
        self.app.on_close()

    def test_source_controls_keep_serial_and_validate_network_before_replacing_session(self):
        app = self.app
        self.assertEqual(app.source_var.get(), '串口')
        app.source_var.set('TCP 服务端')
        app.on_source_changed()
        self.assertEqual(app.host_var.get(), '0.0.0.0')
        original = app.sess
        app.network_port_var.set('70000')
        app.toggle_port()
        self.assertIs(app.sess, original)
        self.assertIn('65535', app.sess.error)
        app.source_var.set('串口')
        app.on_source_changed()
        self.assertEqual(app.btn_open.cget('text'), '连接串口')

    def test_udp_live_connection_can_record_dat_through_existing_action(self):
        app = self.app
        with socket.socket(type=socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        app.source_var.set('UDP 接收')
        app.on_source_changed()
        app.host_var.set('127.0.0.1')
        app.network_port_var.set(str(port))
        app.toggle_port()
        self.assertTrue(app.worker.ready.wait(2))
        app.refresh()
        self.assertEqual(str(app.source_box.cget('state')), 'disabled')
        self.assertEqual(str(app.btn_raw.cget('state')), 'normal')
        with tempfile.TemporaryDirectory() as folder, socket.socket(type=socket.SOCK_DGRAM) as sender:
            path = Path(folder)/'udp.dat'
            with patch('tkinter.filedialog.asksaveasfilename', return_value=str(path)):
                app.toggle_raw_record()
            data = (ROOT/'examples/rawimu_test.dat').read_bytes()[:720]
            sender.sendto(data, ('127.0.0.1', port))
            self.assertTrue(wait_for(lambda: app.sess.n_imu == 10))
            app.toggle_raw_record()
            self.assertEqual(path.read_bytes(), data)
        app.toggle_port()
        app.worker.join(2)
        app.refresh()
        self.assertEqual(str(app.source_box.cget('state')), 'readonly')

    def test_sow_switch_preserves_zoom_and_cursor_and_reaches_all_views(self):
        app = self.app
        app.sess.feed((ROOT/'examples/rawimu_test.dat').read_bytes()[:7200])
        self.root.deiconify()
        app.open_axes_window()
        self.root.update()
        app.toggle_freeze()
        panel = app.p_gyr
        left, top, width, height = panel._bounds[:4]
        panel.c.event_generate('<MouseWheel>', x=int(left+width/2), y=int(top+height/2), delta=120)
        zoom = panel.navigation.view
        app.time_axis_var.set('GPS 周内秒 (s)')
        app.on_axis_changed()
        self.assertEqual(panel.navigation.view, zoom)
        axis = panel.time_axis
        self.assertEqual(axis.mode, 'sow')
        self.assertAlmostEqual(axis.seconds_at(app._status_view['time'][-1]), app.sess.smp['sec'])
        self.assertIs(app.p_integrity.time_plot.time_axis, axis)
        self.assertIs(app.p_bits.time_axis, axis)
        self.assertIs(app.axes_window.panels[3][1].time_axis, axis)
        app.inspect_time(app._status_view['time'][-1])
        self.assertIn('SOW=', app.cursor_var.get())
        self.assertIn('SOW=', app.axes_window.cursor_var.get())
        app.time_axis_var.set('相对时间 (s)')
        app.on_axis_changed()
        self.assertEqual(app.p_gyr.time_axis.mode, 'elapsed')

    def test_gpimu_axis_uses_its_own_clock_and_no_gps_is_explicit(self):
        from test_gpimu import gp_line
        app = self.app
        app.sess.feed(gp_line() + gp_line(100.01))
        self.root.deiconify()
        app.notebook.select(app.gpimu_page)
        self.root.update()
        app.time_axis_var.set('GPS 周内秒 (s)')
        app.on_axis_changed()
        view = app.gpimu_view
        axis = view.monitor_panels[3][1].time_axis
        self.assertEqual(axis.mode, 'sow')
        self.assertAlmostEqual(axis.seconds_at(view._prepared['time'][-1]), view._prepared['rows'][-1, 4])
        view.inspect_time(view._prepared['time'][-1])
        self.assertIn('SOW=', view.cursor_var.get())
        self.assertIs(view.compare_panels[0][1].time_axis, axis)

    def test_static_smoothing_switches_clock_without_changing_allan_tau(self):
        from imu_performance import analyze_static, display_data
        app = self.app
        app.sess.feed((ROOT/'examples/rawimu_test.dat').read_bytes())
        result = analyze_static(app.sess.analysis_archive.read(), confirmed_static=True)
        result['display'] = display_data(result)
        app.open_performance_window()
        window = app.performance_window
        window._show_result(result)
        window.notebook.select(1)
        self.root.update()
        app.time_axis_var.set('GPS 周内秒 (s)')
        app.on_axis_changed()
        axis = window.time_panels[0][1].time_axis
        self.assertAlmostEqual(axis.seconds_at(float(result['time'][-1])), app.sess.smp['sec'])
        self.assertIs(window.allan_plots[0].tau, result['allan']['tau'])
        self.assertEqual(window.time_panels[0][1]._last_t, result['display'][0].tolist())


class SourceCLIArgumentsTests(unittest.TestCase):
    def test_network_and_axis_cli_config(self):
        configs = []
        with patch.object(live, 'App', side_effect=lambda root, cfg, **kwargs: configs.append(cfg)), patch('tkinter.Tk'):
            self.assertEqual(live.main(['--source', 'udp', '--host', '0.0.0.0', '--network-port', '3001',
                                        '--time-axis', 'sow', '--no-autostart']), 0)
        self.assertEqual(configs[0]['source'], 'udp')
        self.assertEqual(configs[0]['network_port'], 3001)
        self.assertEqual(configs[0]['time_axis'], 'sow')

