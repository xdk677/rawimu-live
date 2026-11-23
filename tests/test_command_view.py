import socket
import tkinter as tk
import unittest
import time

from test_rawimu_live import ROOT, live
from test_network_source import wait_for


class CommandViewTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.app = live.App(self.root, dict(live.DEFAULT_CFG, renderer='canvas'), autostart=False)
        self.root.update_idletasks()

    def tearDown(self):
        if self.app.worker:
            self.app.worker.stop()
            if self.app.worker.ident:
                self.app.worker.join(2)
        self.app.on_close()

    def udp(self):
        with socket.socket(type=socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        self.app.source_var.set('UDP 接收')
        self.app.on_source_changed()
        self.app.host_var.set('127.0.0.1')
        self.app.network_port_var.set(str(port))
        self.app.toggle_port()
        self.assertTrue(self.app.worker.ready.wait(2))
        return self.app.command_view

    def test_tab_is_present_and_disconnected_or_replay_cannot_send(self):
        view = self.app.command_view
        self.assertIn('命令发送', [self.app.notebook.tab(tab, 'text') for tab in self.app.notebook.tabs()])
        view.input.insert('1.0', 'PING')
        self.assertIsNone(view.send())
        self.assertIn('连接', view.status_var.get())
        self.app.worker = live.ReplayWorker(self.app.sess, str(ROOT/'examples/rawimu_test.dat'))
        view.refresh()
        self.assertEqual(str(view.send_button.cget('state')), 'disabled')
        self.assertIsNone(view.send())
        self.app.worker = None

    def test_udp_text_hex_history_and_device_response(self):
        view = self.udp()
        with socket.socket(type=socket.SOCK_DGRAM) as peer:
            peer.bind(('127.0.0.1', 0))
            peer.settimeout(2)
            view.target_var.set('custom')
            view.target_host_var.set('127.0.0.1')
            view.target_port_var.set(str(peer.getsockname()[1]))
            view.input.insert('1.0', 'PING')
            self.assertIsNotNone(view.send())
            data, source = peer.recvfrom(100)
            self.assertEqual(data, b'PING\r\n')
            peer.sendto(b'<OK\r\n', source)
            self.assertTrue(wait_for(lambda: len(self.app.worker.commands.responses) > 0))
            view.refresh()
            log = view.log.get('1.0', 'end')
            self.assertIn('TX', log)
            self.assertIn('<OK', log)
            view.mode_var.set('HEX 字节')
            view.input.delete('1.0', 'end')
            view.input.insert('1.0', 'AA 00 FF')
            self.assertIsNotNone(view.send())
            self.assertEqual(peer.recvfrom(100)[0], b'\xaa\x00\xff')
            self.assertEqual(len(view.history), 2)
            view.load_history(1)
            self.assertEqual(view.input.get('1.0', 'end-1c'), 'PING')
            self.assertEqual(view.mode_var.get(), '文本 ASCII')
            self.assertEqual(len(self.app.worker.commands.pending), 0)

    def test_udp_needs_valid_explicit_target_or_known_peer_and_invalid_hex_is_not_queued(self):
        view = self.udp()
        view.input.insert('1.0', 'PING')
        self.assertIsNone(view.send())
        self.assertIn('发送端', view.status_var.get())
        view.target_var.set('custom')
        view.target_host_var.set('0.0.0.0')
        view.target_port_var.set('3001')
        self.assertIsNone(view.send())
        view.target_host_var.set('127.0.0.1')
        view.mode_var.set('HEX 字节')
        self.assertIsNone(view.send())
        self.assertEqual(len(self.app.worker.commands.pending), 0)

    def test_hex_receive_preview_works_with_python37(self):
        view = self.app.command_view
        view._append_events([{'kind': 'rx_hex', 'data': b'\xaa\x00\xff', 'size': 3, 'time': time.time()}])
        self.assertIn('aa 00 ff', view.log.get('1.0', 'end'))

