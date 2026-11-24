import sys
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path

from test_rawimu_live import ROOT, session


def wait_for(predicate, timeout=3.):
    limit = time.monotonic() + timeout
    while time.monotonic() < limit:
        if predicate():
            return True
        threading.Event().wait(.01)
    return False


class NetworkSourceTests(unittest.TestCase):
    def setUp(self):
        from network_source import NetworkWorker
        self.worker_class = NetworkWorker
        self.s = session()
        self.workers = []
        self.data = (ROOT / 'examples/rawimu_test.dat').read_bytes()[:720]

    def tearDown(self):
        for worker in self.workers:
            worker.stop()
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.s.stop_raw_record()
        self.s.close_analysis()

    def start(self, mode, port=0):
        worker = self.worker_class(self.s, mode, '127.0.0.1', port)
        self.workers.append(worker)
        worker.start()
        self.assertTrue(worker.ready.wait(2), self.s.error)
        return worker

    def test_tcp_client_fragmented_frames_and_exact_raw_dat(self):
        with socket.socket() as server, tempfile.TemporaryDirectory() as folder:
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            path = Path(folder)/'network.dat'
            self.assertTrue(self.s.start_raw_record(str(path)))
            worker = self.start('tcp_client', server.getsockname()[1])
            with server.accept()[0] as peer:
                peer.sendall(self.data[:17])
                peer.sendall(self.data[17:])
                self.assertTrue(wait_for(lambda: self.s.n_imu == 10))
            worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(path.read_bytes(), self.data)
            self.assertEqual(self.s.state, 'disconnected')

    def test_tcp_server_reaccepts_and_drops_partial_old_peer_frame(self):
        worker = self.start('tcp_server')
        self.assertEqual(self.s.state, 'listening')
        address = worker.bound_address
        with socket.create_connection(address, timeout=1) as first:
            first.sendall(self.data[:20])
            self.assertTrue(wait_for(lambda: self.s.parser.n_bytes == 20))
        self.assertTrue(wait_for(lambda: self.s.state == 'listening'))
        with socket.create_connection(address, timeout=1) as second:
            second.sendall(self.data)
            self.assertTrue(wait_for(lambda: self.s.n_imu == 10))
        self.assertEqual(self.s.error, '')

    def test_udp_reassembles_fragments_and_pins_one_sender(self):
        worker = self.start('udp')
        with socket.socket(type=socket.SOCK_DGRAM) as sender, socket.socket(type=socket.SOCK_DGRAM) as other:
            sender.sendto(self.data[:19], worker.bound_address)
            self.assertTrue(wait_for(lambda: self.s.parser.n_bytes == 19))
            other.sendto(self.data, worker.bound_address)
            sender.sendto(self.data[19:], worker.bound_address)
            self.assertTrue(wait_for(lambda: self.s.n_imu == 10))
            self.assertEqual(worker.ignored_datagrams, 1)
            self.assertEqual(self.s.parser.n_bytes, len(self.data))

    def test_udp_receives_full_burst_without_small_default_socket_buffer_loss(self):
        worker = self.start('udp')
        data = (ROOT/'examples/rawimu_test.dat').read_bytes()
        payload_size = 8192 if sys.platform.startswith('darwin') else 18000
        with socket.socket(type=socket.SOCK_DGRAM) as sender:
            for first in range(0, len(data), payload_size):
                sender.sendto(data[first:first+payload_size], worker.bound_address)
        self.assertTrue(wait_for(lambda: self.s.n_imu == 2000), str(self.s.n_imu))
        self.assertEqual(self.s.parser.n_bytes, len(data))

    def test_stop_idle_listener_and_udp_closes_dat(self):
        for mode in ('tcp_server', 'udp'):
            worker = self.start(mode)
            worker.stop()
            worker.join(1)
            self.assertFalse(worker.is_alive())
            self.assertEqual(self.s.error, '')
            self.assertFalse(self.s.raw_recording)

    def test_server_can_restart_on_same_port_after_stopping_connected_peer(self):
        worker = self.start('tcp_server')
        address = worker.bound_address
        with socket.create_connection(address, timeout=1) as peer:
            self.assertTrue(wait_for(lambda: self.s.state == 'network'))
            worker.stop()
            worker.join(1)
        replacement = self.start('tcp_server', address[1])
        self.assertEqual(self.s.state, 'listening', self.s.error)
        self.assertEqual(replacement.bound_address, address)

    def test_endpoint_validation_and_connection_failure(self):
        from network_source import validate_endpoint
        for mode, host, port in [('tcp_client', '0.0.0.0', 3001), ('udp', 'bad', 3),
                                 ('udp', '127.0.0.1', 0), ('tcp_server', '127.0.0.1', 65536)]:
            with self.assertRaises(ValueError):
                validate_endpoint(mode, host, port)
        self.assertEqual(validate_endpoint('udp', '0.0.0.0', '3001'), ('udp', '0.0.0.0', 3001))
        with socket.socket() as unavailable:
            unavailable.bind(('127.0.0.1', 0))
            worker = self.worker_class(self.s, 'tcp_client', '127.0.0.1', unavailable.getsockname()[1])
            self.workers.append(worker)
            worker.start()
            worker.join(4)
            self.assertFalse(worker.is_alive())
            self.assertEqual(self.s.state, 'error')
            self.assertTrue(self.s.error)
