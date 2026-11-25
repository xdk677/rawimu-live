import socket
import unittest
import threading
from unittest.mock import patch

from test_rawimu_live import live, session, ROOT
from test_network_source import wait_for


class CommandWorkerTests(unittest.TestCase):
    def setUp(self):
        self.s = session()
        self.workers = []

    def tearDown(self):
        for worker in self.workers:
            worker.stop()
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.s.close_analysis()

    def start(self, worker):
        self.workers.append(worker)
        worker.start()
        self.assertTrue(wait_for(lambda: worker.commands.ready))
        return worker

    def test_serial_send_uses_same_handle_and_keeps_receiver_running(self):
        writes, handles = [], []
        class Port:
            in_waiting = 0
            def write(self, data):
                writes.append(data)
                return len(data)
            def close(self):
                pass
        def open_port(*args, **kwargs):
            handles.append(kwargs)
            return Port()
        with patch('serial.Serial', side_effect=open_port):
            worker = self.start(live.SerialWorker(self.s))
            worker.commands.submit(b'LOG VERSIONA ONCE\r\n')
            self.assertTrue(wait_for(lambda: bool(writes)))
            self.assertEqual(writes, [b'LOG VERSIONA ONCE\r\n'])
            self.assertEqual(len(handles), 1)
            self.assertLessEqual(handles[0]['write_timeout'], .5)
            self.assertEqual(self.s.state, 'serial')

    def test_tcp_client_duplex_command_and_response(self):
        from network_source import NetworkWorker
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen(1)
            worker = self.start(NetworkWorker(self.s, 'tcp_client', *listener.getsockname()))
            with listener.accept()[0] as peer:
                peer.settimeout(2)
                worker.commands.submit(b'PING\r\n')
                self.assertEqual(peer.recv(100), b'PING\r\n')
                peer.sendall(b'<OK\r\n')
                self.assertTrue(wait_for(lambda: len(worker.commands.responses) > 0))
                events = worker.commands.poll()
                self.assertEqual([e['kind'] for e in events], ['tx', 'rx'])

    def test_tcp_server_requires_peer_then_sends_to_current_peer(self):
        from network_source import NetworkWorker
        worker = NetworkWorker(self.s, 'tcp_server', '127.0.0.1', 0)
        self.workers.append(worker)
        worker.start()
        self.assertTrue(worker.ready.wait(2))
        self.assertFalse(worker.commands.ready)
        with socket.create_connection(worker.bound_address, timeout=2) as peer:
            self.assertTrue(wait_for(lambda: worker.commands.ready))
            worker.commands.submit(b'PING')
            self.assertEqual(peer.recv(100), b'PING')
        self.assertTrue(wait_for(lambda: self.s.state == 'listening'))
        self.assertFalse(worker.commands.ready)

    def test_udp_can_send_to_explicit_target_before_receiving_then_reply_to_pinned_peer(self):
        from network_source import NetworkWorker
        worker = self.start(NetworkWorker(self.s, 'udp', '127.0.0.1', 0))
        with socket.socket(type=socket.SOCK_DGRAM) as peer:
            peer.bind(('127.0.0.1', 0))
            peer.settimeout(2)
            worker.commands.submit(b'PING', peer.getsockname())
            data, source = peer.recvfrom(100)
            self.assertEqual(data, b'PING')
            self.assertEqual(source, worker.bound_address)
            peer.sendto(b'<OK\r\n', source)
            self.assertTrue(wait_for(lambda: len(worker.commands.responses) > 0))
            self.assertIsNone(worker.peer)
            peer.sendto((ROOT/'examples/rawimu_test.dat').read_bytes()[:72], source)
            self.assertTrue(wait_for(lambda: worker.peer is not None))
            worker.commands.submit(b'NEXT', worker.peer)
            self.assertEqual(peer.recvfrom(100)[0], b'NEXT')

    def test_explicit_udp_target_response_is_visible_without_entering_primary_telemetry(self):
        from network_source import NetworkWorker
        worker = self.start(NetworkWorker(self.s, 'udp', '127.0.0.1', 0))
        with socket.socket(type=socket.SOCK_DGRAM) as primary, socket.socket(type=socket.SOCK_DGRAM) as target:
            primary.bind(('127.0.0.1', 0))
            primary.sendto(b'PRIMARY\r\n', worker.bound_address)
            self.assertTrue(wait_for(lambda: worker.peer is not None))
            worker.commands.poll()
            target.bind(('127.0.0.1', 0))
            target.settimeout(2)
            worker.commands.submit(b'PING', target.getsockname())
            target.recvfrom(100)
            before = self.s.parser.n_bytes
            target.sendto(b'<OK\r\n', worker.bound_address)
            self.assertTrue(wait_for(lambda: len(worker.commands.responses) > 0))
            response = [e for e in worker.commands.poll() if e['kind'] == 'rx'][0]
            self.assertEqual(response['data'], b'<OK')
            self.assertEqual(response['source'], target.getsockname())
            self.assertEqual(self.s.parser.n_bytes, before)
            self.assertEqual(worker.peer, primary.getsockname())

    def test_slow_serial_write_does_not_block_receiver(self):
        entered, release, read = threading.Event(), threading.Event(), threading.Event()
        class Port:
            @property
            def in_waiting(self):
                return 72 if entered.is_set() and not read.is_set() else 0
            def read(self, size):
                from test_rawimu_live import ROOT
                read.set()
                return (ROOT/'examples/rawimu_test.dat').read_bytes()[:72]
            def write(self, data):
                entered.set()
                release.wait(2)
                return len(data)
            def close(self):
                release.set()
        with patch('serial.Serial', return_value=Port()):
            worker = self.start(live.SerialWorker(self.s))
            try:
                worker.commands.submit(b'PING')
                self.assertTrue(entered.wait(1))
                self.assertTrue(read.wait(.4), 'RX was blocked behind a slow TX')
                self.assertTrue(wait_for(lambda: self.s.n_imu == 1, timeout=.4))
            finally:
                release.set()

    def test_command_target_reply_before_telemetry_does_not_claim_primary_peer(self):
        from network_source import NetworkWorker
        worker = self.start(NetworkWorker(self.s, 'udp', '127.0.0.1', 0))
        with socket.socket(type=socket.SOCK_DGRAM) as target, socket.socket(type=socket.SOCK_DGRAM) as primary:
            target.bind(('127.0.0.1', 0))
            target.settimeout(2)
            worker.commands.submit(b'PING', target.getsockname())
            target.recvfrom(100)
            target.sendto(b'<OK\r\n', worker.bound_address)
            self.assertTrue(wait_for(lambda: len(worker.commands.responses) > 0))
            self.assertIsNone(worker.peer)
            primary.bind(('127.0.0.1', 0))
            primary.sendto((ROOT/'examples/rawimu_test.dat').read_bytes()[:72], worker.bound_address)
            self.assertTrue(wait_for(lambda: self.s.n_imu == 1))
            self.assertEqual(worker.peer, primary.getsockname())
            self.assertEqual(self.s.parser.n_bytes, 72)

