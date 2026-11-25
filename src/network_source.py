"""Receive an unmodified byte stream over TCP or UDP without touching Tk."""
import ipaddress
import socket
import threading
import time
from command_transport import CommandChannel


NETWORK_MODES = {'TCP 客户端': 'tcp_client', 'TCP 服务端': 'tcp_server', 'UDP 接收': 'udp'}


def validate_endpoint(mode, host, port):
    if mode not in NETWORK_MODES.values():
        raise ValueError('请选择 TCP 客户端、TCP 服务端或 UDP 接收')
    host = str(host).strip()
    try:
        address = ipaddress.IPv4Address(host)
        number = int(str(port).strip())
    except (ValueError, TypeError):
        raise ValueError('请输入有效 IPv4 地址和整数端口')
    if not 1 <= number <= 65535:
        raise ValueError('网口端口必须在 1～65535 之间')
    if mode == 'tcp_client' and (address.is_unspecified or address.is_multicast or host == '255.255.255.255'):
        raise ValueError('TCP 客户端需要设备的 IPv4 地址')
    return mode, host, number


class NetworkWorker(threading.Thread):
    def __init__(self, sess, mode, host, port):
        threading.Thread.__init__(self, name='imu-' + mode, daemon=True)
        self.s, self.mode, self.host, self.port = sess, mode, host, int(port)
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.bound_address = None
        self.peer = None
        self.ignored_datagrams = 0
        self._lock = threading.Lock()
        self._sockets = set()
        self.commands = CommandChannel()
        self._command_peers = {}

    def _track(self, sock):
        with self._lock:
            if self.stop_event.is_set():
                sock.close()
                raise InterruptedError('connection stopped')
            self._sockets.add(sock)
        sock.settimeout(.1)
        return sock

    def _close(self, sock):
        with self._lock:
            self._sockets.discard(sock)
        try:
            sock.close()
        except OSError:
            pass

    def stop(self):
        self.stop_event.set()
        self.s.running = False
        self.commands.deactivate()
        with self._lock:
            sockets = list(self._sockets)
        for sock in sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._close(sock)

    def _receive_tcp(self, sock):
        while not self.stop_event.is_set():
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                return
            self.commands.observe(chunk)
            self.s.feed(chunk)

    def _listen_socket(self, sock):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        self.bound_address = sock.getsockname()

    def _tcp_server(self, listener):
        self._listen_socket(listener)
        listener.listen(1)
        self.s.state = 'listening'
        self.s.status = 'TCP 服务端监听 %s:%d，等待设备连接' % self.bound_address
        self.ready.set()
        while not self.stop_event.is_set():
            try:
                peer, address = listener.accept()
            except socket.timeout:
                continue
            peer = self._track(peer)
            self.peer = address
            self.s.state = 'network'
            self.s.status = 'TCP 服务端 %s:%d ← %s:%d' % (self.bound_address + address)
            self.commands.activate(peer, self.s.status)
            self.commands.start_sender(peer, lambda data, target, connection=peer: connection.sendall(data))
            try:
                self._receive_tcp(peer)
            except (ConnectionResetError, ConnectionAbortedError):
                if self.stop_event.is_set():
                    return
            finally:
                self.commands.deactivate()
                self._close(peer)
                self.commands.wait_senders()
            # A new connection must not complete a previous peer's partial frame.
            self.s.parser.n_drop += len(self.s.parser.buf)
            self.s.parser.buf.clear()
            self.peer = None
            self.s.state = 'listening'
            self.s.status = 'TCP 服务端监听 %s:%d，设备已断开，等待连接' % self.bound_address

    def _send_udp(self, sock, data, target):
        now = time.monotonic()
        with self._lock:
            self._command_peers = {peer: expiry for peer, expiry in self._command_peers.items() if expiry > now}
            if target is not None:
                self._command_peers[target] = now+30.
            while len(self._command_peers) > 32:
                self._command_peers.pop(next(iter(self._command_peers)))
        return sock.sendto(data, target)

    def _is_telemetry(self, chunk):
        """An explicit command reply must not claim the first telemetry peer."""
        if chunk.startswith(b'\xaa'):
            if len(chunk) < 6:
                return True
            return (chunk[4] | (chunk[5] << 8)) in self.s.formats.by_id
        stripped = chunk.lstrip(b'\r\n')
        prefixes = [prefix.encode('ascii') for prefix in self.s.formats.by_prefix]
        prefixes.append(b'$GPIMU,')
        return any(stripped.startswith(prefix) or (stripped and prefix.startswith(stripped)) for prefix in prefixes)

    def _udp(self, sock):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
        self._listen_socket(sock)
        self.s.state = 'listening'
        self.s.status = 'UDP 监听 %s:%d，等待数据（锁定首个发送端）' % self.bound_address
        self.commands.activate(sock, 'UDP %s:%d' % self.bound_address)
        self.commands.start_sender(sock, lambda data, target: self._send_udp(sock, data, target))
        self.ready.set()
        while not self.stop_event.is_set():
            try:
                chunk, address = sock.recvfrom(65536)
            except socket.timeout:
                continue
            if not chunk:
                continue
            with self._lock:
                expected_reply = self._command_peers.get(address, 0.) > time.monotonic()
            if expected_reply and address != self.peer and (self.peer is not None or not self._is_telemetry(chunk)):
                self.commands.observe(chunk, address)
                continue
            if self.peer is None:
                self.peer = address
                self.s.state = 'network'
                self.s.status = 'UDP %s:%d ← %s:%d（已锁定发送端）' % (self.bound_address + address)
            if address != self.peer:
                self.ignored_datagrams += 1
                continue
            self.commands.observe(chunk, address)
            self.s.feed(chunk)

    def run(self):
        sess = self.s
        try:
            if self.stop_event.is_set():
                return
            sess.error, sess.running, sess.state = '', True, 'connecting'
            sock = self._track(socket.socket(socket.AF_INET, socket.SOCK_DGRAM if self.mode == 'udp' else socket.SOCK_STREAM))
            if self.mode == 'tcp_client':
                sock.settimeout(3.)
                sock.connect((self.host, self.port))
                sock.settimeout(.1)
                self.bound_address = sock.getsockname()
                self.peer = (self.host, self.port)
                sess.state = 'network'
                sess.status = 'TCP 客户端已连接 %s:%d' % self.peer
                self.commands.activate(sock, sess.status)
                self.commands.start_sender(sock, lambda data, target, connection=sock: connection.sendall(data))
                self.ready.set()
                self._receive_tcp(sock)
                if not self.stop_event.is_set():
                    sess.status = 'TCP 远端已关闭连接 %s:%d' % self.peer
            elif self.mode == 'tcp_server':
                self._tcp_server(sock)
            elif self.mode == 'udp':
                self._udp(sock)
            else:
                raise ValueError('未知网口模式')
        except (OSError, ValueError) as error:
            if not self.stop_event.is_set():
                sess.error = '网口连接 / 接收失败：%s' % error
        finally:
            self.commands.deactivate()
            with self._lock:
                sockets = list(self._sockets)
            for sock in sockets:
                self._close(sock)
            self.commands.wait_senders()
            with sess.raw_lock:
                sess.running = False
                sess.state = 'error' if sess.error else 'disconnected'
                sess.stop_raw_record()
            self.ready.set()
