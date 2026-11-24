"""Bounded manual command queues. Only acquisition workers perform I/O."""
from collections import deque
import threading
import time

MAX_COMMAND_BYTES = 65536
ENDINGS = {'CRLF': b'\r\n', 'CR': b'\r', 'LF': b'\n', '无': b''}


class _ResponseText(object):
    """Skip framed NovAtel binary payloads before inspecting adjacent text."""
    def __init__(self):
        self.buffer = bytearray()
        self.skip = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        buf = self.buffer
        lines = []
        while buf:
            if self.skip:
                count = min(self.skip, len(buf))
                del buf[:count]
                self.skip -= count
                continue
            positions = [v for v in (buf.find(b'\r'), buf.find(b'\n')) if v >= 0]
            end = min(positions) if positions else len(buf)
            sync = buf.find(b'\xaa\x44')
            if 0 <= sync < end:
                del buf[:sync]
                if len(buf) < 3:
                    break
                if buf[2] == 0x12:
                    if len(buf) < 10:
                        break
                    self.skip = 28 + (buf[8] | buf[9] << 8) + 4
                elif buf[2] in (0x10, 0x11, 0x13):
                    if len(buf) < 4:
                        break
                    self.skip = 12 + buf[3] + 4
                else:
                    del buf[:1]
                continue
            if not positions:
                if len(buf) > 8192:
                    del buf[:-1]  # Keep a possible split sync prefix.
                break
            line = bytes(buf[:end])
            del buf[:end+1]
            if (line and len(line) <= 4096 and all(v == 9 or 32 <= v <= 126 for v in line) and
                    not line.startswith((b'#RAWIMU', b'$RAWIMU', b'#GPIMU', b'$GPIMU'))):
                lines.append(line)
        return lines


def encode_command(text, mode='text', ending='CRLF'):
    try:
        if mode == 'hex':
            data = bytes.fromhex(text)
        elif mode == 'text':
            data = text.rstrip('\r\n').encode('ascii') + ENDINGS[ending]
            if not text.rstrip('\r\n').strip():
                raise ValueError('命令不能为空')
        else:
            raise ValueError('未知命令格式')
    except (UnicodeError, KeyError):
        raise ValueError('文本命令仅支持 ASCII；二进制数据请使用 HEX')
    except ValueError as error:
        raise ValueError('HEX 需为成对的十六进制字节，如 AA 00 FF' if mode == 'hex' else str(error))
    if not data or len(data) > MAX_COMMAND_BYTES:
        raise ValueError('命令需为 1～65536 字节')
    return data


class CommandChannel(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.pending = deque()
        self.results = deque()
        self.responses = deque(maxlen=200)
        self.token = None
        self.endpoint = ''
        self.next_id = 0
        self.rx_mode = 'text'
        self._rx = {}
        self._writer_stop = self._writer_wake = None
        self._writers = []

    @property
    def ready(self):
        with self.lock:
            return self.token is not None

    def activate(self, token, endpoint):
        with self.lock:
            if self.token is not token:
                self.deactivate()
            self.token, self.endpoint = token, endpoint
            self._rx.clear()

    def deactivate(self):
        with self.lock:
            self.token = None
            self._rx.clear()
            if self._writer_stop is not None:
                self._writer_stop.set()
                self._writer_wake.set()
            while self.pending:
                request = self.pending.popleft()
                self._result(request, 'error', '连接已断开，命令未发送')

    def submit(self, data, target=None):
        with self.lock:
            if self.token is None:
                raise ValueError('当前连接尚不能发送命令')
            if not data or len(data) > MAX_COMMAND_BYTES:
                raise ValueError('命令需为 1～65536 字节')
            if len(self.pending) >= 32:
                raise ValueError('发送队列已满，请等待已提交命令完成')
            self.next_id += 1
            self.pending.append({'id': self.next_id, 'data': bytes(data), 'target': target,
                                 'token': self.token, 'endpoint': self.endpoint})
            if self._writer_wake is not None:
                self._writer_wake.set()
            return self.next_id

    def start_sender(self, token, sender):
        """One TX thread per connection; the acquisition thread remains free to read."""
        stopped, wake = threading.Event(), threading.Event()
        with self.lock:
            self._writer_stop, self._writer_wake = stopped, wake
        def run():
            while not stopped.is_set():
                wake.wait(.05)
                wake.clear()
                if stopped.is_set():
                    break
                self.drain(token, sender, limit=1)
                with self.lock:
                    if self.pending:
                        wake.set()
        thread = threading.Thread(target=run, name='imu-command-tx', daemon=True)
        with self.lock:
            self._writers = [writer for writer in self._writers if writer.is_alive()]
            self._writers.append(thread)
        thread.start()

    def wait_senders(self, timeout=.5):
        with self.lock:
            writers = list(self._writers)
        for writer in writers:
            writer.join(timeout)

    def _result(self, request, kind, message):
        self.results.append({'kind': kind, 'id': request['id'], 'data': request['data'],
                             'endpoint': request['endpoint'], 'target': request['target'],
                             'message': message, 'time': time.time()})

    def drain(self, token, sender, limit=1):
        for _ in range(limit):
            with self.lock:
                if not self.pending:
                    return
                request = self.pending.popleft()
                if request['token'] is not token or self.token is not token:
                    self._result(request, 'error', '连接已变化，命令未发送')
                    continue
            try:
                count = sender(request['data'], request['target'])
                if count is not None and count != len(request['data']):
                    raise OSError('仅写入 %s/%d 字节' % (count, len(request['data'])))
            except Exception as error:
                kind, message = 'error', '发送失败（可能已部分发送，不自动重发）：%s' % error
            else:
                kind, message = 'tx', '已写入连接 %d B（设备是否执行请查看应答）' % len(request['data'])
            with self.lock:
                self._result(request, kind, message)

    def observe(self, chunk, source=None):
        if not chunk:
            return
        with self.lock:
            if self.rx_mode == 'hex':
                self._rx.clear()
                self.responses.append({'kind': 'rx_hex', 'data': bytes(chunk[:256]),
                                       'size': len(chunk), 'time': time.time(), 'source': source})
                return
            if self.rx_mode != 'text':
                self._rx.clear()
                return
            if source not in self._rx:
                if len(self._rx) >= 32:
                    self._rx.pop(next(iter(self._rx)))
                self._rx[source] = _ResponseText()
            for line in self._rx[source].feed(chunk):
                self.responses.append({'kind': 'rx', 'data': line, 'size': len(line),
                                       'time': time.time(), 'source': source})

    def poll(self):
        with self.lock:
            events = list(self.results) + list(self.responses)
            self.results.clear()
            self.responses.clear()
        return sorted(events, key=lambda event: event['time'])
