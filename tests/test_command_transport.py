import unittest

from test_rawimu_live import live, ROOT


class CommandTransportTests(unittest.TestCase):
    def test_text_hex_and_terminators_are_exact(self):
        from command_transport import encode_command
        self.assertEqual(encode_command('LOG VERSIONA ONCE', 'text', 'CRLF'), b'LOG VERSIONA ONCE\r\n')
        self.assertEqual(encode_command('ABC\r\n', 'text', 'CR'), b'ABC\r')
        self.assertEqual(encode_command('AA 00 ff\n12', 'hex', 'CRLF'), b'\xaa\x00\xff\x12')
        self.assertEqual(encode_command('ABC', 'text', '无'), b'ABC')
        for value, mode in [('', 'text'), ('XYZ', 'hex'), ('0', 'hex'), ('中文', 'text')]:
            with self.assertRaises(ValueError):
                encode_command(value, mode, 'CRLF')

    def test_disconnected_queue_is_rejected_and_old_commands_cannot_cross_connections(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        with self.assertRaises(ValueError):
            channel.submit(b'ABC')
        first, second = object(), object()
        channel.activate(first, 'first')
        channel.submit(b'old')
        channel.deactivate()
        channel.activate(second, 'second')
        sent = []
        channel.drain(second, lambda payload, target: sent.append(payload) or len(payload))
        self.assertEqual(sent, [])
        self.assertIn('断开', channel.poll()[0]['message'])

    def test_completion_occurs_only_after_worker_write_and_partial_write_is_failure(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        token = object()
        channel.activate(token, 'test')
        channel.submit(b'ABC')
        self.assertEqual(channel.poll(), [])
        channel.drain(token, lambda payload, target: 2)
        self.assertEqual(channel.poll()[0]['kind'], 'error')
        channel.submit(b'ABC')
        channel.drain(token, lambda payload, target: len(payload))
        self.assertEqual(channel.poll()[0]['kind'], 'tx')

    def test_response_observer_keeps_fragmented_lines_and_filters_binary_imu(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        channel.observe(b'<O')
        self.assertEqual(channel.poll(), [])
        channel.observe(b'K\r\n#RAWIMUA,noise\r\n\xaa\x00\x01\r\n')
        events = channel.poll()
        self.assertEqual([event['data'] for event in events], [b'<OK'])
        channel.rx_mode = 'hex'
        channel.observe(bytes(range(256))*4)
        events = channel.poll()
        self.assertEqual(len(events[-1]['data']), 256)
        self.assertEqual(events[-1]['size'], 1024)

    def test_cr_only_responses_are_displayed_without_waiting_for_lf(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        channel.observe(b'OK\rREADY\r')
        self.assertEqual([event['data'] for event in channel.poll()], [b'OK', b'READY'])

    def test_command_backlog_is_bounded_and_cancelled_on_disconnect(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        channel.activate(object(), 'test')
        for _ in range(32):
            channel.submit(b'PING')
        with self.assertRaises(ValueError):
            channel.submit(b'OVERFLOW')
        channel.deactivate()
        self.assertEqual(len(channel.poll()), 32)
        self.assertEqual(len(channel.pending), 0)

    def test_binary_frame_does_not_swallow_immediately_following_fragmented_response(self):
        from command_transport import CommandChannel
        channel = CommandChannel()
        frame = (ROOT/'examples/rawimu_test.dat').read_bytes()[:72]
        channel.observe(frame[:21])
        channel.observe(frame[21:]+b'<O')
        self.assertEqual(channel.poll(), [])
        channel.observe(b'K\r\n')
        self.assertEqual([e['data'] for e in channel.poll()], [b'<OK'])

