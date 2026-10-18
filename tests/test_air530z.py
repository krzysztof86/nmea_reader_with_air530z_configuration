import os
from pathlib import Path
import queue
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import nmea_reader as app


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.baudrate = 9600
        self.writes = []
        self.flushes = 0

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        self.flushes += 1


class ConfiguratorTests(unittest.TestCase):
    def setUp(self):
        self.ser = FakeSerial()
        self.config = app.Air530ZConfigurator(self.ser)

    def test_documented_checksums(self):
        for body, expected in (
            ('PCAS01,5', '19'), ('PCAS02,1000', '2E'),
            ('PCAS10,0', '1C'), ('PCAS10,2', '1E'),
            ('PCAS06,0', '1B'), ('PCAS04,3', '1A'),
            ('PCAS15,2,FFFFFFE0', '42'),
            ('PCAS03,1,0,0,1,1,0,0,0,0,0,0,0,0', '1F'),
        ):
            with self.subTest(body=body):
                self.assertEqual(app.calculate_checksum(body), expected)

    def test_all_commands_and_flush(self):
        with patch.object(app.time, 'sleep') as sleep:
            self.assertEqual(self.config.set_baudrate(115200), '$PCAS01,5*19')
            sleep.assert_called_once_with(0.2)
        self.assertEqual(self.ser.baudrate, 115200)
        self.config.set_system_mode(7)
        self.config.set_update_rate(100)
        self.config.set_nmea_sentences([1, 0, 0, 1, 1, 0, 0, 0])
        self.config.restart(0)
        self.config.query_firmware()
        self.config.set_satellite_mask(2, 'ffffffe0')
        self.assertEqual(self.ser.writes, [
            b'$PCAS01,5*19\r\n', b'$PCAS04,7*1E\r\n',
            b'$PCAS02,100*1E\r\n',
            b'$PCAS03,1,0,0,1,1,0,0,0,0,0,0,0,0*1F\r\n',
            b'$PCAS10,0*1C\r\n', b'$PCAS06,0*1B\r\n',
            b'$PCAS15,2,FFFFFFE0*42\r\n'])
        self.assertEqual(self.ser.flushes, 7)

    def test_baud_is_changed_after_transmission_and_delay(self):
        def delay(seconds):
            self.assertEqual(self.ser.writes, [b'$PCAS01,0*1C\r\n'])
            self.assertEqual(self.ser.flushes, 1)
            self.assertEqual(self.ser.baudrate, 115200)
        self.ser.baudrate = 115200
        with patch.object(app.time, 'sleep', side_effect=delay):
            self.config.set_baudrate(9600)
        self.assertEqual(self.ser.baudrate, 9600)

    def test_validation_does_not_write(self):
        for method, args in (
            ('set_baudrate', (4800,)), ('set_system_mode', (0,)),
            ('set_update_rate', (0,)), ('restart', (4,)),
            ('set_nmea_sentences', ([1] * 13,)),
            ('set_nmea_sentences', ([2] * 8,)),
            ('set_satellite_mask', (3, '00000001')),
            ('set_satellite_mask', (1, '0000000G')),
            ('set_satellite_mask', (1, '00000001\r\n')),
        ):
            with self.subTest(method=method, args=args), self.assertRaises(ValueError):
                getattr(self.config, method)(*args)
        self.assertEqual(self.ser.writes, [])

    def test_closed_port_and_write_failure(self):
        self.ser.is_open = False
        with self.assertRaises(app.serial.SerialException):
            self.config.query_firmware()
        self.ser.is_open = True
        with patch.object(self.ser, 'write', return_value=1):
            with self.assertRaises(app.serial.SerialException):
                self.config.restart(0)
        with patch.object(self.ser, 'flush', side_effect=OSError('disconnected')):
            with self.assertRaises(OSError):
                self.config.query_firmware()


@unittest.skipUnless(os.name == 'posix', 'requires POSIX pseudo terminal')
class CaptureIntegrationTests(unittest.TestCase):
    def setUp(self):
        import pty
        self.master, self.slave = pty.openpty()
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'capture.nmea'
        self.capture = app.Capture(os.ttyname(self.slave), 9600, self.path)
        self.capture.start()
        self.event('opened')
        self.expected = b''

    def tearDown(self):
        self.capture.stop()
        self.capture.thread.join(3)
        os.close(self.master)
        os.close(self.slave)
        self.temp.cleanup()
        self.assertFalse(self.capture.thread.is_alive())

    def event(self, kind, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                event, value = self.capture.events.get(timeout=0.1)
            except queue.Empty:
                continue
            if event == kind:
                return value
            if event == 'error':
                self.fail(value)
        self.fail('Missing event: ' + kind)

    def command(self, method, args, expected):
        self.capture.configure(method, *args)
        data = b''
        deadline = time.monotonic() + 3
        while not data.endswith(b'\r\n') and time.monotonic() < deadline:
            if select.select([self.master], [], [], 0.1)[0]:
                data += os.read(self.master, 4096)
        self.assertEqual(data, expected)
        self.assertEqual(self.event('configured')[0].encode() + b'\r\n', expected)

    def receive(self, payload):
        os.write(self.master, payload)
        received = b''
        while len(received) < len(payload):
            received += self.event('data')
        self.assertEqual(received, payload)
        self.expected += payload
        self.assertEqual(self.path.read_bytes(), self.expected)

    def test_receive_after_baud_and_restart_without_ack(self):
        self.receive(b'partial\xff\x00\r\n$GNGGA,bad*00\r\n')
        self.command('set_baudrate', (115200,), b'$PCAS01,5*19\r\n')
        self.assertEqual(self.capture.baud, 115200)
        import termios
        self.assertEqual(termios.tcgetattr(self.slave)[4], termios.B115200)
        self.receive(b'$GNRMC,after-baud\r\n')
        self.command('restart', (0,), b'$PCAS10,0*1C\r\n')
        # No response or ACK; the reader survives a quiet period.
        time.sleep(0.3)
        self.receive(b'$GNGGA,after-restart\r\nlast-partial')
        self.assertTrue(self.capture.thread.is_alive())

    def test_firmware_split_chunks_still_logged_verbatim(self):
        self.command('query_firmware', (), b'$PCAS06,0*1B\r\n')
        self.receive(b'$GPRMC,ordinary\r\n$GPT')
        self.receive(b'XT,01,01,02,FW 1.0*00\r\n')
        # data is enqueued after firmware, so inspect timeout separately below.
        self.assertTrue(self.capture.firmware_found)
        self.capture.firmware_deadline = time.monotonic() - 1
        self.receive(b'$GNRMC,continues\r\n')

    def test_invalid_command_is_nonfatal(self):
        self.capture.configure('set_satellite_mask', 1, 'invalid')
        self.assertIn('8', self.event('configuration_error'))
        self.receive(b'$GNGGA,still-receiving\r\n')

    def test_write_error_does_not_stop_reception(self):
        with patch.object(app.Air530ZConfigurator, '_send',
                          side_effect=app.serial.SerialException('write failed')):
            self.capture.configure('restart', 0)
            self.assertEqual(self.event('configuration_error'), 'write failed')
        self.receive(b'$GNRMC,after-write-error\r\n')

    def test_baud_failure_closes_port_and_log(self):
        with patch.object(app.Air530ZConfigurator, 'set_baudrate',
                          side_effect=app.serial.SerialException('baud failed')):
            self.capture.configure('set_baudrate', 115200)
            self.assertEqual(self.event('configuration_error'), 'baud failed')
            closed = None
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                kind, value = self.capture.events.get(timeout=1)
                if kind == 'closed':
                    closed = value
                    break
            self.assertIsNotNone(closed)
            self.assertTrue(closed['failed'])
        self.capture.thread.join(2)
        self.assertFalse(self.capture.thread.is_alive())

    def test_existing_log_is_not_overwritten(self):
        other = app.Capture(os.ttyname(self.slave), 9600, self.path)
        original = self.path.read_bytes()
        other.start()
        other.thread.join(2)
        self.assertFalse(other.thread.is_alive())
        events = []
        while not other.events.empty():
            events.append(other.events.get_nowait())
        self.assertTrue(any(k == 'error' for k, _ in events))
        self.assertEqual(self.path.read_bytes(), original)

    def test_console_preserves_stdout_and_log(self):
        output = Path(self.temp.name) / 'console.nmea'
        self.capture.stop()
        self.capture.thread.join(2)
        process = subprocess.Popen(
            [sys.executable, str(Path(app.__file__)), '--console', '--port',
             os.ttyname(self.slave), '--output', str(output)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            # Wait for console's log to open before transmitting.
            deadline = time.monotonic() + 3
            while not output.exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(output.exists())
            payload = b'\xff\x00partial\r\n$GPGGA,bad*00\r\nlast'
            os.write(self.master, payload)
            deadline = time.monotonic() + 3
            while output.read_bytes() != payload and time.monotonic() < deadline:
                time.sleep(0.01)
            process.send_signal(signal.SIGINT)
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertEqual(stdout, payload)
            self.assertEqual(output.read_bytes(), payload)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()


class FirmwareTests(unittest.TestCase):
    def test_observer_reports_split_response_and_timeout(self):
        capture = app.Capture('unused', 9600, Path('unused'))
        capture.firmware_deadline = time.monotonic() + 5
        capture._observe_firmware(b'$GPT')
        capture._observe_firmware(b'XT,FW 1.0\r\n$GNRMC,ignored\r\n')
        self.assertEqual(capture.events.get_nowait(), ('firmware', '$GPTXT,FW 1.0'))
        self.assertTrue(capture.events.empty())
        capture.firmware_found = False
        capture.firmware_deadline = time.monotonic() - 1
        capture._observe_firmware(b'')
        self.assertIn('5 s', capture.events.get_nowait()[1])
        self.assertIsNone(capture.firmware_deadline)


if __name__ == '__main__':
    unittest.main()
