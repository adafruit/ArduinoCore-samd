#!/usr/bin/env python3
"""The HIL profile: agentrc's shared core battery under this profile against a simulated DUT, the rosters, and
the profile bound to agentrc's arduino_hil.py. The simulation proves the host's parsing and verdicts only; what the
firmware does is a farm run's evidence."""
import json
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import hil_test as ht  # noqa: E402

E = ht.engine()
BOARD = {'name': 'metro_m4_express'}


class FakeClock:
    """arduino_hil's `time`: the host clock moves only on sleep() and on the FakeDut's port I/O, so every exchange takes
    the same host time whatever the machine's load."""

    def __init__(self):
        self.us = 0

    def monotonic(self):
        return self.us / 1e6

    def sleep(self, seconds):
        self.us += round(seconds * 1e6)


class FakeDut:
    """core_cdc's and core_timing's protocol (test/hil/core_cdc/core_cdc.ino, test/hil/core_timing/core_timing.ino)
    over a byte stream, each write and read taking IO_US of the host's clock, with a clock running `rate` times the
    host's."""
    IO_US = 1000

    def __init__(self, clock, stack='arduino', chip='samd51', rate=1.0, delay_us=None, status='PASS'):
        self.clock, self.stack, self.chip, self.rate, self.delay_us = clock, stack, chip, rate, delay_us
        self.status = status   # what every HIL_RESULT reports
        self.out, self.pending = b'', b''
        self.echo = None   # (nonce, bytes left, bytes so far)
        self.closed = False

    def reset_input_buffer(self):
        pass

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return min(len(self.out), 17)   # replies arrive in fragments

    def read(self, n):
        self.clock.us += self.IO_US
        chunk, self.out = self.out[:n], self.out[n:]
        return chunk

    def write(self, data):
        self.clock.us += self.IO_US
        self.pending += data
        while self.pending:
            if self.echo:
                nonce, left, seen = self.echo
                take, self.pending = self.pending[:left], self.pending[left:]
                seen += take
                self.out += take
                left -= len(take)
                self.echo = (nonce, left, seen) if left else None
                if not left:
                    self.out += (f'\r\nHIL_RESULT {nonce} echo {self.status} len={len(seen)} '
                                 f'crc={zlib.crc32(seen):08x}\r\n').encode()
                continue
            if b'\n' not in self.pending:
                return
            line, self.pending = self.pending.split(b'\n', 1)
            self.command(line.decode())

    def command(self, line):
        w = line.split()
        cmd, nonce = w[0], w[1]
        if cmd == 'INFO':
            reply = f'HIL_INFO {nonce} stack={self.stack} chip={self.chip} f_cpu=120000000 crystalless=0'
        elif cmd == 'ECHO':
            self.echo = (nonce, int(w[2]), b'')
            return
        elif cmd == 'TIME':
            us = int(self.clock.us * self.rate)
            reply = f'HIL_RESULT {nonce} time {self.status} ms={us // 1000} us={us}'
        else:   # DELAY
            us = int(w[2]) * 1000 + 400 if self.delay_us is None else self.delay_us
            reply = f'HIL_RESULT {nonce} delay {self.status} ms={w[2]} us={us}'
        self.out += (reply + '\r\n').encode()


class Battery(unittest.TestCase):
    """CORE_TESTS run agentrc's shared core battery with this profile's stacks, chips and constants."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = FakeClock()
        for patch in (mock.patch.object(E, 'time', self.clock),
                      mock.patch.object(E, 'node', return_value='/dev/ttyACM0')):
            patch.start()
            self.addCleanup(patch.stop)

    def dut(self, **kw):
        return FakeDut(self.clock, **kw)

    def run_test(self, test, dut, timeout=5):
        with mock.patch('serial.Serial', return_value=dut):
            result = E.test_func(test)({'dir': '/x'}, 'abcd1234', self.clock.monotonic() + timeout, None,
                                       self.tmp.name, BOARD)
        self.assertTrue(dut.closed)
        self.assertTrue((Path(self.tmp.name) / 'core-abcd1234.log').is_file())
        return result

    def test_cdc_passes_on_its_stack_only(self):
        for stack in ('arduino', 'tinyusb'):
            with self.subTest(stack=stack):
                ok, detail = self.run_test(f'cdc_{stack}', self.dut(stack=stack))
                self.assertTrue(ok, detail)
                self.assertIn(f'echoed {list(ht.ECHO_LENGTHS)} bytes', detail)
        ok, detail = self.run_test('cdc_tinyusb', self.dut(stack='arduino'))
        self.assertFalse(ok)
        self.assertIn("'stack': 'arduino'", detail)

    def test_the_chip_is_the_boards_family(self):
        self.assertEqual(E.core_chip(BOARD), 'samd51')
        ok, _ = self.run_test('cdc_arduino', self.dut(chip='samd21'))
        self.assertFalse(ok)

    def test_timing_passes_at_the_hosts_rate_naming_crystalless(self):
        ok, detail = self.run_test('timing', self.dut())
        self.assertTrue(ok, detail)
        self.assertIn('f_cpu=120000000 crystalless=0: millis', detail)
        ok, detail = self.run_test('timing', self.dut(rate=1.05))
        self.assertFalse(ok)
        self.assertIn('millis()', detail)

    def test_timing_fails_on_a_short_delay(self):
        ok, detail = self.run_test('timing', self.dut(delay_us=900000))
        self.assertFalse(ok)
        self.assertIn('delay(1000) 900.0 ms outside the host bracket', detail)

    def test_a_non_pass_time_reply_fails_naming_it(self):
        ok, detail = self.run_test('timing', self.dut(status='REJECT'))
        self.assertFalse(ok)
        self.assertIn('time: HIL_RESULT abcd1234a time REJECT', detail)

    def test_a_reply_with_another_nonce_times_out(self):
        dut = self.dut()
        dut.command = lambda line: setattr(dut, 'out', dut.out + b'HIL_INFO other stack=arduino chip=samd51\r\n')
        with self.assertRaisesRegex(TimeoutError, 'no reply for nonce abcd1234i'):
            self.run_test('cdc_arduino', dut, timeout=0.5)

    def test_info_answered_by_a_result_fails(self):
        dut = self.dut()
        reply = 'HIL_RESULT {} info PASS stack=arduino chip=samd51\r\n'
        dut.command = lambda line: setattr(dut, 'out', dut.out + reply.format(line.split()[1]).encode())
        ok, detail = self.run_test('cdc_arduino', dut)
        self.assertFalse(ok)
        self.assertIn('INFO answered HIL_RESULT abcd1234i info PASS', detail)

    def test_timing_bracket(self):
        self.assertEqual(E.core_timing_problem(1000, 995, 1005), '')
        for dev_ms in (980, 1020):   # beyond the bracket widened by TIMING_TOLERANCE and a millis() tick
            with self.subTest(dev_ms=dev_ms):
                self.assertIn(f'{dev_ms:.1f} ms outside the host bracket', E.core_timing_problem(dev_ms, 995, 1005))


class Rosters(unittest.TestCase):
    def test_checked_in_roster_loads(self):
        roster = E.load_roster(ROOT / 'test/hil/ci.json')
        self.assertEqual(sorted(roster), ['metro_m4_express'])
        self.assertEqual(roster['metro_m4_express']['arduino']['tests'], list(ht.ROLE_TESTS['device']))

    def test_example_roster_names_a_profile_board(self):
        boards = json.loads((ROOT / 'test/hil/local.example.json').read_text())['boards']
        for b in boards:
            self.assertIn('span', ht.BUILD.BOARDS[b['name']])
            self.assertEqual(b['arduino']['tests'], list(ht.ROLE_TESTS['device']))


class Profile(unittest.TestCase):
    def test_tests_and_sketches_agree(self):
        sketches = {Path(s).name for s in E.B.hil_sketches('metro_m4_express')}
        self.assertEqual(set(ht.TESTS.values()), sketches)   # the engine's bind() checks the restore and the roles

    def test_engine_is_bound_to_this_profile(self):
        self.assertIs(E.P, ht)
        self.assertIs(E.B.P, ht.BUILD)


if __name__ == '__main__':
    unittest.main()
