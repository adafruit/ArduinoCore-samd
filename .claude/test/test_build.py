#!/usr/bin/env python3
"""The build contract's profile: the board table against CI's matrix and boards.txt, sketch eligibility against
tools/build_all.py, source classification and selection through agentrc's arduino_build.py in core mode, and the
VID:PID adapter. arduino-cli is not run; a real build is evidence only from a run of the script itself."""
import ast
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'build' / 'scripts'))
import check_build as cb  # noqa: E402

E = cb.engine()


def ci_matrix():
    text = (ROOT / '.github/workflows/githubci.yml').read_text()
    return re.findall(r"^\s+- '([^']+)'", text.split('board:')[1].split('runs-on')[0], re.M)


def build_all_literal(name):
    for node in ast.parse((ROOT / 'tools/build_all.py').read_text()).body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


class Tables(unittest.TestCase):
    def test_ci_matrix_is_the_table(self):
        self.assertEqual(sorted(ci_matrix()), sorted(cb.CI_VARIANTS))
        self.assertEqual(build_all_literal('FQBN_PREFIX'), cb.FQBN_PREFIX)
        for v in cb.CI_VARIANTS:
            entries = [b for b in cb.BOARDS.values() if b['filters'] == v and 'span' not in b]
            self.assertEqual([b['fqbn'] for b in entries], [cb.FQBN_PREFIX + v], v)

    def test_ci_dependencies_are_the_extras(self):
        text = (ROOT / '.github/workflows/githubci.yml').read_text()
        line = next(ln for ln in text.splitlines() if 'arduino-cli lib install' in ln)
        self.assertEqual(sorted(re.findall(r'"([^"]+)"', line)), sorted(cb.EXTRA_DEPENDENCIES))

    def test_library_name_is_the_platforms(self):
        self.assertIn(f'name={cb.LIBRARY_NAME}\n', (ROOT / 'platform.txt').read_text())

    def test_every_boards_txt_board_has_an_entry_of_its_family(self):
        ids = cb.boards_txt_variants()
        self.assertTrue(ids)
        for i, (_, mcu) in ids.items():
            entries = [b for b in cb.BOARDS.values() if b['fqbn'].split(':')[2] == i]
            self.assertTrue(entries, i)
            for b in entries:
                self.assertEqual(b['family'], 'samd51' if mcu == 'cortex-m4' else 'samd21', i)

    def test_boards_txt_is_makeboards_output(self):
        out = subprocess.run([sys.executable, str(ROOT / 'tools/makeboards.py')], capture_output=True, text=True,
                             check=True).stdout
        self.assertEqual(out, (ROOT / 'boards.txt').read_text(), 'edit tools/makeboards.py and regenerate')

    def test_crystalless_representatives_define_it(self):
        text = (ROOT / 'boards.txt').read_text()
        for b in cb.CRYSTALLESS:
            board_id = cb.BOARDS[b]['fqbn'].split(':')[2]
            self.assertRegex(text, rf'(?m)^{board_id}\.build\.extra_flags=.*-DCRYSTALLESS\b', b)

    def test_representatives_and_rig_boards(self):
        for family, boards in cb.REPRESENTATIVES.items():
            for b in boards:
                self.assertEqual(cb.BOARDS[b]['family'], family)
        self.assertEqual({b for b, v in cb.BOARDS.items() if 'span' in v}, {'metro_m4_express', 'metro_m0_express'})
        for b in ('metro_m4_express', 'metro_m0_express'):
            self.assertIn('usbstack=arduino', cb.BOARDS[b]['fqbn'])
            self.assertIn(cb.BOARDS[b]['filters'], cb.CI_VARIANTS)

    def test_hil_sketches_exist_and_tinyusb_is_core_cdc_rebuilt(self):
        for sketch in cb.HIL_SKETCHES['device']:
            self.assertTrue((ROOT / sketch / f'{Path(sketch).name}.ino').is_file(), sketch)
        self.assertEqual((ROOT / 'test/hil/core_cdc_tinyusb/core_cdc_tinyusb.ino').read_bytes(),
                         (ROOT / 'test/hil/core_cdc/core_cdc.ino').read_bytes())
        self.assertEqual(cb.fqbn('metro_m4_express', 'test/hil/core_cdc_tinyusb'),
                         'adafruit:samd:adafruit_metro_m4:speed=120,usbstack=tinyusb')
        self.assertEqual(cb.fqbn('metro_m4_express', 'test/hil/core_cdc'), cb.BOARDS['metro_m4_express']['fqbn'])

    def test_unbuildable_examples_are_pinned(self):
        # the sketches tools/build_all.py skips on every CI variant (their .test.only/.test.skip markers): no build
        # here compiles them either
        self.assertEqual(E.unbuildable_examples(), [
            'libraries/Adafruit_TinyUSB_Arduino/examples/Host/Simple/host_device_info',
            'libraries/Adafruit_TinyUSB_Arduino/examples/MIDI/midi_pizza_box_dj',
            'libraries/Adafruit_TinyUSB_Arduino/examples/MassStorage/msc_esp32_file_browser',
            'libraries/Adafruit_TinyUSB_Arduino/examples/MassStorage/msc_internal_flash_samd',
            'libraries/Adafruit_ZeroDMA/examples/zerodma_adc'])

    def test_contract_files_exist(self):
        for f in cb.CONTRACT_FILES:
            self.assertTrue((ROOT / f).is_file(), f)


class Eligibility(unittest.TestCase):
    """build_all.py's rules: its markers and its TinyUSB/USBHost skips."""

    def sketch(self, lib):
        return next(e for e in E.examples() if e.startswith(f'libraries/{lib}/'))

    def test_tinyusb_examples_only_on_tinyusb_variants(self):
        ex = self.sketch('Adafruit_TinyUSB_Arduino')
        self.assertEqual([b for b in ('metro_m0', 'metro_m0_tinyusb', 'metro_m4', 'metro_m4_tinyusb')
                          if E.eligible(b, ex)], ['metro_m0_tinyusb', 'metro_m4_tinyusb'])

    def test_usbhost_examples_never_on_tinyusb_variants(self):
        ex = self.sketch('USBHost')
        self.assertTrue(E.eligible('metro_m0', ex))
        self.assertFalse(E.eligible('metro_m0_tinyusb', ex))

    def test_sketches_outside_examples_are_scanned_like_ci(self):
        self.assertIn('libraries/SDU/extras/SDUBoot', E.examples())


class Selection(unittest.TestCase):
    def test_rules(self):
        reps = tuple(b for f in cb.REPRESENTATIVES.values() for b in f)
        cases = {'cores/arduino/startup.c': ('boards', reps + cb.CRYSTALLESS),
                 'cores/arduino/USB/CDC.cpp': ('boards', cb.ARDUINO_USB_BOARDS),
                 'cores/arduino/SERCOM.cpp': ('shared', None),
                 'libraries/SPI/SPI.cpp': ('shared', None),
                 'libraries/Adafruit_ZeroDMA': ('shared', None),
                 'libraries/Adafruit_TinyUSB_Arduino': ('boards', cb.TINYUSB_BOARDS),
                 'libraries/Servo/src/Servo.h': ('sketches', [e for e in E.examples()
                                                              if e.startswith('libraries/Servo/')]),
                 'libraries/Wire/examples/master_reader/master_reader.ino':
                     ('example', 'libraries/Wire/examples/master_reader'),
                 'test/hil/core_cdc/core_cdc.ino': ('hil_sketch', 'test/hil/core_cdc'),
                 'test/hil/ci.json': ('hil', None),
                 '.claude/skills/build/scripts/check_build.py': ('tests', 'build'),
                 'tools/build_all.py': ('tests', 'build'), 'tools/makeboards.py': ('tests', 'build'),
                 '.github/workflows/githubci.yml': ('tests', 'build'),
                 'README.md': ('nothing', None), 'CLAUDE.md': ('nothing', None), '.coderabbit.yaml': ('nothing', None),
                 'bootloaders/metroM4/main.c': ('uncovered', None), 'extras/pack.release.bash': ('uncovered', None),
                 'scripts/openocd/jlink_samd51.cfg': ('uncovered', None), 'programmers.txt': ('uncovered', None),
                 'weird/file': ('uncovered', None)}
        for path, want in cases.items():
            self.assertEqual(E.classify(path)[:2], want, path)

    def test_a_variant_builds_every_board_on_it(self):
        effect, boards, _ = E.classify('variants/metro_m0/variant.cpp')
        self.assertEqual((effect, sorted(boards)), ('boards', ['metro_m0', 'metro_m0_tinyusb']))
        effect, boards, _ = E.classify('variants/itsybitsy_m4/variant.h')   # on no CI board
        self.assertEqual((effect, boards), ('boards', ('itsybitsy_m4',)))
        self.assertEqual(E.classify('variants/nowhere/variant.h')[0], 'uncovered')

    def test_boards_txt_builds_ci_and_the_rig_board(self):
        effect, boards, _ = E.classify('boards.txt')
        self.assertEqual(effect, 'boards')
        self.assertEqual(sorted(boards), sorted([b for b, e in cb.BOARDS.items() if 'span' not in e
                                                 and e['filters'] in cb.CI_VARIANTS] + ['metro_m4_express']))

    def test_core_change_runs_the_farm_board_and_names_samd21_uncovered(self):
        sel = E.select(['cores/arduino/wiring.c'], roster=['metro_m4_express'])
        self.assertEqual(sorted(sel['builds']), sorted(b for f in cb.REPRESENTATIVES.values() for b in f))
        self.assertEqual(sel['hil']['boards'], ['metro_m4_express'])
        self.assertEqual([u['family'] for u in sel['hil']['uncovered']], ['samd21'])

    def test_hil_sketch_builds_only_on_rig_boards(self):
        sel = E.select(['test/hil/core_timing/core_timing.ino'], roster=['metro_m4_express'])
        self.assertEqual(sorted(sel['builds']), ['metro_m0_express', 'metro_m4_express'])
        self.assertTrue(sel['hil']['full'])


class Identity(unittest.TestCase):
    def commands(self, *flags):
        d = Path(self.tmp.name)
        (d / 'compile_commands.json').write_text(json.dumps(
            [{'file': f'/x/{i}.cpp', 'arguments': ['arm-none-eabi-g++', *f.split()]} for i, f in enumerate(flags)]))
        return cb.vid_pid(d)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_vid_pid_from_the_compile_flags(self):
        self.assertEqual(self.commands('-DUSB_VID=0x239A -DUSB_PID=0x8020 -DUSBCON', '-O2'), '239a:8020')

    def test_vid_pid_absent_or_disagreeing_is_none(self):
        self.assertIsNone(self.commands('-O2'))
        self.assertIsNone(self.commands('-DUSB_VID=0x239A -DUSB_PID=0x8020', '-DUSB_VID=0x239A -DUSB_PID=0x8021'))
        self.assertIsNone(cb.vid_pid(Path(self.tmp.name) / 'missing'))


if __name__ == '__main__':
    unittest.main()
