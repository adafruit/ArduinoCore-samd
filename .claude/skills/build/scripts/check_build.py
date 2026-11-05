#!/usr/bin/env python3
"""Build this core's bundled-library and HIL sketches with arduino-cli: the project's build contract (SKILL.md).

  check_build.py (--scope PATH... | --base REF [--worktree] | --diff-file FILE) [--example E]... [-j N]
  check_build.py --board B... [--example E]... [--shared] [--build-property K=V]...
  check_build.py --board B --shared --record FILE --hil-config ROSTER [--experimental --build-property K=V...]
  check_build.py --select-only (--scope ... | --base REF [--worktree] | --diff-file FILE) [--hil-config ROSTER]

stdout ends with one JSON line. Exit 0 pass, 1 a build or test failed, 2 usage or a missing
prerequisite ("error" names the remedy), 3 an uncovered path. 2 outranks 1, 1 outranks 3.

This file is the core's profile for agentrc's arduino_build.py in core mode (the `arduino` skill), loaded from the
agentrc test/hil/agentrc.json pins; the mechanics are there.
"""

import json
import re
import shlex
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402

LIBRARY_NAME = 'Adafruit SAMD (32-bits ARM Cortex-M0+ and Cortex-M4) Boards'   # platform.txt name=
PLATFORM = 'adafruit:samd'
SKETCH_ROOTS = ('libraries',)
FQBN_PREFIX = 'adafruit:samd:adafruit_'   # tools/build_all.py's

# CI's matrix (.github/workflows/githubci.yml): each entry is build_all.py's variant, the FQBN suffix and the name its
# .<variant>.test.skip/.test.only markers use (`filters`)
CI_VARIANTS = ('circuitplayground_m0', 'feather_m4_can', 'hallowing', 'hallowing_m4', 'metro_m0', 'metro_m4',
               'pybadge_m4', 'pygamer_m4', 'pyportal_m4', 'pyportal_m4_titano', 'metro_m0:usbstack=tinyusb',
               'metro_m4:speed=120,usbstack=tinyusb')
SAMD51_IDS = ('feather_m4_can', 'hallowing_m4', 'metro_m4', 'pybadge_m4', 'pygamer_m4', 'pyportal_m4',
              'pyportal_m4_titano')
BOARDS = {
    v.split(':')[0] + ('_tinyusb' if 'usbstack=tinyusb' in v else ''): {
        'fqbn': FQBN_PREFIX + v, 'family': 'samd51' if v.split(':')[0] in SAMD51_IDS else 'samd21', 'filters': v}
    for v in CI_VARIANTS
}
# the rig board, named as in agentrc's board registry: Metro M4 under the Arduino USB stack (the core's default);
# `span` is above the UF2 bootloader (variants/metro_m4/linker_scripts/gcc/flash_with_bootloader.ld) and below the
# SAMD51J19's 512 KiB
BOARDS['metro_m4_express'] = {'fqbn': FQBN_PREFIX + 'metro_m4:speed=120,usbstack=arduino', 'family': 'samd51',
                              'filters': 'metro_m4', 'span': (0x4000, 0x80000)}
# a dev PC's SAMD21 board (test/hil/local.example.json), never on the farm: above its 8 KiB bootloader
# (variants/metro_m0/linker_scripts/gcc/flash_with_bootloader.ld), below the SAMD21G18's 256 KiB
BOARDS['metro_m0_express'] = {'fqbn': FQBN_PREFIX + 'metro_m0:usbstack=arduino', 'family': 'samd21',
                              'filters': 'metro_m0', 'span': (0x2000, 0x40000)}
REPRESENTATIVES = {'samd21': ['metro_m0', 'metro_m0_tinyusb'], 'samd51': ['metro_m4_express', 'metro_m4_tinyusb']}
CRYSTALLESS = ('circuitplayground_m0', 'pyportal_m4')   # CI boards whose boards.txt flags define CRYSTALLESS
TINYUSB_BOARDS = ('metro_m0_tinyusb', 'metro_m4_tinyusb')
ARDUINO_USB_BOARDS = ('metro_m0', 'metro_m4_express')
# the sketches a HIL record carries, the first also the restoration image: it writes no flash. core_cdc_tinyusb is
# a copy of core_cdc (a contract test keeps them equal) built with the TinyUSB stack (fqbn())
HIL_SKETCHES = {'device': ('test/hil/core_cdc', 'test/hil/core_cdc_tinyusb', 'test/hil/core_timing')}
CONTRACT_TESTS = {'build': '.claude/test/test_build.py', 'hil': '.claude/test/test_hil.py',
                  'hil_remote': '.claude/test/test_hil_remote.py', 'agentrc_pin': '.claude/test/test_agentrc_pin.py'}
SMOKE_BOARDS = ('metro_m4_express',)
NOTHING = ('.claude/', '.github/', '.agents', 'AGENTS.md', 'CLAUDE.md', 'README.md', 'CHANGELOG', 'LICENSE',
           'VARIANT_COMPLIANCE_CHANGELOG', 'keywords.txt', '.gitignore', '.gitmodules', '.pre-commit-config.yaml',
           '.coderabbit.yaml')
# outside both contracts: no build here makes or flashes them, so a change to one is reported uncovered
UNCOVERED = {'bootloaders/': 'prebuilt bootloaders and their own sources: no build or HIL run here makes or flashes '
                             'them',
             'drivers/': 'Windows USB drivers', 'extras/': 'Board Manager packaging', 'post_install.bat':
             'Windows driver install', 'programmers.txt': 'upload programmers: no build or HIL run uses them',
             'scripts/': 'J-Link and OpenOCD upload/debug scripts: no build or HIL run uses them'}
# libraries every sketch may pull in (SPI's DMA through ZeroDMA, HID through the USB stack): a change builds all
SHARED_LIBRARIES = ('SPI', 'Wire', 'HID', 'Adafruit_ZeroDMA')
# CI's lib install list (.github/workflows/githubci.yml)
EXTRA_DEPENDENCIES = ('Adafruit NeoPixel', 'Adafruit seesaw Library', 'Adafruit SPIFlash', 'FlashStorage',
                      'MIDI Library', 'SD', 'SdFat - Adafruit Fork')
CONTRACT_FILES = ('.claude/skills/build/scripts/check_build.py', '.claude/skills/hil-contract/scripts/hil_test.py',
                  '.claude/skills/hil-contract/scripts/hil_remote.py',
                  '.claude/skills/hil-contract/scripts/agentrc_pin.py', 'test/hil/agentrc.json')
AGENTRC = None   # set by engine(): agentrc_pin.identity()


def boards_txt_variants():
    """{boards.txt id: (build.variant, build.mcu)}, or {} where boards.txt is absent (a tree staged on a rig)."""
    path = ROOT / 'boards.txt'
    if not path.is_file():
        return {}
    out = {}
    for m in re.finditer(r'^(\w+)\.build\.(variant|mcu)=(.*)$', path.read_text(), re.M):
        out.setdefault(m.group(1), {})[m.group(2)] = m.group(3).strip()
    return {i: (v['variant'], v.get('mcu', '')) for i, v in out.items() if 'variant' in v}


# every other boards.txt board builds only when its own variant changes; `filters` is build_all.py's variant name
for _id, (_variant, _mcu) in boards_txt_variants().items():
    _name = _id.removeprefix('adafruit_')
    if _name not in BOARDS:
        BOARDS[_name] = {'fqbn': f'adafruit:samd:{_id}', 'filters': _name,
                         'family': 'samd51' if _mcu == 'cortex-m4' else 'samd21'}


def variant_boards(variant):
    """The board names whose boards.txt id builds `variant`: CI's entries for that id and the off-CI one."""
    ids = [i for i, (v, _) in boards_txt_variants().items() if v == variant]
    return [b for b, e in BOARDS.items() if 'span' not in e and e['fqbn'].split(':')[2] in ids]


def library_sketches(lib):
    return [e for e in E.examples() if e.startswith(f'libraries/{lib}/')]


def classify_source(path):
    """The core's own source rules (arduino_build.classify falls through to the common ones on None; a changed file
    of an example sketch is classified before this)."""
    for prefix, why in UNCOVERED.items():
        if path == prefix or path.startswith(prefix):
            return 'uncovered', None, why
    if path == 'cores/arduino/startup.c':
        return 'boards', tuple(b for f in REPRESENTATIVES.values() for b in f) + CRYSTALLESS, \
            'clock and startup: both families, both USB stacks, crystal and crystalless'
    if path.startswith('cores/arduino/USB/'):
        return 'boards', ARDUINO_USB_BOARDS, 'the Arduino USB stack'
    if path.startswith('cores/'):
        return 'shared', None, 'core runtime'
    if path.startswith('variants/'):
        variant = path.split('/')[1]
        boards = variant_boards(variant)
        return ('boards', tuple(boards), f'variant {variant}') if boards else \
            ('uncovered', None, f'variant {variant} is built by no boards.txt board')
    if path in ('boards.txt', 'platform.txt'):
        return 'boards', tuple(b for b, e in BOARDS.items() if 'span' not in e and e['filters'] in CI_VARIANTS) + \
            ('metro_m4_express',), \
            'board and build recipes: CI\'s matrix and the rig board'
    if path in ('tools/build_all.py', 'tools/makeboards.py', '.github/workflows/githubci.yml'):
        return 'tests', 'build', 'CI\'s matrix, filters or the boards.txt generator: the contract tests check parity'
    if path == 'libraries/Adafruit_TinyUSB_Arduino':
        return 'boards', TINYUSB_BOARDS, 'the bundled TinyUSB (gitlink): the TinyUSB-stack boards'
    if path.startswith('libraries/'):
        lib = path.split('/')[1]
        if lib in SHARED_LIBRARIES:
            return 'shared', None, f'bundled {lib}, which other libraries and sketches include'
        sketches = library_sketches(lib)
        return ('sketches', sketches, f'bundled {lib}') if sketches else \
            ('uncovered', None, f'bundled {lib} has no sketch to build')
    return None


def exclude(board, example):
    """tools/build_all.py's own skips: TinyUSB's examples only on a TinyUSB-stack variant, USBHost's never on one."""
    tinyusb = 'usbstack=tinyusb' in BOARDS[board].get('filters', board)
    return (not tinyusb and example.startswith('libraries/Adafruit_TinyUSB_Arduino/')) or \
        (tinyusb and example.startswith('libraries/USBHost/'))


def fqbn(board, example):
    """The board's FQBN; core_cdc_tinyusb takes the TinyUSB stack instead."""
    base = BOARDS[board]['fqbn']
    return base.replace('usbstack=arduino', 'usbstack=tinyusb') if example == 'test/hil/core_cdc_tinyusb' else base


def vid_pid(build_dir, board=None, roster_entry=None):
    """VID:PID the sketch enumerates with: platform.txt's build.usb_flags pass -DUSB_VID/-DUSB_PID to every compile,
    under either USB stack (compile_commands.json); None when absent or disagreeing."""
    try:
        commands = json.loads((Path(build_dir) / 'compile_commands.json').read_text())
    except (OSError, ValueError):
        return None
    seen = set()
    for entry in commands:
        try:
            argv = entry.get('arguments') or shlex.split(entry.get('command', ''))
        except ValueError:
            return None
        flags = dict(a[2:].split('=', 1) for a in argv if a.startswith(('-DUSB_VID=', '-DUSB_PID=')))
        if 'USB_VID' in flags and 'USB_PID' in flags:
            seen.add(f'{int(flags["USB_VID"], 0):04x}:{int(flags["USB_PID"], 0):04x}')
    return seen.pop() if len(seen) == 1 else None


E = None   # agentrc's arduino_build, bound to this profile by engine()


def engine():
    """agentrc's arduino_build bound to this profile."""
    global AGENTRC, E
    if E is None:
        AGENTRC = agentrc_pin.identity()
        E = agentrc_pin.load('skills/arduino/scripts/arduino_build.py').bind(sys.modules[__name__])
    return E


def main(argv=None):
    try:
        eng = engine()
    except agentrc_pin.PinError as e:
        print(json.dumps({'version': 2, 'pass': False, 'error': str(e), 'remedy': e.remedy}))
        return 2
    return eng.main(argv)


if __name__ == '__main__':
    sys.modules.setdefault('check_build', sys.modules[__name__])
    sys.exit(main())
