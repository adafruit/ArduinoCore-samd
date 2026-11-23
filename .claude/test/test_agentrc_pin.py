#!/usr/bin/env python3
"""agentrc_pin.py in this core: its test/hil/agentrc.json configuration and the agentrc this checkout really selects.
The bootstrap's common behaviour is tested in agentrc (tests/test_arduino_pin.py)."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy

HIL_LOAD = 'skills/hil/scripts/hil_load.py'


class Configuration(unittest.TestCase):
    def test_the_configuration_is_this_cores(self):
        data = agentrc_pin.config()
        self.assertEqual(agentrc_pin.PIN_FILE, ROOT / 'test' / 'hil' / 'agentrc.json')
        self.assertRegex(data['commit'], r'^[0-9a-f]{40}$')
        # core mode (PLATFORM), named HIL builds (core_cdc_tinyusb) and the shared core battery (CORE_TESTS)
        self.assertEqual((data['cache_namespace'], data['profile_api'], data['features'], data['extra_engines']),
                         ('adafruit-samd', 1, ['core', 'named', 'battery'], []))

    def test_the_features_are_the_ones_the_profiles_use(self):
        self.assertEqual(hil_test.BUILD.PLATFORM, 'adafruit:samd')
        self.assertTrue(any(isinstance(v, dict) for sk in hil_test.BUILD.HIL_SKETCHES.values() for v in sk))
        self.assertTrue(hil_test.CORE_TESTS)

    def test_both_bootstrap_files_are_contract_files(self):
        for f in ('.claude/skills/hil-contract/scripts/agentrc_pin.py', 'test/hil/agentrc.json'):
            self.assertIn(f, hil_test.BUILD.CONTRACT_FILES)


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_the_copy_is_the_selected_canonical(self):
        root = Path(agentrc_pin.identity()['root'])
        self.assertEqual(Path(agentrc_pin.__file__).read_bytes(), (root / agentrc_pin.SELF).read_bytes())

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertLessEqual({*agentrc_pin.ENGINES, agentrc_pin.SELF, HIL_LOAD,
                              'skills/hil/scripts/hil_remote_core.py'}, set(ident['files']))
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertIs(hil_test.BUILD.AGENTRC, ident)

    def test_the_real_engines_load_from_their_staged_closure_alone(self):
        # staged here, loaded in a fresh interpreter: one process loads one copy of agentrc
        code = '''if True:
            import json, sys, sysconfig
            from pathlib import Path
            from unittest import mock
            sys.path.insert(0, sys.argv[1])
            import agentrc_pin
            d = Path(sys.argv[2])
            before = set(sys.modules)
            with mock.patch.object(agentrc_pin, 'ROOT', d), \\
                    mock.patch.object(agentrc_pin, 'checkout', side_effect=AssertionError('a rig has no cache')), \\
                    mock.patch.object(agentrc_pin, 'installed', side_effect=AssertionError('a rig has no install')):
                staged = agentrc_pin.identity()
                engines = [agentrc_pin.load(e).__file__ for e in agentrc_pin.ENGINES]
            lib = tuple(sysconfig.get_paths()[k] for k in ('stdlib', 'platstdlib', 'purelib', 'platlib'))
            new = sorted(f for n in set(sys.modules) - before
                         if (f := getattr(sys.modules[n], '__file__', None)) and not f.startswith(lib))
            print(json.dumps({'engines': engines, 'modules': new, 'source': staged['source'],
                              'ident': {k: staged[k] for k in ('commit', 'files', 'sha256')}}))
        '''
        helper, _, ident = agentrc_pin._selection()
        with tempfile.TemporaryDirectory() as d:
            helper.stage(ident, d, name='agentrc')
            Path(d, 'manifest.json').write_text('{}')
            p = subprocess.run([sys.executable, '-I', '-c', code, str(Path(agentrc_pin.__file__).parent), d],
                               capture_output=True, text=True)
            self.assertEqual(p.returncode, 0, p.stderr)
            got = json.loads(p.stdout)
            staged = Path(d, 'agentrc').resolve()
            self.assertEqual(got['engines'], [str(Path(d, 'agentrc', e)) for e in agentrc_pin.ENGINES])
            self.assertEqual(got['source'], 'staged')
            self.assertEqual(got['ident'], {k: ident[k] for k in ('commit', 'files', 'sha256')})
            for f in got['modules']:
                self.assertTrue(Path(f).resolve().is_relative_to(staged), f)


if __name__ == '__main__':
    unittest.main()
