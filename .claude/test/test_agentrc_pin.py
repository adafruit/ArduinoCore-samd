#!/usr/bin/env python3
"""agentrc_pin.py in this core: its test/hil/agentrc.json configuration, the agentrc this checkout really selects, and
the staged copy a rig tree carries. The bootstrap's common behaviour is tested in agentrc (tests/test_arduino_pin.py)."""
import json
import os
import shutil
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

CONFIG = 'test/hil/agentrc.json'
CONSUMER = '.claude/skills/hil-contract/scripts/agentrc_pin.py'
CLEAN_ENV = {k: v for k, v in os.environ.items()
             if not k.startswith('GIT_') and k not in ('AGENTRC_DIR', 'CI', 'GITHUB_ACTIONS')}


class Configuration(unittest.TestCase):
    def test_the_configuration_is_this_cores(self):
        data = agentrc_pin.config()
        # core mode (PLATFORM), named HIL builds (core_cdc_tinyusb) and the shared core battery (CORE_TESTS)
        self.assertEqual((data['cache_namespace'], data['profile_api'], data['features'], data['extra_engines']),
                         ('adafruit-samd', 1, ['core', 'named', 'battery'], []))

    def test_the_features_are_the_ones_the_profiles_use(self):
        self.assertEqual(hil_test.BUILD.PLATFORM, 'adafruit:samd')
        self.assertTrue(any(isinstance(v, dict) for sk in hil_test.BUILD.HIL_SKETCHES.values() for v in sk))
        self.assertTrue(hil_test.CORE_TESTS)

    def test_both_bootstrap_files_are_contract_files(self):
        self.assertLessEqual({CONSUMER, CONFIG}, set(hil_test.BUILD.CONTRACT_FILES))


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_the_copy_is_the_selected_canonical(self):
        canonical = Path(agentrc_pin.identity()['root']) / 'skills/arduino/scripts/agentrc_pin.py'
        self.assertEqual((ROOT / CONSUMER).read_bytes(), canonical.read_bytes(), canonical)

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertLessEqual({*agentrc_pin.ENGINES, agentrc_pin.SELF, *agentrc_pin.FILES,
                              'skills/hil/scripts/hil_remote_core.py'}, set(ident['files']))
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertIs(hil_test.BUILD.AGENTRC, ident)

    def test_a_staged_tree_loads_every_script_from_its_own_copy(self):
        # a fresh interpreter: the staged modules must never replace the engines the other tests load
        helper, _, ident = agentrc_pin._selection()
        with tempfile.TemporaryDirectory() as d:
            tree = Path(d)
            for rel in (CONSUMER, CONFIG):
                (tree / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / rel, tree / rel)
            helper.stage(ident, tree, name='agentrc')   # as arduino_remote.py stages a rig tree
            (tree / 'manifest.json').write_text(json.dumps({'agentrc': {'commit': ident['commit'],
                                                                        'dirty': ident['dirty']}}))
            code = '''if True:
                import json, os, sys, sysconfig
                sys.path.insert(0, sys.argv[1])
                import agentrc_pin as pin
                before = set(sys.modules)
                ident = pin.identity()
                engines = [pin.load(e).__file__ for e in (*pin.ENGINES, *pin.config()['extra_engines'])]
                paths = {k: os.path.join(v, '') for k, v in sysconfig.get_paths().items()}
                lib = (paths['stdlib'], paths['platstdlib'])
                site = (paths['purelib'], paths['platlib'])  # may sit inside stdlib
                modules = sorted(f for n in set(sys.modules) - before
                                 if (f := getattr(sys.modules[n], '__file__', None))
                                 and (f.startswith(site) or not f.startswith(lib)))
                print(json.dumps({'source': ident['source'], 'commit': ident['commit'], 'files': ident['files'],
                                  'sha256': ident['sha256'], 'engines': engines, 'modules': modules}))
            '''
            p = subprocess.run([sys.executable, '-I', '-B', '-c', code, str((tree / CONSUMER).parent)],
                               capture_output=True, text=True,
                               env={**CLEAN_ENV, 'CI': 'true', 'AGENTRC_DIR': str(tree / 'absent')})
            self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
            got = json.loads(p.stdout)
            self.assertEqual((got['source'], got['commit'], got['files'], got['sha256']),
                             ('staged', ident['commit'], ident['files'], ident['sha256']))
            staged = (tree / 'agentrc').resolve()
            for f in (*got['engines'], *got['modules']):
                self.assertTrue(Path(f).resolve().is_relative_to(staged), f)


if __name__ == '__main__':
    unittest.main()
