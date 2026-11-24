#!/usr/bin/env python3
"""agentrc_pin.py in this core: its test/hil/agentrc.json configuration and the agentrc this checkout really selects.
The bootstrap's common behaviour is tested in agentrc (tests/test_arduino_pin.py); `agentrc_pin.py test` compares this
copy with the canonical one and checks the staged round trip, contract files and load() targets (pin_conformance.py)."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / '.claude' / 'skills' / 'hil-contract' / 'scripts'))
import agentrc_pin  # noqa: E402
import hil_test  # noqa: E402

hil_test.engine()   # the selected agentrc: a missing one fails here, naming the remedy


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


class Selected(unittest.TestCase):
    """The agentrc this checkout really selects: the installed one on a dev PC, the pin in CI."""

    def test_hil_test_binds_the_selected_identity(self):
        ident = agentrc_pin.identity()
        self.assertEqual(ident['source'], 'pin' if agentrc_pin.ci() else 'installed')
        self.assertIs(hil_test.BUILD.AGENTRC, ident)


if __name__ == '__main__':
    unittest.main()
