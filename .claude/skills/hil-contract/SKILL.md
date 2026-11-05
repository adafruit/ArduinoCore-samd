---
name: hil-contract
description: The Adafruit SAMD core HIL contract, under agentrc's `hil` rules. Use when running the core's HIL battery on rig boards (Metro M4 Express on the ci.lan farm, a dev PC's SAMD21 board), when a HIL run fails, hangs or reports a board locked, when reading its report, or for a manual session (flash, GDB, console, a check, a command) on a rig board from hw-validator or hw-debugger.
---

# HIL contract

Read agentrc's `hil` skill first; this page supplies its contract sections. The mechanics are agentrc's `arduino`
skill (`arduino_hil.py`, `arduino_remote.py`), shared with Adafruit TinyUSB Arduino and Adafruit SPIFlash, whose HIL
contracts have the same commands, budgets, session rules and recovery; this page names what is this core's. Manual
runs only: no CI job runs this battery. Scripts, all in `scripts/`, each with its usage in its docstring:

- `hil_test.py`: the harness on the host with the boards (this core's profile and tests).
- `hil_remote.py`: drives the harness on ci.lan from the dev PC that built the firmware.
- `agentrc_pin.py`: which agentrc both load their shared scripts from: the installed one
  (`${AGENTRC_DIR:-$HOME/.claude}`) on a dev PC, never fetched; in CI the commit `test/hil/agentrc.json` pins; on the
  rig, the copy `hil_remote.py` staged. It refuses an agentrc without core mode.

```bash
H=.claude/skills/hil-contract/scripts
python3 $H/agentrc_pin.py check            # the agentrc a run would load, validated; its root
```

## Hosts/rosters

| Host     | How to tell                        | Roster                                                     |
|----------|------------------------------------|------------------------------------------------------------|
| ci.lan   | the HIL farm; reached over ssh     | `test/hil/ci.json` (tracked)                               |
| a dev PC | a board on this PC's USB           | `test/hil/local.json` (ignored; copy `local.example.json`) |

ci.lan has no arduino-cli and no checkout of this core: firmware is built on the dev PC and staged by
`hil_remote.py`. On a dev PC without `test/hil/local.json`, ask the user for one; an agent that cannot ask returns
each requested board not-run naming the missing file.

A roster entry is the `arduino` skill's (`name`, the board's name in agentrc's board registry and its lock; `uid`, the
DUT's USB serial; `usb_port`; `probe`, its debug probe's serial, whose registry entry supplies the flasher, or a dev
PC's unregistered board's own `flasher`; `debug`) plus `arduino.tests`: `["cdc_arduino", "cdc_tinyusb", "timing"]`.
A board's name must be a rig board of the build profile (`metro_m4_express`, `metro_m0_express`).

ci.lan's `metro_m4_express` is also on tinyusb's roster, tinyuf2's, Adafruit TinyUSB Arduino's and Adafruit
SPIFlash's. This contract performs no rig-wide operation. **Farm check**: tinyusb's launcher,
`ssh ci.lan 'cd ~/code/tinyusb && python3 test/hil/helper/hil_farm_check.py -b <board>...'`.

## Selection

A fixed battery per board, the roster's `arduino.tests`, each over the DUT's USB CDC with a reply line carrying its
command's nonce (a missing, stale or other reply fails the test at its deadline):

- `cdc_arduino` (`test/hil/core_cdc/`, the Arduino USB stack; also the restoration check): `INFO` must report
  `stack=arduino` and the board's chip family; `ECHO` of 1, 63, 64, 65 and 1000 random bytes must return every byte
  as sent, then a `PASS` line with their length and crc32.
- `cdc_tinyusb` (`test/hil/core_cdc_tinyusb/`, a copy of `core_cdc` kept equal by a contract test, built with
  `usbstack=tinyusb`): the same checks with `stack=tinyusb`, through the core's TinyUSB integration and the bundled
  TinyUSB library.
- `timing` (`test/hil/core_timing/`, Arduino stack): `millis()` read twice 3 s apart must lie within the host's
  monotonic bracket of the two exchanges widened by 1%, `micros()` must agree with `millis()` within 2 ms, and
  `delay(1000)` measured by `micros()` must take 1000 to 1002 ms, widened by 1%. A coarse functional bound, no clock
  accuracy claim; `INFO` reports `f_cpu` and `crystalless` for the record.

`-b` narrows the boards. The PR-scoped selector is the build contract's:

```bash
python3 .claude/skills/build/scripts/check_build.py --select-only --base origin/master --hil-config test/hil/ci.json
```

Its `hil.boards` are the roster boards to run, `hil.full` means the harness, a HIL sketch or a roster changed (every
board), and `hil.uncovered` names each family the change reaches with no board on that roster (on the farm,
`samd21`): report those uncovered, never pass.

## Build

Each run board needs a clean HIL record at HEAD (build skill, "A HIL record"), with the submodules at their gitlinks:

```bash
python3 .claude/skills/build/scripts/check_build.py --board <board> --shared --record build/hil/<board>.build.json \
  --hil-config <roster>
```

The harness admits a record only as the `arduino` skill's "The HIL build record" entry says. A manual flash or GDB
session also takes an experimental record (`--experimental`).

## Execution

Budgets, the remote run and the manual session commands are Adafruit TinyUSB Arduino's HIL contract's, with this
core's names: the attempt dir is `~/hil-remote/adafruit-samd/<id>`, `check --test` takes `cdc_arduino` (default),
`cdc_tinyusb` or `timing`, `session close` restores `core_cdc`.

```bash
python3 $H/hil_remote.py run --run-id <id> [-b BOARD]...
python3 $H/hil_remote.py wait <id> --timeout 570    # repeat while it exits 3
python3 $H/hil_test.py report .hil-remote/<id>/hil-run
python3 $H/hil_remote.py session open --run-id <id> -b <board>     # then session exec/stage/close, as TinyUSB's
```

On the dev PC with the board: `python3 $H/hil_test.py run [-b BOARD]... test/hil/local.json`.

## Reservation

As the `arduino` skill's Locks: the board lock `/tmp/tinyusb-hil-locks/<name>.lock` and the pending kind
`arduino-hil` are shared with tinyusb, tinyuf2, Adafruit TinyUSB Arduino and Adafruit SPIFlash. `hil_test.py run` and
`commission` lock each board themselves; a manual session holds it under a reason unique to its attempt,
`arduino-hil session <attempt dir> <token>`. Cleanup state is `core_cdc` flashed and checked, every child proven
ended, and the hold released.

## Results

`hil_test.py run` writes `hil_report.json` (schema `adafruit-samd-core-hil`) with the same rows, exits, verdicts and
delegated output (`hil_test.py report`) as Adafruit TinyUSB Arduino's contract; `firmware.fqbns` names each sketch's
FQBN, so a row says which USB stack each test ran. The caller retries a fail once under a new run id, per the `hil`
Caller rules.

## Recovery

Adafruit TinyUSB Arduino's Recovery, run in this core's attempt dir with its roster and records. The battery writes
no flash beyond the application span (above the UF2 bootloader, `BOARDS[...]['span']`) and keeps no state across
sketches, so restoring `core_cdc` is the whole cleanup.

## Limits

- **Coverage:** the SAMD51 (Metro M4 Express, crystal) under both USB stacks: USB CDC enumeration and bulk data
  integrity, and the core's time base against the host. Not covered on hardware: every SAMD21 board (no farm board;
  `metro_m0_express` runs from a dev PC's `local.json` only), crystalless startup, SERCOM UART/I2C/SPI, analog,
  PWM, I2S, USBHost and every other peripheral (no fixture wired), the bootloaders, cold power-on, and every other
  variant; those build only.
- **Commissioning:** ci.lan's `metro_m4_express` (uid `9995AD485337433231202020FF100A34`, `239a:8020`) ran the
  battery on 2026-11-05: `cdc_arduino`, `cdc_tinyusb` and `timing` passed and `core_cdc` was restored. `timing` read
  `millis()` about 0.3% fast against the host, inside its bound; it makes no clock-accuracy claim.
