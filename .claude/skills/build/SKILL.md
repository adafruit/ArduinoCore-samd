---
name: build
description: The Adafruit SAMD core build contract. Use when a change must be verified by building, when an agent needs the boards and sketches a diff affects, when a HIL run or debug session needs firmware and its build record, or when a workflow asks for "the project's build". Resolves scope to boards, compiles the bundled libraries' sketches and the HIL sketches with arduino-cli against this checkout as the adafruit:samd platform, reports pass/fail with the first error.
---

# Build contract

`scripts/check_build.py` is the entry point for verifying a change; agents and workflows call it rather than composing
`arduino-cli compile` lines. It is this core's profile for agentrc's `arduino` skill (`arduino_build.py`) in core
mode, loaded from the installed agentrc (`${AGENTRC_DIR:-$HOME/.claude}`), never fetched; CI fetches the commit
`test/hil/agentrc.json` pins. `.claude/skills/hil-contract/scripts/agentrc_pin.py check` names the one in use. That
skill's page ("A core") is the shared behaviour: the isolated arduino-cli environment that makes this checkout the
`adafruit:samd` platform whatever is installed or linked under `~/Arduino/hardware`, bundled libraries as the core's
own, submodule identity and the record. This page is what is this core's.

```bash
B=.claude/skills/build/scripts/check_build.py
python3 $B --scope <changed paths or dirs>          # boards and sketches a change affects
python3 $B --base origin/master                      # same, from the branch diff
python3 $B --base origin/master --worktree           # same, plus uncommitted and untracked changes
python3 $B --select-only (--scope ... | --base REF [--worktree] | --diff-file FILE) [--hil-config ROSTER]
python3 $B --board metro_m4 [--board ...] [--example libraries/Wire/examples/master_reader ...]
python3 $B --board metro_m4_express --shared --record build/hil/metro_m4_express.build.json \
  --hil-config test/hil/ci.json
```

Submodules (`libraries/Adafruit_TinyUSB_Arduino`, `libraries/Adafruit_ZeroDMA`) must be initialised at their
gitlinks first: `git submodule update --init`. Dependencies are CI's install list (`EXTRA_DEPENDENCIES`), installed
beforehand in the user's Arduino libraries (exit 2 names the `arduino-cli lib install` line).

The last stdout line is JSON: `pass`; `firstError`, and `error` on exit 2 naming the cause or remedy; `boards[]` (one
row per board and sketch: `status` `ok`, `failed` or `error`, `fqbn`, `log`, `artifacts`, `platform`,
`foreignHeaders`, `libraries`, `removed`); `tests[]`; `paths[]` (each changed path's `rule` and `reason`);
`uncovered`; `dependencies`; `nothingToBuild`; `warnings`; `input` (HEAD, dirty, submodules); `narrowed`; `record`
when one was written. Exit 0 pass, 1 a build, test or record check failed, 2 usage or a missing prerequisite, 3 an
uncovered path; 2 outranks 1, and 1 outranks 3. Each build's log is `<buildDir>/check_build.log`.

`--select-only` prints the selection and stops before any compile. With `--hil-config <the host's roster>` it adds
`hil.{boards,full,uncovered}`: the roster boards the change puts under the HIL battery, and each family (`samd21`,
`samd51`) the change reaches with no roster board on this host.

## Reporting

An agent that reports `buildOk` and `notes` (agentrc's `code-writer`) maps the JSON this way: `buildOk` is `pass`. A
pass with no build or test run is "nothing to build": say so in `notes`. Exit 3 is `buildOk` false with each
`uncovered` reason quoted verbatim. Exit 1 or 2 quotes `firstError` (and `error`). Each `warnings` entry is quoted in
`notes`.

## Judgment

- **Selection is the script's**, first match per path; `paths[].reason` says why:
  - a changed file of a bundled library's sketch builds that sketch wherever it is eligible;
  - `cores/arduino/startup.c` builds the representatives and the crystalless CI boards (`CRYSTALLESS`);
    `cores/arduino/USB/` the Arduino-stack representatives; any other `cores/` file every representative;
  - `variants/<v>/` every `boards.txt` board on that variant, CI's or not; `boards.txt` and `platform.txt` CI's
    whole matrix and the rig board;
  - SPI, Wire, HID and the ZeroDMA gitlink (`SHARED_LIBRARIES`) every representative; the TinyUSB gitlink the
    TinyUSB-stack boards; another bundled library its own sketches on the representatives;
  - `tools/build_all.py`, `tools/makeboards.py` and the CI workflow run the contract tests, which hold the board
    table to CI's matrix and `boards.txt` to `makeboards.py`'s output;
  - `bootloaders/`, `drivers/`, `extras/`, `scripts/`, `programmers.txt` and `post_install.bat` are uncovered
    (`UNCOVERED`): nothing here builds, flashes or uses them; anything unmatched is uncovered too.
- **Boards** (`BOARDS`): CI's matrix with `tools/build_all.py`'s FQBNs (`adafruit:samd:adafruit_<variant>`), its
  skip markers and its own skips (TinyUSB's examples only on a TinyUSB-stack variant, USBHost's never on one); every
  other `boards.txt` board, built only when its variant changes; and the rig boards: `metro_m4_express` (the farm)
  and `metro_m0_express` (a dev PC's SAMD21), each on the Arduino USB stack. Representatives: `metro_m0` and
  `metro_m0_tinyusb` (SAMD21), `metro_m4_express` and `metro_m4_tinyusb` (SAMD51).
- **Sketches** are every `.ino` dir under `libraries/`, as CI scans them (`SDU/extras/SDUBoot` included). The
  contract tests pin the ones no CI variant builds (`unbuildable_examples()`).
- **A HIL record** (`--board B --shared --record FILE --hil-config ROSTER`, by convention
  `build/hil/<board>.build.json`) is the HIL contract's build receipt. It carries `core_cdc` (Arduino stack, the
  restoration image), `core_cdc_tinyusb` (the same source under the TinyUSB stack, its own FQBN) and `core_timing`;
  each sketch's VID:PID is the `-DUSB_VID`/`-DUSB_PID` `platform.txt` passes to every compile.
- **Private dirs for parallel agents.** Each build uses `build/agent-<pid>/<board>/<sketch path>`, removed when it
  passes; `--shared` builds into `build/hil/<board>/<sketch>` under `build/hil/.lock` and keeps it for flashing.
- **Printing a command is not verification.** A verdict is the JSON of a run that happened; an agent that cannot run
  the script reports the build as not run.
