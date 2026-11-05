# Adafruit SAMD Core Agent Instructions

## Working Rules

- An Arduino core (the `adafruit:samd` board package) with no standalone build: sketches compile against this checkout
  as that platform through the build contract's isolated arduino-cli environment, never the installed board package
  or a `~/Arduino/hardware` link to another checkout. CI's board matrix is in `.github/workflows/githubci.yml`, its
  FQBNs and skip rules in `tools/build_all.py`. Initialise the submodules (`git submodule update --init`) before any
  build.
- Development boards: `metro_m0` (SAMD21) and `metro_m4` (SAMD51), each under both USB stacks; on hardware, the
  ci.lan farm's `metro_m4_express`.
- Code references — runtime: `cores/arduino/` (`startup.c` clocks, `SERCOM.*`, `wiring_*`, `cortex_handlers.c`, the
  Arduino USB stack in `USB/`); boards: `variants/<variant>/` (`variant.*`, `linker_scripts/gcc/`), `boards.txt`,
  `platform.txt`; bundled libraries: `libraries/` (SPI and Wire on SERCOM, `Adafruit_TinyUSB_Arduino` and
  `Adafruit_ZeroDMA` as submodules); `bootloaders/`: prebuilt bootloaders and legacy bootloader sources with their
  own make recipes, outside both contracts.

## Code Rules

- `boards.txt` is generated: edit `tools/makeboards.py` and regenerate (`python3 tools/makeboards.py > boards.txt`);
  a contract test fails when they differ.
- SAMD21 and SAMD51 diverge under `__SAMD51__`; `CRYSTALLESS` and `USE_TINYUSB` come from `boards.txt` flags and the
  `usbstack` menu, never from a source default.
- Check clock, register and peripheral behaviour against the SAMD21/SAMD51 datasheet and errata (agentrc's `read-doc`)
  before changing or reviewing code that depends on it.
- Fix the submodules' code upstream (adafruit/Adafruit_TinyUSB_Arduino, adafruit/Adafruit_ZeroDMA) and bump the
  gitlink; never edit it here.
- `.pre-commit-config.yaml` is authoritative: the contract tests, and clang-format v15 on `test/hil/` only (the core's
  existing code is not reformatted).
- Keep each file's line endings.

## Skills

- Agents, skills and workflows beyond this repo's `.claude/` come from [agentrc](https://github.com/hathach/agentrc);
  install it before agent work on this core. Its `arduino` skill's core mode holds the mechanics both contracts below
  run, shared with Adafruit TinyUSB Arduino and Adafruit SPIFlash.
- Skill script tests live in `.claude/test/test_*.py`; pre-commit runs them against the installed agentrc
  (`${AGENTRC_DIR:-$HOME/.claude}`); CI fetches the commit `test/hil/agentrc.json` pins instead.
- Hardware diagnosis and repair use agentrc's `hw-debugger`; independent hardware claims and committed-fix validation
  use `hw-validator`; routine HIL runs use `hil-operator` — all orchestrated by `chief`.
- Core audit: the `code-audit` workflow with `dirs` (e.g. `cores/arduino`, `cores/arduino/USB`, `libraries/Wire`) and
  `dimensions`: `clocks and startup: oscillator, DFLL and GCLK setup, crystal and crystalless boards, SAMD21 vs
  SAMD51`; `SERCOM and its libraries: UART, SPI and I2C setup, pin muxing, the variants' assignments`; `USB stacks:
  the Arduino stack and the TinyUSB integration, enumeration, CDC`; `interrupts and DMA: handler and priority setup,
  shared state with ISRs, ZeroDMA`; `style: repo conventions and the clang-format scope`.

## Build and Validate

- Build contract: `.claude/skills/build/SKILL.md`. It resolves a change to boards and sketches and builds them with
  arduino-cli against this checkout; `--shared --record` writes the HIL build record and keeps the images HIL flashes.
- HIL contract: `.claude/skills/hil-contract/SKILL.md`. It owns taking and releasing a rig board, names the roster for
  the host you are on, runs the battery on a dev PC's board or the ci.lan farm, and gives hw-validator and
  hw-debugger their manual sessions. Manual runs only: no CI job runs it. Hardware validation comes from its report,
  never from a build; the farm has no SAMD21 board, so a change reaching SAMD21 is reported pending on hardware.
- Before submitting, and again after a rebase: `pre-commit run --all-files`.
- For code changes: `python3 .claude/skills/build/scripts/check_build.py --base origin/master --worktree`.

## PRs and Follow-ups

- Before opening or updating a PR, follow Build and Validate; CI does not run HIL.
- A non-draft PR opened on the human's request, unless they declined babysitting, or their request or yes to babysit
  one, authorizes one headless `chief` to drive its reviews and CI to green: push commits to its branch, reply to and
  resolve its review threads through `pr-babysit` with `autoPush: true` and `markSonar: true`. Launch it in the PR
  worktree through agentrc's `headless-chief` skill (`chief_run.py`), never a bare `claude -p`, and relay its exit as
  that skill directs. Merging needs the human's own words asking to merge that PR. `pr-babysit` args:
  `{"pr": <num>, "protected": "^test/hil/[^/]+\\.json$"}`; reviewers are agentrc's default (`protected` keeps the HIL
  rosters and the agentrc pin out of automated fixes).
- This repository has no follow-up label: label a follow-up issue with an applicable existing label, or leave it
  unlabeled.
