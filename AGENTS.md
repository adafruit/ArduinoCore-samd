# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

Adafruit's fork of the Arduino Core for SAMD21 (Cortex-M0+) and SAMD51 (Cortex-M4) microcontrollers. Provides the board support package (BSP) installed via Arduino Board Manager for Adafruit SAMD boards.

## Build & Test Commands

This is an Arduino BSP — there is no standalone build. Code is compiled via Arduino CLI.

**Default/preferred boards**: Use **metro_m0** (SAMD21/M0) and **metro_m4** (SAMD51/M4) as the reference boards for development and testing.

```bash
# Install Arduino CLI and set up BSP (CI does this automatically)
# Build all examples for a specific board:
python3 tools/build_all.py adafruit_metro_m0   # SAMD21/M0
python3 tools/build_all.py adafruit_metro_m4   # SAMD51/M4

# FQBN format: adafruit:samd:adafruit_<board_id>

# Build a single sketch:
arduino-cli compile --fqbn adafruit:samd:adafruit_metro_m0 path/to/sketch  # SAMD21
arduino-cli compile --fqbn adafruit:samd:adafruit_metro_m4 path/to/sketch  # SAMD51

# Upload:
arduino-cli upload --fqbn adafruit:samd:adafruit_metro_m0 -p /dev/ttyACM0 path/to/sketch
arduino-cli upload --fqbn adafruit:samd:adafruit_metro_m4 -p /dev/ttyACM0 path/to/sketch
```

CI runs `tools/build_all.py` across a matrix of boards to compile all library examples. When testing changes, always verify against both metro_m0 (M0/SAMD21) and metro_m4 (M4/SAMD51) to cover both chip families.

## Architecture

### Core Build System Files
- **`platform.txt`** — Defines compiler toolchain (arm-none-eabi-gcc), flags, upload tools (bossac, openocd), and build recipes. This is the central build configuration.
- **`boards.txt`** — Board definitions (36+ boards): MCU type, clock speed, memory layout, USB VID/PID, upload protocol, menu options (optimization level, USB stack selection).
- **`programmers.txt`** — J-Link and Atmel-ICE debugger configurations.

### `cores/arduino/` — Core Runtime
The Arduino API implementation for SAMD. Key subsystems:
- **`startup.c`** — Clock initialization, oscillator config, peripheral clocks (differs significantly between SAMD21 and SAMD51 via `#ifdef __SAMD51__`)
- **`SERCOM.cpp/h`** — Abstraction over SAMD SERCOM peripherals. SERCOMs are flexible hardware blocks that can be configured as UART, SPI, or I2C. This is the foundation for Wire, SPI, and Serial libraries.
- **`wiring_analog.c`** — ADC/DAC operations with SAMD21/SAMD51 divergence
- **`cortex_handlers.c`** — ARM exception/interrupt vector table and handlers
- **`USB/`** — Native Arduino USB stack (CDC, HID, PluggableUSB)

### `variants/` — Board Pin Definitions (49 variants)
Each variant has:
- **`variant.h`** — Pin count, pin-to-peripheral mappings, LED pins, SERCOM assignments
- **`variant.cpp`** — `g_APinDescription[]` array mapping Arduino pin numbers to physical port/pin/function/PWM/ADC
- **`linker_scripts/`** — Memory layout (`flash_with_bootloader.ld` is the typical one)

When adding a new board: create a variant directory, add entries to `boards.txt`, and provide a bootloader binary.

### `libraries/` — Bundled Libraries
- **SPI, Wire** — SERCOM-based, support multiple instances via different SERCOM peripherals
- **Adafruit_TinyUSB_Arduino** — Alternative USB stack (submodule), supports mass storage, HID, MIDI, etc.
- **Adafruit_ZeroDMA** — DMA controller library (submodule)
- **Servo, I2S, HID, SDU** — Standard peripheral libraries

### `bootloaders/` — Prebuilt UF2 Bootloader Binaries
One binary per board. These enable USB drag-and-drop and bossac uploads.

## Key Patterns

- **SAMD21 vs SAMD51 divergence**: Most low-level code uses `#ifdef __SAMD51__` (or `_SAMD21_`) to handle register-level differences. The two chips have different clock trees, peripheral registers, and interrupt structures.
- **SERCOM multiplexing**: Pin functions are routed through SERCOM peripherals. A board's variant files define which SERCOMs are assigned to SPI, I2C, and UART.
- **Upload via bossac**: Default upload uses the SAM-BA bootloader protocol. Board enters bootloader mode via double-tap reset or 1200-baud touch.
- **USB stack choice**: Boards can select between Arduino's native USB stack and TinyUSB via the `build.usbstack` menu option in `boards.txt`.
- **Submodules**: `Adafruit_TinyUSB_Arduino` and `Adafruit_ZeroDMA` are git submodules — remember to `git submodule update --init` after cloning.
