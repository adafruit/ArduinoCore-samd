#!/usr/bin/env python3
"""
Generate WirePinMux.inc from SAMD21/DA1 and SAMD51/E5x device tables.

Workflow:
1. Load device-to-package mappings from Tables 2-1, 2-2 (SAMD21/DA1) and 1-1, 1-2 (SAMD51/E5x)
2. Load I2C pin configurations from Tables 7-5 (SAMD21/DA1) and 6-8 (SAMD5x/E5x)
3. For each pin, parse columns C and D to extract primary/alternate SERCOM+PAD
4. Group devices by series and emit guard blocks with I2C_PIN macros

Emits:
    libraries/Wire/WirePinMux.inc

Each pin entry:
    I2C_PIN(PA08, 0, 0, 2, 1)
where arguments are (pin, primary_sercom, primary_pad, alt_sercom, alt_pad).
If an alternate SERCOM is missing, alt_sercom/pad are set to 255.
"""
from __future__ import annotations

import csv
import re
from pathlib import Path
from typing import Dict, List, Set, Tuple

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "extras" / "wirepinmux"
OUTPUT = ROOT / "libraries" / "Wire" / "WirePinMux.inc"

# Device-to-package mapping CSVs
SAMD21_TABLE_2_1 = DATA_DIR / "SAMD21_Table_2-1.csv"
SAMD21_TABLE_2_2 = DATA_DIR / "SAMD21_Table_2-2.csv"
SAMD51_TABLE_1_1 = DATA_DIR / "SAMD51_Table_1-1.csv"
SAMD51_TABLE_1_2 = DATA_DIR / "SAMD51_Table_1-2.csv"

# I2C pin configuration CSVs
SAMD21_TABLE_7_5 = DATA_DIR / "SAMD21_Table_7-5.csv"
SAMD51_TABLE_6_8 = DATA_DIR / "SAMD51_Table_6-8.csv"

# Regex patterns
SERCOM_PAD_RE = re.compile(r"SERCOM(\d+)/PAD\[(\d+)\]")

# Fallback when alt is missing
NO_ALT = (255, 255)


def load_csv(path: Path, skip_rows: int = 0) -> List[Dict[str, str]]:
    """Load CSV file and return list of row dictionaries."""
    with path.open(newline="", encoding="utf-8-sig") as f:  # utf-8-sig removes BOM
        # Skip header rows if needed
        for _ in range(skip_rows):
            next(f)
        return list(csv.DictReader(f))


def parse_sercom_pad(text: str) -> Tuple[int, int] | None:
    """Parse 'SERCOM0/PAD[0]' -> (0, 0), return None if not found."""
    if not text or text.strip() == "":
        return None
    m = SERCOM_PAD_RE.search(text)
    if m:
        return (int(m.group(1)), int(m.group(2)))
    return None


def load_device_pin_map(
    csv_paths: List[Tuple[Path, int]],
) -> Dict[str, Set[str]]:
    """
    Load device-to-package tables and return device -> set of pins mapping.

    For SAMD21/DA1 (Tables 2-1, 2-2): 'Pins' column indicates package size (32, 48, 64).
    For SAMD51/E5x (Tables 1-1, 1-2): 'Pins' column indicates package size (48, 64, 100, 120/128).

    Args:
        csv_paths: List of (path, skip_rows) tuples

    We'll cross-reference with the I2C pin tables (7-5, 6-8) which list pins by package size.
    """
    device_to_pins: Dict[str, Set[str]] = {}

    for csv_path, skip_rows in csv_paths:
        rows = load_csv(csv_path, skip_rows=skip_rows)
        for row in rows:
            device = row.get("Device", "").strip()
            pins_str = row.get("Pins", "").strip()

            if not device or not pins_str:
                continue

            # Extract numeric pin count
            try:
                pin_count = int(pins_str)
            except ValueError:
                continue

            device_to_pins[device] = {str(pin_count)}

    return device_to_pins


def load_i2c_pins_samd21(csv_path: Path) -> Dict[str, List[str]]:
    """Load I2C pin list by package from Table 7-5 (SAMD21/DA1)."""
    package_to_pins: Dict[str, List[str]] = {}

    rows = load_csv(csv_path)
    for row in rows:
        package = row.get("Device", "").strip()
        if not package:
            continue

        pin_tokens: List[str] = []
        for key, val in row.items():
            if key == "Device":
                continue
            if val is None:
                continue
            values = val if isinstance(val, list) else [val]
            for v in values:
                token = v.strip()
                if not token:
                    continue
                pin_tokens.extend(t.strip() for t in token.split(",") if t.strip())

        if not pin_tokens:
            continue

        package_to_pins[package] = pin_tokens

    return package_to_pins


def load_i2c_pins_samd51(csv_path: Path) -> Dict[str, List[str]]:
    """
    Load I2C pin list by package from Table 6-8 (SAMD5x/E5x).

    Each device can have multiple rows (different supplies). Merge pin lists per device:
    - VDDIOB rows (if any) come first
    - then other supplies (e.g., VDDIO)
    Pins are de-duplicated preserving first-seen order.
    Returns: package_size -> [pin_names]
    """
    package_to_pins: Dict[str, List[str]] = {}
    rows = load_csv(csv_path)

    # Collect per-device rows keyed by (device, supply_order)
    per_device: Dict[str, List[Tuple[int, List[str]]]] = {}
    for row in rows:
        device = row.get("Device", "").strip()
        supply = row.get("Supply", "").strip().upper()
        # Gather pin tokens from all remaining columns (some rows spread pins across columns)
        pin_tokens: List[str] = []
        for key, val in row.items():
            if key in {"Device", "Supply"}:
                continue
            if val is None:
                continue
            values = val if isinstance(val, list) else [val]
            for v in values:
                token = v.strip()
                if not token:
                    continue
                pin_tokens.extend(t.strip() for t in token.split(",") if t.strip())

        if not device or not pin_tokens:
            continue
        pin_names = pin_tokens

        # Order supplies: VDDIOB first (0), others after (1)
        supply_order = 0 if supply == "VDDIOB" else 1
        per_device.setdefault(device, []).append((supply_order, pin_names))

    # Merge per device
    for device, pin_lists in per_device.items():
        # Sort by supply order so VDDIOB pins precede
        pin_lists.sort(key=lambda x: x[0])
        merged: List[str] = []
        seen: Set[str] = set()
        for _, pins in pin_lists:
            for pin in pins:
                if pin not in seen:
                    seen.add(pin)
                    merged.append(pin)
        package_to_pins[device] = merged

    return package_to_pins


def load_pin_mux_configs(
    csv_path: Path, is_samd51: bool = False
) -> Dict[str, Tuple[int, int, int, int]]:
    """
    Load pin multiplexing configuration from Table 7-1 (SAMD21) or 6-1 (SAMD51).

    Returns: pin_name -> (primary_sercom, primary_pad, alt_sercom, alt_pad)

    For SAMD21 Table 7-1: columns are organized with column C (primary) and D (alt) for SERCOM
    For SAMD51 Table 6-1: columns C (primary) and D (alt) for SERCOM

    Special case: Some pins only have alternate SERCOM (column D only, no column C).
    For these, we store them with 255,255 as primary and the SERCOM from D as alternate.
    """
    pin_configs: Dict[str, Tuple[int, int, int, int]] = {}

    rows = load_csv(csv_path)
    for row in rows:
        # Pin name is in 'I/O Pin' column for SAMD21, 'Pad Name' for SAMD51
        pin_name = row.get("Pad Name" if is_samd51 else "I/O Pin", "").strip()

        if not pin_name or not pin_name.startswith("P"):
            continue

        # Column C is primary SERCOM, column D is alternate SERCOM
        col_c = row.get("C", "").strip()
        col_d = row.get("D", "").strip()

        primary = parse_sercom_pad(col_c)
        alt = parse_sercom_pad(col_d)

        if primary:
            # Pin has primary SERCOM (column C)
            primary_sercom, primary_pad = primary
            alt_sercom, alt_pad = alt if alt else NO_ALT
            pin_configs[pin_name] = (primary_sercom, primary_pad, alt_sercom, alt_pad)
        elif alt:
            # Pin only has alternate SERCOM (column D only, no column C)
            # Store as: no primary (255,255), alternate from column D
            alt_sercom, alt_pad = alt
            pin_configs[pin_name] = (NO_ALT[0], NO_ALT[1], alt_sercom, alt_pad)

    return pin_configs


def macro_candidates(dev: str) -> List[str]:
    """
    Return the canonical sam.h device macro for a device name.

    sam.h defines exactly one macro per device with pattern __DEVICENAME__
    where DEVICENAME is the datasheet name with "AT" prefix stripped.

    Examples:
        ATSAMD21E18A → __SAMD21E18A__
        ATSAMDA1E14B → __SAMDA1E14B__
        SAMD51J19A → __SAMD51J19A__
        SAME53N20 → __SAME53N20__
    """
    dev_upper = dev.upper()
    # Strip "AT" prefix if present (ATSAMD21E18A → SAMD21E18A)
    if dev_upper.startswith("AT"):
        dev_upper = dev_upper[2:]
    return [f"__{dev_upper}__"]


def classify_device_series(device: str, pin_count: str) -> str:
    """
    Classify device into series based on family and package.

    Returns series macro name like "SAMD21E_SERIES", "SAMD51_120_SERIES", etc.
    """
    dev_upper = device.upper()

    # Remove "AT" prefix if present
    if dev_upper.startswith("AT"):
        dev_upper = dev_upper[2:]

    # SAMD21/DA1 classification by letter (E, G, J)
    if dev_upper.startswith("SAMD21E"):
        return "SAMD21E_SERIES"
    elif dev_upper.startswith("SAMD21G"):
        return "SAMD21G_SERIES"
    elif dev_upper.startswith("SAMD21J"):
        return "SAMD21J_SERIES"
    elif dev_upper.startswith("SAMDA1"):
        return "SAMDA1_SERIES"

    # SAMD51/E5x classification by pin count
    # 120/128-pin packages have 5 SERCOMs (with PD08/PD09)
    # Others have 4 SERCOMs
    pin_num = int(pin_count) if pin_count.isdigit() else 0

    if dev_upper.startswith("SAMD51"):
        if pin_num >= 120:
            return "SAMD51_120_SERIES"
        else:
            return "SAMD51_SERIES"
    elif dev_upper.startswith("SAME51"):
        return "SAME51_SERIES"
    elif dev_upper.startswith("SAME53"):
        return "SAME53_SERIES"
    elif dev_upper.startswith("SAME54"):
        if pin_num >= 120:
            return "SAME54_120_SERIES"
        else:
            return "SAME54_SERIES"

    return "UNKNOWN_SERIES"


def build_series_map() -> (
    Dict[str, Tuple[List[str], List[Tuple[str, Tuple[int, int, int, int]]]]]
):
    """
    Build series -> (devices, pins) mapping.

    Returns: {
        "SAMD21E_SERIES": ([device_names], [(pin, (s0, p0, s1, p1))])
    }
    """
    series_map: Dict[
        str, Tuple[List[str], List[Tuple[str, Tuple[int, int, int, int]]]]
    ] = {}

    # Load device-to-package mappings (with skip_rows for header lines)
    device_pins_samd21 = load_device_pin_map(
        [
            (SAMD21_TABLE_2_1, 2),  # Skip 2 header rows
            (SAMD21_TABLE_2_2, 1),  # Skip 1 header row
        ]
    )
    device_pins_samd51 = load_device_pin_map(
        [
            (SAMD51_TABLE_1_1, 2),  # Skip 2 header rows (table title + subheader)
            (SAMD51_TABLE_1_2, 2),  # Skip 2 header rows (table title + subheader)
        ]
    )

    # Load I2C pin configurations by package
    i2c_pins_samd21 = load_i2c_pins_samd21(SAMD21_TABLE_7_5)
    i2c_pins_samd51 = load_i2c_pins_samd51(SAMD51_TABLE_6_8)

    # Load pin mux configurations (SERCOM mappings from columns C/D)
    pin_mux_samd21 = load_pin_mux_configs(
        ROOT / "docs" / "SAMD21_Table_7-1.csv", is_samd51=False
    )
    pin_mux_samd51 = load_pin_mux_configs(
        ROOT / "docs" / "SAMD51_Table_6-1.csv", is_samd51=True
    )

    # Process SAMD21/DA1 devices
    for device, pin_counts in device_pins_samd21.items():
        pin_count = list(pin_counts)[0]  # Get the pin count
        series = classify_device_series(device, pin_count)

        if series not in series_map:
            series_map[series] = ([], [])

        series_map[series][0].append(device)

        # Get I2C pins for this package size
        if pin_count in i2c_pins_samd21:
            pin_list = i2c_pins_samd21[pin_count]
            # Resolve SERCOM configs from mux table
            resolved_pins = []
            for pin_name in pin_list:
                if pin_name in pin_mux_samd21:
                    resolved_pins.append((pin_name, pin_mux_samd21[pin_name]))

            # Only set pins for the first device in series (all same series share pins)
            if not series_map[series][1]:
                series_map[series] = (series_map[series][0], resolved_pins)

    # Process SAMD51/E5x devices
    for device, pin_counts in device_pins_samd51.items():
        pin_count = list(pin_counts)[0]
        series = classify_device_series(device, pin_count)

        if series not in series_map:
            series_map[series] = ([], [])

        series_map[series][0].append(device)

        # Get I2C pins for this package size
        if pin_count in i2c_pins_samd51:
            pin_list = i2c_pins_samd51[pin_count]
            # Resolve SERCOM configs from mux table
            resolved_pins = []
            for pin_name in pin_list:
                if pin_name in pin_mux_samd51:
                    resolved_pins.append((pin_name, pin_mux_samd51[pin_name]))

            # Only set pins for the first device in series
            if not series_map[series][1]:
                series_map[series] = (series_map[series][0], resolved_pins)

    return series_map


def format_series_macro(series_name: str, device_macros: List[str]) -> List[str]:
    """
    Format a series macro with intelligent grouping.

    For SAMD21, group by numeric suffix (15, 16, 17, 18).
    For SAMD51/E5x, group by letter prefix (G, J, N, P).

    Returns list of lines including #ifndef, #define, content lines, #endif.
    """
    lines: List[str] = []
    lines.append(f"#ifndef {series_name}")

    if len(device_macros) == 1:
        lines.append(f"#define {series_name} (defined({device_macros[0]}))")
    else:
        # Group devices by their variant (numeric for SAMD21, letter for SAMD51/E5x)
        groups: Dict[str, List[str]] = {}

        for macro in sorted(device_macros):
            # Extract the device name from __DEVICENAME__
            device_name = macro.strip("_")

            # For SAMD21/DA1: extract numeric suffix (15, 16, 17, 18, 14, etc.)
            # Examples: SAMD21E15A -> "15", SAMDA1E14B -> "14", SAMD21J18A -> "18"
            # Match: (SAMD21|SAMDA1)([DEGJ]?)(\d+)
            if "SAMD21" in device_name or "SAMDA1" in device_name:
                # Extract the number that appears after the family letter (E, G, J)
                match = re.search(r"(SAMD21|SAMDA1)([DEGJ]?)(\d+)", device_name)
                if match:
                    suffix = match.group(3)  # "15", "16", "14", etc.
                else:
                    suffix = "unknown"
            # For SAMD51/E5x: extract letter prefix (G, J, N, P)
            # Examples: SAMD51G18 -> "G", SAME51J19 -> "J"
            else:
                match = re.search(r"(SAM[DE]\d+)([A-Z])\d+", device_name)
                if match:
                    suffix = match.group(2)  # "G", "J", "N", etc.
                else:
                    suffix = "unknown"

            if suffix not in groups:
                groups[suffix] = []
            groups[suffix].append(macro)

        # Sort groups by their key (numeric or letter order)
        sorted_groups = sorted(groups.items(), key=lambda x: x[0])

        # Build macro output with proper line continuations
        output_lines = []
        for group_idx, (suffix, macros) in enumerate(sorted_groups):
            is_last = group_idx == len(sorted_groups) - 1
            group_line = " || ".join(f"defined({m})" for m in macros)

            if group_idx == 0:
                # First group: start the macro
                if is_last:
                    # Only one group
                    output_lines.append(f"#define {series_name} ({group_line})")
                else:
                    # Multiple groups
                    output_lines.append(f"#define {series_name} ({group_line} || \\")
            else:
                # Continuation lines
                if is_last:
                    # Last group
                    output_lines.append(f"                        {group_line})")
                else:
                    # Middle group
                    output_lines.append(f"                        {group_line} || \\")

        lines.extend(output_lines)

    lines.append("#endif")
    return lines


def emit_inc(
    series_map: Dict[
        str, Tuple[List[str], List[Tuple[str, Tuple[int, int, int, int]]]]
    ],
) -> str:
    """Generate the WirePinMux.inc file content."""
    lines: List[str] = []
    lines.append(
        "// Auto-generated from SAMD21/SAMD51 device tables. Do not edit manually.\n"
    )

    # Define custom ordering for series (moved here to use in sorting)
    series_order = {
        "SAMD21E_SERIES": 0,
        "SAMD21G_SERIES": 1,
        "SAMD21J_SERIES": 2,
        "SAMDA1_SERIES": 3,
        "SAMD51_120_SERIES": 4,
        "SAMD51_SERIES": 5,
        "SAME51_SERIES": 5,
        "SAME53_SERIES": 5,
        "SAME54_SERIES": 5,
        "SAME54_120_SERIES": 6,
    }

    # Emit series macro definitions in custom order
    for series_name in sorted(
        series_map.keys(), key=lambda s: (series_order.get(s, 999), s)
    ):
        devices, _ = series_map[series_name]

        # Build the macro definition
        device_checks = []
        for dev in sorted(set(devices)):
            device_checks.extend(macro_candidates(dev))

        # Remove duplicates while preserving order
        seen = set()
        unique_checks = []
        for check in device_checks:
            if check not in seen:
                seen.add(check)
                unique_checks.append(check)

        # Use the intelligent grouping formatter
        lines.extend(format_series_macro(series_name, unique_checks))
        lines.append("")  # Add blank line between series

    lines.append("")

    # Fallback: if a TARGET_* flag is set but no series macro matches, default to the base series
    lines.append(
        "#if defined(TARGET_SAMD21) && !(SAMD21E_SERIES || SAMD21G_SERIES || SAMD21J_SERIES || SAMDA1_SERIES)"
    )
    lines.append("#define SAMD21E_SERIES 1")
    lines.append("#endif")
    lines.append(
        "#if defined(TARGET_SAMD51) && !(SAMD51_SERIES || SAMD51_120_SERIES || SAME51_SERIES || SAME53_SERIES || SAME54_SERIES || SAME54_120_SERIES)"
    )
    lines.append("#define SAMD51_SERIES 1")
    lines.append("#endif")
    lines.append("")

    # Group series by their pin configuration
    pin_config_groups: Dict[Tuple, List[str]] = {}
    for series_name, (_, pins) in series_map.items():
        pin_tuple = tuple((p, s0, p0, s1, p1) for p, (s0, p0, s1, p1) in pins)
        if pin_tuple not in pin_config_groups:
            pin_config_groups[pin_tuple] = []
        pin_config_groups[pin_tuple].append(series_name)

    # Sort by series order
    sorted_groups = sorted(
        pin_config_groups.items(),
        key=lambda x: (series_order.get(sorted(x[1])[0], 999), sorted(x[1])[0]),
    )

    # Emit pin configurations with guards
    for pin_config, series_list in sorted_groups:
        # Build guard expression
        guard_expr = " || ".join(sorted(series_list))
        lines.append(f"#if {guard_expr}")

        # Emit I2C_PIN macros
        for pin_name, s0, p0, s1, p1 in pin_config:
            lines.append(f"I2C_PIN({pin_name}, {s0}, {p0}, {s1}, {p1})")

        lines.append("#endif\n")

    return "\n".join(lines)


def main() -> None:
    """Main entry point."""
    series_map = build_series_map()
    inc_text = emit_inc(series_map)
    OUTPUT.write_text(inc_text)

    total_devices = sum(len(devices) for devices, _ in series_map.values())
    print(
        f"Wrote {len(series_map)} series macros covering {total_devices} devices to {OUTPUT}"
    )


if __name__ == "__main__":
    main()
