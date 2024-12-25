#!/usr/bin/env python3

print('''# Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
''')

mcu_dict = {
    'SAMD21': {
        'flash_size': 262144,
        'data_size': 0,
        'offset': '0x2000',
        'build_mcu': 'cortex-m0plus',
        'f_cpu': '48000000L',
        'extra_flags': '-DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS',
        'openocdscript': 'scripts/openocd/daplink_samd21.cfg',
    },

    'SAMD51': {
        'flash_size': 507904, # SAMD51P20A and SAMD51J20A has 1032192
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16',
        'openocdscript': 'scripts/openocd/daplink_samd51.cfg',
    },

    'SAME51': {
        'flash_size': 507904,
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16',
        'openocdscript': 'scripts/openocd/daplink_samd51.cfg',
    },
}


def build_header(mcu, name, vendor, product, vid, pid_list):
    prettyname = f"{vendor} {product} ({mcu})"
    print()
    print("# -----------------------------------")
    print(f"# {prettyname}")
    print("# -----------------------------------")
    print(f"{name}.name={prettyname}")
    print()

    print("# VID/PID for Bootloader, Arduino & CircuitPython")
    for i in range(len(pid_list)):
        print(f"{name}.vid.{i}={vid}")
        print(f"{name}.pid.{i}={pid_list[i]}")
    print()


def build_upload(mcu, name, extra_flags):
    print("# Upload")    
    print(f"{name}.upload.tool=bossac18")
    print(f"{name}.upload.protocol=sam-ba")
    
    if ('SAMD51P20A' in extra_flags) or ('SAMD51J20A' in extra_flags):
        flash_size = 1032192
    else:
        flash_size = mcu_dict[mcu]['flash_size']
    print(f"{name}.upload.maximum_size={flash_size}")
    #print(f"{name}.upload.maximum_data_size={mcu_dict[mcu]['data_size']}")
    
    print(f"{name}.upload.offset={mcu_dict[mcu]['offset']}")
    print(f"{name}.upload.use_1200bps_touch=true")
    print(f"{name}.upload.wait_for_upload_port=true")
    print(f"{name}.upload.native_usb=true")
    print()


def build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    mcu_properties = mcu_dict[mcu]

    print("# Build")
    print(f"{name}.build.mcu={mcu_properties['build_mcu']}")
    print(f"{name}.build.f_cpu={mcu_properties['f_cpu']}")
    print(f'{name}.build.usb_product="{product}"')
    print(f'{name}.build.usb_manufacturer="{vendor}"')
    print(f"{name}.build.board={boarddefine}")
    print(f"{name}.build.core=arduino")

    # Due to fastLed issue https://github.com/FastLED/FastLED/issues/1363
    # although there is a simple fix already https://github.com/FastLED/FastLED/pull/1424
    # fastLED is not well maintained, and we need to skip ARDUINO_SAMD_ZERO for affected boards
    # in the long run we should move all of our libraries away from ARDUINO_SAMD_ZERO
    if variant in [ 'gemma_m0', 'trinket_m0', 'qtpy_m0', 'itsybitsy_m0' ]:
        print(f"{name}.build.extra_flags={extra_flags} -DARM_MATH_CM0PLUS {{build.usb_flags}}")
    else:
        print(f"{name}.build.extra_flags={extra_flags} {mcu_properties['extra_flags']} {{build.usb_flags}}")

    print(f"{name}.build.ldscript=linker_scripts/gcc/flash_with_bootloader.ld")
    print(f"{name}.build.openocdscript={mcu_properties['openocdscript']}")
    print(f"{name}.build.variant={variant}")
    print(f"{name}.build.variant_system_lib=")
    print(f"{name}.build.vid={vid}")
    print(f"{name}.build.pid={pid_list[0]}")
    print(f"{name}.bootloader.tool=openocd")
    print(f"{name}.bootloader.file={bootloader}")
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print(f'{name}.compiler.arm.cmsis.ldflags="-L{{runtime.tools.CMSIS-5.4.0.path}}/CMSIS/Lib/GCC/" "-L{{build.variant.path}}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16')
    print()
    

def build_menu(mcu, name):
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("# Menu: Cache")
        print(f"{name}.menu.cache.on=Enabled")
        print(f"{name}.menu.cache.on.build.cache_flags=-DENABLE_CACHE")
        print(f"{name}.menu.cache.off=Disabled")
        print(f"{name}.menu.cache.off.build.cache_flags=")
        print()

        print("# Menu: Speed")
        print(f"{name}.menu.speed.120=120 MHz (standard)")
        print(f"{name}.menu.speed.120.build.f_cpu=120000000L")
        print(f"{name}.menu.speed.150=150 MHz (overclock)")
        print(f"{name}.menu.speed.150.build.f_cpu=150000000L")
        print(f"{name}.menu.speed.180=180 MHz (overclock)")
        print(f"{name}.menu.speed.180.build.f_cpu=180000000L")
        print(f"{name}.menu.speed.200=200 MHz (overclock)")
        print(f"{name}.menu.speed.200.build.f_cpu=200000000L")
        print()

    print("# Menu: Optimization")
    print(f"{name}.menu.opt.small=Small (-Os) (standard)")
    print(f"{name}.menu.opt.small.build.flags.optimize=-Os")
    print(f"{name}.menu.opt.fast=Fast (-O2)")
    print(f"{name}.menu.opt.fast.build.flags.optimize=-O2")
    print(f"{name}.menu.opt.faster=Faster (-O3)")
    print(f"{name}.menu.opt.faster.build.flags.optimize=-O3")
    print(f"{name}.menu.opt.fastest=Fastest (-Ofast)")
    print(f"{name}.menu.opt.fastest.build.flags.optimize=-Ofast")
    print(f"{name}.menu.opt.dragons=Here be dragons (-Ofast -funroll-loops)")
    print(f"{name}.menu.opt.dragons.build.flags.optimize=-Ofast -funroll-loops")
    print()
    
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("# Menu: QSPI Speed")
        print(f"{name}.menu.maxqspi.50=50 MHz (standard)")
        print(f"{name}.menu.maxqspi.50.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=50000000")
        print(f"{name}.menu.maxqspi.fcpu=CPU Speed / 2")
        print(f"{name}.menu.maxqspi.fcpu.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=({{build.f_cpu}})")
        print()

    print("# Menu: USB Stack")
    print(f"{name}.menu.usbstack.arduino=Arduino")
    print(f"{name}.menu.usbstack.tinyusb=TinyUSB")
    print(f"{name}.menu.usbstack.tinyusb.build.flags.usbstack=-DUSE_TINYUSB")
    print()

    print("# Menu: Debug")
    print(f"{name}.menu.debug.off=Off")
    print(f"{name}.menu.debug.on=On")
    print(f"{name}.menu.debug.on.build.flags.debug=-g")
    print()

    # comment out for now since debugger selection does not work, debug does not pickup the right openocd script
    # print("# Menu: Debugger")
    # script_mcu = 'samd21' if mcu == 'SAMD21' else 'samd51'
    # print(f"{name}.menu.debugger.daplink=CMSIS-DAP (DAPLink)")
    # print(f"{name}.menu.debugger.daplink.build.openocdscript=scripts/openocd/daplink_{script_mcu}.cfg")
    # print(f"{name}.menu.debugger.jlink=J-Link")
    # print(f"{name}.menu.debugger.jlink.build.openocdscript=scripts/openocd/jlink_{script_mcu}.cfg")


def build_global_menu():
    print("menu.cache=Cache")
    print("menu.speed=CPU Speed")
    print("menu.opt=Optimize")
    print("menu.maxqspi=Max QSPI")    
    print("menu.usbstack=USB Stack")
    print("menu.debug=Debug")
    #print("menu.debugger=Debugger")


def make_board(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    build_header(mcu, name, vendor, product, vid, pid_list)
    build_upload(mcu, name, extra_flags)
    build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader)    
    build_menu(mcu, name)


# ------------------------------
# main
# ------------------------------

build_global_menu()

# ------------------------------
# SAM D21 (M0)
# ------------------------------

# name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader
# try to sort in Alphabetical order
d21_board_list = [
    ["adafruit_feather_m0", "feather_m0", "Adafruit", "Feather M0",
     "0x239A", ["0x800B", "0x000B", "0x0015"],
     "SAMD_ZERO", "-D__SAMD21G18A__ -DADAFRUIT_FEATHER_M0",
     "featherM0/bootloader-feather_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_feather_m0_express", "feather_m0_express", "Adafruit", "Feather M0 Express",
     "0x239A", ["0x801B", "0x001B"],
     "SAMD_FEATHER_M0_EXPRESS", "-D__SAMD21G18A__ -DARDUINO_SAMD_FEATHER_M0 -DADAFRUIT_FEATHER_M0_EXPRESS",
     "featherM0/bootloader-feather_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_metro_m0", "metro_m0", "Adafruit", "Metro M0 Express",
     "0x239A", ["0x8013", "0x0013"],
     "SAMD_ZERO", "-D__SAMD21G18A__ -DADAFRUIT_METRO_M0_EXPRESS",
     "metroM0/bootloader-metro_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_circuitplayground_m0", "circuitplay", "Adafruit", "Circuit Playground Express",
     "0x239A", ["0x8018", "0x0019"],
     "SAMD_CIRCUITPLAYGROUND_EXPRESS", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_CIRCUITPLAYGROUND_M0",
     "circuitplayM0/bootloader-circuitplay_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_gemma_m0", "gemma_m0", "Adafruit", "Gemma M0",
     "0x239A", ["0x801C", "0x001C"],
     "GEMMA_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_GEMMA_M0",
     "gemmaM0/bootloader-gemma_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_trinket_m0", "trinket_m0", "Adafruit", "Trinket M0",
     "0x239A", ["0x801E", "0x001E"],
     "TRINKET_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0",
     "trinketm0/bootloader-trinket_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_qtpy_m0", "qtpy_m0", "Adafruit", "QT Py M0",
     "0x239A", ["0x80CB", "0x00CB", "0x00CC"],
     "QTPY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_QTPY_M0",
     "qtpyM0/bootloader-qtpy_m0.bin"],

    ["adafruit_neotrinkey_m0", "neotrinkey_m0", "Adafruit", "NeoPixel Trinkey M0",
     "0x239A", ["0x80EF", "0x00EF", "0x80F0"],
     "NEOTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_NEOTRINKEY_M0",
     "neotrinkey_m0/bootloader-neotrinkey_m0.bin"],

    ["adafruit_rotarytrinkey_m0", "rotarytrinkey_m0", "Adafruit", "Rotary Trinkey M0",
     "0x239A", ["0x80FB", "0x00FB", "0x80FC"],
     "ROTARYTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_ROTARYTRINKEY_M0",
     "rotarytrinkey_m0/bootloader-rotarytrinkey_m0.bin"],

    ["adafruit_neokeytrinkey_m0", "neokeytrinkey_m0", "Adafruit", "NeoKey Trinkey M0",
     "0x239A", ["0x80FF", "0x00FF", "0x8100"],
     "NEOKEYTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_NEOKEYTRINKEY_M0",
     "neokeytrinkey_m0/bootloader-neokeytrinkey_m0.bin"],

    ["adafruit_slidetrinkey_m0", "slidetrinkey_m0", "Adafruit", "Slide Trinkey M0",
     "0x239A", ["0x8101", "0x0101", "0x8102"],
     "SLIDETRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_SLIDETRINKEY_M0",
     "slidetrinkey_m0/bootloader-slidetrinkey_m0.bin"],

    ["adafruit_proxlighttrinkey_m0", "proxlighttrinkey_m0", "Adafruit", "ProxLight Trinkey M0",
     "0x239A", ["0x8103", "0x0103", "0x8104"],
     "PROXLIGHTTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_PROXLIGHTTRINKEY_M0",
     "proxlighttrinkey_m0/bootloader-proxlighttrinkey_m0.bin"],

    ["adafruit_sht4xtrinkey_m0", "sht4xtrinkey_m0", "Adafruit", "SHT4x Trinkey M0",
     "0x239A", ["0x8153", "0x0153", "0x8154"],
     "SHT4XTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_SHT4XTRINKEY_M0",
     "sht4xtrinkey_m0/bootloader-sht4xtrinkey_m0.bin"],

    ["adafruit_pixeltrinkey_m0", "pixeltrinkey_m0", "Adafruit", "Pixel Trinkey M0",
     "0x239A", ["0x8155", "0x0155", "0x8156"],
     "PIXELTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_PIXELTRINKEY_M0",
     "pixeltrinkey_m0/bootloader-pixeltrinkey_m0.bin"],

    ["adafruit_TRRStrinkey_m0", "trrstrinkey_m0", "Adafruit", "TRRS Trinkey M0",
     "0x239A", ["0x8157", "0x0157", "0x8158"],
     "TRRSTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRRSTRINKEY_M0",
     "trrstrinkey_m0/bootloader-TRRStrinkey_m0.bin"],

    ["adafruit_thumbsticktrinkey_m0", "thumbsticktrinkey_m0", "Adafruit", "Thumbstick Trinkey M0",
     "0x239A", ["0x8159", "0x0159", "0x8160"],
     "THUMBSTICKTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_THUMBSTICKTRINKEY_M0",
     "thumbsticktrinkey_m0/bootloader-thumbsticktrinkey_m0.bin"],

    ["adafruit_itsybitsy_m0", "itsybitsy_m0", "Adafruit", "ItsyBitsy M0 Express",
     "0x239A", ["0x800F", "0x000F", "0x8012"],
     "ITSYBITSY_M0", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_ITSYBITSY_M0",
     "itsybitsyM0/bootloader-itsybitsy_m0-v2.0.0-adafruit.5.bin"],

    ["adafruit_pirkey", "pirkey", "Adafruit", "pIRKey",
     "0x239A", ["0x8027", "0x0027", "0x8028"],
     "PIRKEY", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_PIRKEY",
     "pirkey/bootloader-pirkey-v2.0.0-adafruit.5.bin"],

    ["adafruit_hallowing", "hallowing_m0_express", "Adafruit", "Hallowing M0",
     "0x239A", ["0xDEAD", "0xD1ED", "0xB000"],
     "SAMD_HALLOWING", "-D__SAMD21G18A__ -DCRYSTALLESS -DARDUINO_SAMD_HALLOWING_M0 -DADAFRUIT_HALLOWING",
     "hallowingM0/bootloader-hallowing_m0-v2.0.0-adafruit.0-21-g887cc30.bin"],

    ["adafruit_crickit_m0", "crickit_m0", "Adafruit", "Crickit M0",
     "0x239A", ["0x802D", "0x002D", "0x802D"],
     "CRICKIT_M0", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_CRICKIT_M0",
     "crickit/samd21_sam_ba.bin"],

    ["adafruit_blm_badge", "blm_badge", "Adafruit", "BLM Badge",
     "0x239A", ["0x80BF", "0x00BF", "0x80C0"],
     "BLM_BADGE_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_BLM_BADGE",
     "blmbadge/bootloader-blm_badge.bin"],
]

for b in d21_board_list:
    make_board("SAMD21", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8])


# ----------------------------
# SAM D51 and E51 (M4)
# ----------------------------

d51_board_list = [
    ["adafruit_metro_m4", "metro_m4", "Adafruit", "Metro M4",
     "0x239A", ["0x8020", "0x0020", "0x8021", "0x0021"],
     "METRO_M4", "-D__SAMD51J19A__ -DADAFRUIT_METRO_M4_EXPRESS",
     "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_grandcentral_m4", "grand_central_m4", "Adafruit", "Grand Central M4",
     "0x239A", ["0x8031", "0x0031", "0x0032"],
     "GRAND_CENTRAL_M4", "-D__SAMD51P20A__ -DADAFRUIT_GRAND_CENTRAL_M4",
     "grand_central_m4/bootloader-grandcentral_m4.bin"],

    ["adafruit_itsybitsy_m4", "itsybitsy_m4", "Adafruit", "ItsyBitsy M4",
     "0x239A", ["0x802B", "0x002B"],
     "ITSYBITSY_M4", "-D__SAMD51G19A__ -DCRYSTALLESS -DADAFRUIT_ITSYBITSY_M4_EXPRESS",
     "itsybitsyM4/bootloader-itsybitsy_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_feather_m4", "feather_m4", "Adafruit", "Feather M4 Express",
     "0x239A", ["0x8022", "0x0022", "0x8026"],
     "FEATHER_M4", "-D__SAMD51J19A__ -DADAFRUIT_FEATHER_M4_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_feather_m4_can", "feather_m4_can", "Adafruit", "Feather M4 CAN",
     "0x239A", ["0x80CD", "0x00CD"],
     "FEATHER_M4_CAN", "-D__SAME51J19A__ -DADAFRUIT_FEATHER_M4_EXPRESS -DADAFRUIT_FEATHER_M4_CAN",
     "featherM4/bootloader-feather_m4_express-v2.0.0-adafruit.5.bin"],

    # ["adafruit_feather_m4_log", "feather_m4_log", "Adafruit", "Feather M4 Adalogger",
    #  "0x239A", ["0x8115", "0x0115", "0x8116"],
    #  "FEATHER_M4_ADALOGGER", "-D__SAME51J19A__ -DADAFRUIT_FEATHER_M4_ADALOGGER",
    #  "feather_m4_log/bootloader-feather_m4_log-v2.0.0-adafruit.5.bin"],

    ["adafruit_trellis_m4", "trellis_m4",
     "Adafruit", "Trellis M4", "0x239A", ["0x802F", "0x002F", "0x0030"],
     "TRELLIS_M4", "-D__SAMD51G19A__ -DCRYSTALLESS -DADAFRUIT_TRELLIS_M4_EXPRESS",
     "trellisM4/bootloader-trellis_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_pyportal_m4", "pyportal_m4", "Adafruit", "PyPortal M4",
     "0x239A", ["0x8035", "0x0035", "0x8036"],
     "PYPORTAL_M4", "-D__SAMD51J20A__ -DCRYSTALLESS -DADAFRUIT_PYPORTAL",
     "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_pyportal_m4_titano", "pyportal_m4_titano", "Adafruit", "PyPortal M4 Titano",
     "0x239A", ["0x8053", "0x8053"],
     "PYPORTAL_M4_TITANO", "-D__SAMD51J20A__ -DCRYSTALLESS -DADAFRUIT_PYPORTAL_M4_TITANO",
     "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_pybadge_m4", "pybadge_m4", "Adafruit", "pyBadge M4 Express",
     "0x239A", ["0x8033", "0x0033", "0x8034", "0x0034"],
     "PYBADGE_M4", "-D__SAMD51J19A__ -DCRYSTALLESS -DADAFRUIT_PYBADGE_M4_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_metro_m4_airliftlite", "metro_m4_airlift", "Adafruit", "Metro M4 AirLift Lite",
     "0x239A", ["0x8037", "0x0037"],
     "METRO_M4_AIRLIFT_LITE", "-D__SAMD51J19A__ -DADAFRUIT_METRO_M4_AIRLIFT_LITE",
     "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_pygamer_m4", "pygamer_m4", "Adafruit", "PyGamer M4 Express",
     "0x239A", ["0x803D", "0x003D", "0x803E"],
     "PYGAMER_M4", "-D__SAMD51J19A__ -DCRYSTALLESS -DADAFRUIT_PYGAMER_M4_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_pybadge_airlift_m4", "pybadge_airlift_m4", "Adafruit", "pyBadge AirLift M4",
     "0x239A", ["0x8043", "0x0043", "0x8044"],
     "PYBADGE_AIRLIFT_M4", "-D__SAMD51J20A__ -DCRYSTALLESS -DADAFRUIT_PYBADGE_AIRLIFT_M4",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_monster_m4sk", "monster_m4sk", "Adafruit", "MONSTER M4SK",
     "0x239A", ["0x8047", "0x0047", "0x8048"],
     "MONSTER_M4SK", "-D__SAMD51G19A__ -DCRYSTALLESS -DADAFRUIT_MONSTER_M4SK_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_hallowing_m4", "hallowing_m4", "Adafruit", "Hallowing M4",
     "0x239A", ["0x8049", "0x0049", "0x804A"],
     "HALLOWING_M4", "-D__SAMD51J19A__ -DCRYSTALLESS -DADAFRUIT_HALLOWING_M4_EXPRESS",
     "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin"],

    ["adafruit_matrixportal_m4", "matrixportal_m4", "Adafruit", "Matrix Portal M4",
     "0x239A", ["0x80C9", "0x00C9", "0x80CA"],
     "MATRIXPORTAL_M4", "-D__SAMD51J19A__ -DCRYSTALLESS  -DADAFRUIT_MATRIXPORTAL_M4_EXPRESS",
     "matrixportalM4/bootloader-matrixportal_m4.bin"],
]

for b in d51_board_list:
    # M4 CAN is the only SAME51
    if b[0] == "adafruit_feather_m4_can":
        make_board("SAME51", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8])
    else:
        make_board("SAMD51", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8])
