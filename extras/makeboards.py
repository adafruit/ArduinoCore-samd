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
        'extra_flags': '-DARDUINO_SAMD_ZERO -DARM_MATH_CM0PLUS'
    },
    
    'SAMD51': {
        'flash_size': 507904, # SAMD51P20A and SAMD51J20A has 1032192
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16'
    },
    
    'SAME51': {
        'flash_size': 507904,
        'data_size': 0,
        'offset': '0x4000',
        'build_mcu': 'cortex-m4',
        'f_cpu': '120000000L',
        'extra_flags': '-D__SAMD51__ -D__FPU_PRESENT -DARM_MATH_CM4 -mfloat-abi=hard -mfpu=fpv4-sp-d16'
    },                    
}


def build_header(mcu, name, vendor, product, vid, pid_list):
    prettyname = vendor + " " + product + " ({})".format(mcu)
    print()
    print("# -----------------------------------")
    print("# {}".format(prettyname))
    print("# -----------------------------------")
    print("{}.name={}".format(name, prettyname))
    print()

    print("# VID/PID for Bootloader, Arduino & CircuitPython")
    for i in range(len(pid_list)):
        print("{}.vid.{}={}".format(name, i, vid))
        print("{}.pid.{}={}".format(name, i, pid_list[i]))
    print()

def build_upload(mcu, name, extra_flags):
    print("# Upload")    
    print("{}.upload.tool=bossac18".format(name))
    print("{}.upload.protocol=sam-ba".format(name))
    
    if ('SAMD51P20A' in extra_flags) or ('SAMD51J20A' in extra_flags):
        flash_size = 1032192
    else:
        flash_size = mcu_dict[mcu]['flash_size']
    print("{}.upload.maximum_size={}".format(name, flash_size))
    #print("{}.upload.maximum_data_size=%d".format((name, mcu_dict[mcu]['data_size']))
    
    print("{}.upload.offset={}".format(name, mcu_dict[mcu]['offset']))    
    print("{}.upload.use_1200bps_touch=true".format(name))
    print("{}.upload.wait_for_upload_port=true".format(name))
    print("{}.upload.native_usb=true".format(name))
    print()

def build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    print("# Build")
    print("{}.build.mcu={}".format(name, mcu_dict[mcu]['build_mcu']))
    print("{}.build.f_cpu={}".format(name, mcu_dict[mcu]['f_cpu']))
    print('{}.build.usb_product="{}"'.format(name, product))
    print('{}.build.usb_manufacturer="{}"'.format(name, vendor))    
    print("{}.build.board={}".format(name, boarddefine))    
    print("{}.build.core=arduino".format(name))    
    print("{}.build.extra_flags={} {} {{build.usb_flags}}".format(name, extra_flags, mcu_dict[mcu]['extra_flags']))
    print("{}.build.ldscript=linker_scripts/gcc/flash_with_bootloader.ld".format(name))    
    print("{}.build.openocdscript=openocd_scripts/{}.cfg".format(name, variant))    
    print("{}.build.variant={}".format(name, variant))
    print("{}.build.variant_system_lib=".format(name))    
    print("{}.build.vid={}".format(name, vid))
    print("{}.build.pid={}".format(name, pid_list[0]))            
    print("{}.bootloader.tool=openocd".format(name))
    print("{}.bootloader.file={}".format(name, bootloader))
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print('{}.compiler.arm.cmsis.ldflags="-L{{runtime.tools.CMSIS-5.4.0.path}}/CMSIS/Lib/GCC/" "-L{{build.variant.path}}" -larm_cortexM4lf_math -mfloat-abi=hard -mfpu=fpv4-sp-d16'.format(name))
    print()
    

def build_menu(mcu, name):
    print("# Menu")
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("{}.menu.cache.on=Enabled".format(name))
        print("{}.menu.cache.on.build.cache_flags=-DENABLE_CACHE".format(name))
        print("{}.menu.cache.off=Disabled".format(name))
        print("{}.menu.cache.off.build.cache_flags=".format(name))
        
        print("{}.menu.speed.120=120 MHz (standard)".format(name))
        print("{}.menu.speed.120.build.f_cpu=120000000L".format(name))
        print("{}.menu.speed.150=150 MHz (overclock)".format(name))
        print("{}.menu.speed.150.build.f_cpu=150000000L".format(name))
        print("{}.menu.speed.180=180 MHz (overclock)".format(name))
        print("{}.menu.speed.180.build.f_cpu=180000000L".format(name))
        print("{}.menu.speed.200=200 MHz (overclock)".format(name))
        print("{}.menu.speed.200.build.f_cpu=200000000L".format(name))
    
    print("{}.menu.opt.small=Small (-Os) (standard)".format(name))
    print("{}.menu.opt.small.build.flags.optimize=-Os".format(name))
    print("{}.menu.opt.fast=Fast (-O2)".format(name))
    print("{}.menu.opt.fast.build.flags.optimize=-O2".format(name))
    print("{}.menu.opt.faster=Faster (-O3)".format(name))
    print("{}.menu.opt.faster.build.flags.optimize=-O3".format(name))
    print("{}.menu.opt.fastest=Fastest (-Ofast)".format(name))
    print("{}.menu.opt.fastest.build.flags.optimize=-Ofast".format(name))
    print("{}.menu.opt.dragons=Here be dragons (-Ofast -funroll-loops)".format(name))
    print("{}.menu.opt.dragons.build.flags.optimize=-Ofast -funroll-loops".format(name))
    
    if (mcu == 'SAMD51' or mcu == 'SAME51'):
        print("{}.menu.maxqspi.50=50 MHz (standard)".format(name))
        print("{}.menu.maxqspi.50.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=50000000".format(name))
        print("{}.menu.maxqspi.fcpu=CPU Speed / 2".format(name))
        print("{}.menu.maxqspi.fcpu.build.flags.maxqspi=-DVARIANT_QSPI_BAUD_DEFAULT=({{build.f_cpu}})".format(name))

    print("{}.menu.usbstack.arduino=Arduino".format(name))
    print("{}.menu.usbstack.tinyusb=TinyUSB".format(name))
    print("{}.menu.usbstack.tinyusb.build.flags.usbstack=-DUSE_TINYUSB".format(name))

    print("{}.menu.debug.off=Off".format(name))
    print("{}.menu.debug.on=On".format(name))
    print("{}.menu.debug.on.build.flags.debug=-g".format(name))
    print()

def build_global_menu():
    print("menu.cache=Cache")
    print("menu.speed=CPU Speed")
    print("menu.opt=Optimize")
    print("menu.maxqspi=Max QSPI")    
    print("menu.usbstack=USB Stack")
    print("menu.debug=Debug")

def make_board(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader):
    build_header(mcu, name, vendor, product, vid, pid_list)
    build_upload(mcu, name, extra_flags)
    build_build(mcu, name, variant, vendor, product, vid, pid_list, boarddefine, extra_flags, bootloader)    
    build_menu(mcu, name)

build_global_menu()

######################## SAMD21

make_board("SAMD21", "adafruit_feather_m0", "feather_m0", 
           "Adafruit", "Feather M0", "0x239A", ["0x800B", "0x000B", "0x0015"],
           "SAMD_ZERO", "-D__SAMD21G18A__ -DADAFRUIT_FEATHER_M0", "featherM0/bootloader-feather_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_feather_m0_express", "feather_m0_express", 
           "Adafruit", "Feather M0 Express", "0x239A", ["0x801B", "0x001B"],
           "SAMD_FEATHER_M0_EXPRESS", "-D__SAMD21G18A__ -DARDUINO_SAMD_FEATHER_M0 -DADAFRUIT_FEATHER_M0_EXPRESS", "featherM0/bootloader-feather_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_metro_m0", "metro_m0", 
           "Adafruit", "Metro M0 Express", "0x239A", ["0x8013", "0x0013"],
           "SAMD_ZERO", "-D__SAMD21G18A__ -DADAFRUIT_METRO_M0_EXPRESS", "metroM0/bootloader-metro_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_circuitplayground_m0", "circuitplay", 
           "Adafruit", "Circuit Playground Express", "0x239A", ["0x8018", "0x0019"],
           "SAMD_CIRCUITPLAYGROUND_EXPRESS", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_CIRCUITPLAYGROUND_M0", "circuitplayM0/bootloader-circuitplay_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_gemma_m0", "gemma_m0", 
           "Adafruit", "Gemma M0", "0x239A", ["0x801C", "0x001C"],
           "GEMMA_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_GEMMA_M0", "gemmaM0/bootloader-gemma_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_trinket_m0", "trinket_m0", 
           "Adafruit", "Trinket M0", "0x239A", ["0x801E", "0x001E"],
           "TRINKET_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_TRINKET_M0", "trinketm0/bootloader-trinket_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_qtpy_m0", "qtpy_m0", 
           "Adafruit", "QT Py M0", "0x239A", ["0x80CB", "0x00CB", "0x00CC"],
           "QTPY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_QTPY_M0", "qtpyM0/bootloader-qtpy_m0.bin")

make_board("SAMD21", "adafruit_neotrinkey_m0", "neotrinkey_m0", 
           "Adafruit", "NeoPixel Trinkey M0", "0x239A", ["0x80EF", "0x00EF", "0x80F0"],
           "NEOTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_NEOTRINKEY_M0", "neotrinkey_m0/bootloader-neotrinkey_m0.bin")

make_board("SAMD21", "adafruit_rotarytrinkey_m0", "rotarytrinkey_m0", 
           "Adafruit", "Rotary Trinkey M0", "0x239A", ["0x80FB", "0x00FB", "0x80FC"],
           "ROTARYTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_ROTARYTRINKEY_M0", "rotarytrinkey_m0/bootloader-rotarytrinkey_m0.bin")

make_board("SAMD21", "adafruit_neokeytrinkey_m0", "neokeytrinkey_m0", 
           "Adafruit", "NeoKey Trinkey M0", "0x239A", ["0x80FF", "0x00FF", "0x8100"],
           "NEOKEYTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_NEOKEYTRINKEY_M0", "neokeytrinkey_m0/bootloader-neokeytrinkey_m0.bin")

make_board("SAMD21", "adafruit_slidetrinkey_m0", "slidetrinkey_m0", 
           "Adafruit", "Slide Trinkey M0", "0x239A", ["0x8101", "0x0101", "0x8102"],
           "SLIDETRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_SLIDETRINKEY_M0", "slidetrinkey_m0/bootloader-slidetrinkey_m0.bin")

make_board("SAMD21", "adafruit_proxlighttrinkey_m0", "proxlighttrinkey_m0", 
           "Adafruit", "ProxLight Trinkey M0", "0x239A", ["0x8103", "0x0103", "0x8104"],
           "PROXLIGHTTRINKEY_M0", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_PROXLIGHTTRINKEY_M0", "proxlighttrinkey_m0/bootloader-proxlighttrinkey_m0.bin")

make_board("SAMD21", "adafruit_itsybitsy_m0", "itsybitsy_m0", 
           "Adafruit", "ItsyBitsy M0 Express", "0x239A", ["0x800F", "0x000F", "0x8012"],
           "ITSYBITSY_M0", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_ITSYBITSY_M0", "itsybitsyM0/bootloader-itsybitsy_m0-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_pirkey", "pirkey", 
           "Adafruit", "pIRKey", "0x239A", ["0x801E", "0x001E"],
           "PIRKEY", "-D__SAMD21E18A__ -DCRYSTALLESS -DADAFRUIT_PIRKEY", "pirkey/bootloader-pirkey-v2.0.0-adafruit.5.bin")

make_board("SAMD21", "adafruit_hallowing", "hallowing_m0_express", 
           "Adafruit", "Hallowing M0", "0x239A", ["0xDEAD", "0xD1ED", "0xB000"],
           "SAMD_HALLOWING", "-D__SAMD21G18A__ -DCRYSTALLESS -DARDUINO_SAMD_HALLOWING_M0 -DADAFRUIT_HALLOWING", "hallowingM0/bootloader-hallowing_m0-v2.0.0-adafruit.0-21-g887cc30.bin")

make_board("SAMD21", "adafruit_crickit_m0", "crickit_m0", 
           "Adafruit", "Crickit M0", "0x239A", ["0x802D", "0x002D", "0x802D"],
           "CRICKIT_M0", "-D__SAMD21G18A__ -DCRYSTALLESS -DADAFRUIT_CRICKIT_M0", "crickit/samd21_sam_ba.bin")

make_board("SAMD21", "adafruit_blm_badge", "blm_badge", 
           "Adafruit", "BLM Badge", "0x239A", ["0x80BF", "0x00BF", "0x80C0"],
           "BLM_BADGE_M0", "-D__SAMD21E18A__ -DCRYSTALLESS  -DADAFRUIT_BLM_BADGE", "blmbadge/bootloader-blm_badge.bin")

######################## SAMD51

make_board("SAMD51", "adafruit_metro_m4", "metro_m4", 
           "Adafruit", "Metro M4", "0x239A", ["0x8020", "0x0020", "0x8021", "0x0021"],
           "METRO_M4", "-D__SAMD51J19A__ -DADAFRUIT_METRO_M4_EXPRESS", "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_grandcentral_m4", "grand_central_m4", 
           "Adafruit", "Grand Central M4", "0x239A", ["0x8031", "0x0031", "0x0032"],
           "GRAND_CENTRAL_M4", "-D__SAMD51P20A__ -DADAFRUIT_GRAND_CENTRAL_M4", "grand_central_m4/bootloader-grandcentral_m4.bin")

make_board("SAMD51", "adafruit_itsybitsy_m4", "itsybitsy_m4", 
           "Adafruit", "ItsyBitsy M4", "0x239A", ["0x802B", "0x002B"],
           "ITSYBITSY_M4", "-D__SAMD51G19A__ -DCRYSTALLESS -DADAFRUIT_ITSYBITSY_M4_EXPRESS", "itsybitsyM4/bootloader-itsybitsy_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_feather_m4", "feather_m4", 
           "Adafruit", "Feather M4 Express", "0x239A", ["0x8022", "0x0022", "0x8026"],
           "FEATHER_M4", "-D__SAMD51J19A__ -DADAFRUIT_FEATHER_M4_EXPRESS", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAME51", "adafruit_feather_m4_can", "feather_m4_can", 
           "Adafruit", "Feather M4 CAN", "0x239A", ["0x80CD", "0x00CD"],
           "FEATHER_M4_CAN", "-D__SAME51J19A__ -DADAFRUIT_FEATHER_M4_EXPRESS -DADAFRUIT_FEATHER_M4_CAN", "featherM4/bootloader-feather_m4_express-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_trellis_m4", "trellis_m4", 
           "Adafruit", "Trellis M4", "0x239A", ["0x802F", "0x002F", "0x0030"],
           "TRELLIS_M4", "-D__SAMD51G19A__ -DCRYSTALLESS -DADAFRUIT_TRELLIS_M4_EXPRESS", "trellisM4/bootloader-trellis_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_pyportal_m4", "pyportal_m4", 
           "Adafruit", "PyPortal M4", "0x239A", ["0x8035", "0x0035", "0x8036"],
           "PYPORTAL_M4", "-D__SAMD51J20A__ -DCRYSTALLESS -DADAFRUIT_PYPORTAL", "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_pyportal_m4_titano", "pyportal_m4_titano", 
           "Adafruit", "PyPortal M4 Titano", "0x239A", ["0x8053", "0x8053"],
           "PYPORTAL_M4_TITANO", "-D__SAMD51J20A__ -DCRYSTALLESS -DADAFRUIT_PYPORTAL_M4_TITANO", "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_pybadge_m4", "pybadge_m4", 
           "Adafruit", "pyBadge M4 Express", "0x239A", ["0x8033", "0x0033", "0x8034", "0x0034"],
           "PYBADGE_M4", "-D__SAMD51J19A__ -DCRYSTALLESS -DADAFRUIT_PYBADGE_M4_EXPRESS", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_metro_m4_airliftlite", "metro_m4_airlift", 
           "Adafruit", "Metro M4 AirLift Lite", "0x239A", ["0x8037", "0x0037"],
           "METRO_M4_AIRLIFT_LITE", "-D__SAMD51J19A__ -DADAFRUIT_METRO_M4_AIRLIFT_LITE", "metroM4/bootloader-metro_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_pygamer_m4", "pygamer_m4", 
           "Adafruit", "PyGamer M4 Express", "0x239A", ["0x803D", "0x003D", "0x803E"],
           "PYGAMER_M4", "-D__SAMD51J19A__ -DCRYSTALLESS  -DADAFRUIT_PYGAMER_M4_EXPRESS", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_pybadge_airlift_m4", "pybadge_airlift_m4", 
           "Adafruit", "pyBadge AirLift M4", "0x239A", ["0x8043", "0x0043", "0x8044"],
           "PYBADGE_AIRLIFT_M4", "-D__SAMD51J20A__ -DCRYSTALLESS  -DADAFRUIT_PYBADGE_AIRLIFT_M4", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_monster_m4sk", "monster_m4sk", 
           "Adafruit", "MONSTER M4SK", "0x239A", ["0x8047", "0x0047", "0x8048"],
           "MONSTER_M4SK", "-D__SAMD51G19A__ -DCRYSTALLESS  -DADAFRUIT_MONSTER_M4SK_EXPRESS", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_hallowing_m4", "hallowing_m4", 
           "Adafruit", "Hallowing M4", "0x239A", ["0x8049", "0x0049", "0x804A"],
           "HALLOWING_M4", "-D__SAMD51J19A__ -DCRYSTALLESS  -DADAFRUIT_HALLOWING_M4_EXPRESS", "featherM4/bootloader-feather_m4-v2.0.0-adafruit.5.bin")

make_board("SAMD51", "adafruit_matrixportal_m4", "matrixportal_m4", 
           "Adafruit", "Matrix Portal M4", "0x239A", ["0x80C9", "0x00C9", "0x80CA"],
           "MATRIXPORTAL_M4", "-D__SAMD51J19A__ -DCRYSTALLESS  -DADAFRUIT_MATRIXPORTAL_M4_EXPRESS", "matrixportalM4/bootloader-matrixportal_m4.bin")
