/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_XPLAINED_M4_
#define _VARIANT_XPLAINED_M4_

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC (32768ul)

/** Master clock frequency */
#define VARIANT_MCK (120000000ul)

#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT (60u)
#define NUM_DIGITAL_PINS (47u)
#define NUM_ANALOG_INPUTS (16u)
#define NUM_ANALOG_OUTPUTS (1u)
#define analogInputToDigitalPin(p) ((p < NUM_ANALOG_INPUTS) ? g_AAnalogPinMap[(p)] : -1)

extern const uint8_t g_AAnalogPinMap[NUM_ANALOG_INPUTS];
#if defined(__SAME53__) || defined(__SAME54__)
#define digitalPinToPort(P)        ( &(PORT_REGS->GROUP[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
// #define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &((port)->PORT_OUT) )
#define portInputRegister(port)    ( &((port)->PORT_IN) )
#define portModeRegister(port)     ( &((port)->PORT_DIR) )
#else
#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
// #define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &((port)->OUT.reg) )
#define portInputRegister(port)    ( &((port)->IN.reg) )
#define portModeRegister(port)     ( &((port)->DIR.reg) )
#endif // __SAME53__ / __SAME54__
#define digitalPinHasPWM(P) (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LEDs
#define PIN_LED (46u)
#define LED_BUILTIN PIN_LED
#define PIN_BUTTON (59u)

/*
 * Analog pins
 */
#define PIN_A0 (43u)
#define PIN_A1 (12u)
#define PIN_A2 (13u)
#define PIN_A3 (28u)
#define PIN_A4 (29u)
#define PIN_A5 (41u)
#define PIN_A6 (42u)
#define PIN_A7 (2u)
#define PIN_A8 (3u)
#define PIN_A9 (4u)
#define PIN_A10 (5u)
#define PIN_A11 (6u)
#define PIN_A12 (18u)
#define PIN_A13 (19u)
#define PIN_A14 (22u)
#define PIN_A15 (34u)

#define PIN_DAC0 PIN_A0

  static const uint8_t A0 = PIN_A0;
  static const uint8_t A1 = PIN_A1;
  static const uint8_t A2 = PIN_A2;
  static const uint8_t A3 = PIN_A3;
  static const uint8_t A4 = PIN_A4;
  static const uint8_t A5 = PIN_A5;
  static const uint8_t A6 = PIN_A6;
  static const uint8_t A7 = PIN_A7;
  static const uint8_t A8 = PIN_A8;
  static const uint8_t A9 = PIN_A9;
  static const uint8_t A10 = PIN_A10;
  static const uint8_t A11 = PIN_A11;
  static const uint8_t A12 = PIN_A12;
  static const uint8_t A13 = PIN_A13;
  static const uint8_t A14 = PIN_A14;
  static const uint8_t A15 = PIN_A15;

  static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION 12

// Other pins
#define PIN_EXT1_SDA (14u)
#define PIN_EXT1_SCL (15u)
#define PIN_EXT2_SDA (30u)
#define PIN_EXT2_SCL (31u)
#define PIN_EXT3_SDA PIN_EXT2_SDA
#define PIN_EXT3_SCL PIN_EXT2_SCL

/*
 * Serial interfaces
 */

// Numbered Arduino UARTs: EXT1, EXT2, EXT3, then the EDBG virtual COM port.
#define PIN_SERIAL1_RX (0u)
#define PIN_SERIAL1_TX (1u)
#define PAD_SERIAL1_RX SERCOM_RX_PAD_1
#define PAD_SERIAL1_TX UART_TX_PAD_0

#define PIN_SERIAL2_RX (16u)
#define PIN_SERIAL2_TX (17u)
#define PAD_SERIAL2_RX SERCOM_RX_PAD_1
#define PAD_SERIAL2_TX UART_TX_PAD_0

#define PIN_SERIAL3_RX (32u)
#define PIN_SERIAL3_TX (33u)
#define PAD_SERIAL3_RX SERCOM_RX_PAD_1
#define PAD_SERIAL3_TX UART_TX_PAD_0

#define PIN_SERIAL4_RX (47u)
#define PIN_SERIAL4_TX (48u)
#define PAD_SERIAL4_RX SERCOM_RX_PAD_1
#define PAD_SERIAL4_TX UART_TX_PAD_0

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO (10u)
#define PIN_SPI_MOSI (9u)
#define PIN_SPI_SCK (11u)
#define PIN_SPI_SS (8u)
#define PERIPH_SPI sercom4
#define PAD_SPI_TX SPI_PAD_0_SCK_1
#define PAD_SPI_RX SERCOM_RX_PAD_3
#define SPI_IT_HANDLER_0 SERCOM4_0_Handler
#define SPI_IT_HANDLER_1 SERCOM4_1_Handler
#define SPI_IT_HANDLER_2 SERCOM4_2_Handler
#define SPI_IT_HANDLER_3 SERCOM4_3_Handler

  static const uint8_t SS = PIN_SPI_SS;
  static const uint8_t MOSI = PIN_SPI_MOSI;
  static const uint8_t MISO = PIN_SPI_MISO;
  static const uint8_t SCK = PIN_SPI_SCK;

#define PIN_SPI1_MISO (26u)
#define PIN_SPI1_MOSI (25u)
#define PIN_SPI1_SCK (27u)
#define PIN_SPI1_SS (24u)
#define PERIPH_SPI1 sercom6
#define PAD_SPI1_TX SPI_PAD_0_SCK_1
#define PAD_SPI1_RX SERCOM_RX_PAD_3
#define SPI1_IT_HANDLER_0 SERCOM6_0_Handler
#define SPI1_IT_HANDLER_1 SERCOM6_1_Handler
#define SPI1_IT_HANDLER_2 SERCOM6_2_Handler
#define SPI1_IT_HANDLER_3 SERCOM6_3_Handler

  static const uint8_t SS1 = PIN_SPI1_SS;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA PIN_EXT1_SDA
#define PIN_WIRE_SCL PIN_EXT1_SCL
#define PERIPH_WIRE sercom3
#define WIRE_IT_HANDLER SERCOM3_Handler
#define WIRE_IT_HANDLER_0 SERCOM3_0_Handler
#define WIRE_IT_HANDLER_1 SERCOM3_1_Handler
#define WIRE_IT_HANDLER_2 SERCOM3_2_Handler
#define WIRE_IT_HANDLER_3 SERCOM3_3_Handler

  static const uint8_t SDA = PIN_WIRE_SDA;
  static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA PIN_EXT2_SDA
#define PIN_WIRE1_SCL PIN_EXT2_SCL
#define PERIPH_WIRE1 sercom7
#define WIRE1_IT_HANDLER SERCOM7_Handler
#define WIRE1_IT_HANDLER_0 SERCOM7_0_Handler
#define WIRE1_IT_HANDLER_1 SERCOM7_1_Handler
#define WIRE1_IT_HANDLER_2 SERCOM7_2_Handler
#define WIRE1_IT_HANDLER_3 SERCOM7_3_Handler

  static const uint8_t SDA1 = PIN_WIRE1_SDA;
  static const uint8_t SCL1 = PIN_WIRE1_SCL;

/*
 * USB
 */
#define PIN_USB_VBUS (49u)
#define PIN_USB_HOST_ENABLE (50u)
#define PIN_USB_DM (51u)
#define PIN_USB_DP (52u)

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE 0
#define I2S_CLOCK_GENERATOR 3

#define PIN_I2S_SDO (14u)
#define PIN_I2S_SDI (15u)
#define PIN_I2S_SCK PIN_SERIAL2_TX
#define PIN_I2S_FS (33u)
#define PIN_I2S_MCK PIN_SERIAL2_RX
// Onboard Micron N25Q256A QSPI flash routes.
#define PIN_QSPI_SCK (53u)
#define PIN_QSPI_CS (54u)
#define PIN_QSPI_IO0 (55u)
#define PIN_QSPI_IO1 (56u)
#define PIN_QSPI_IO2 (57u)
#define PIN_QSPI_IO3 (58u)

  // TODO: meaningful value for this
  // #define VARIANT_QSPI_BAUD_DEFAULT 5000000

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
 */
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern SERCOM sercom6;
extern SERCOM sercom7;

extern Uart Serial1;
extern Uart Serial2;
extern Uart Serial3;
extern Uart Serial4;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL Serial
#define SERIAL_PORT_MONITOR Serial
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE Serial1
#define SERIAL_PORT_HARDWARE_OPEN Serial1

#endif /* _VARIANT_XPLAINED_M4_ */
