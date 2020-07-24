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

#include "variant.h"

/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // 0..13 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTB, 12, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 },  // TX SERCOM 4.0
  { PORTB, 13, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 },  // RX SERCOM 4.1

  // 2..12
  // Digital Low
  { PORTB,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel15, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 }, // D2
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel14, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // D3
  { PORTA,  14, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH0, TC3_CH0, EXTERNAL_INT_14 }, // D4
  { PORTA,  16, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0  }, // D5
  { PORTA, 17, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // D6 
  { PORTB,  15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, TC5_CH0, EXTERNAL_INT_15 }, // D7 (LISIRQ)

  // Digital High

  { PORTB,  16, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH4, NOT_ON_TIMER, EXTERNAL_INT_0 }, // D8 (NEOPIX)
  { PORTA,  18, PIO_SERCOM, PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2  }, // D9
  { PORTA,  19, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH1, TC3_CH1,  EXTERNAL_INT_3 }, // D10
  { PORTA,  20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER,  EXTERNAL_INT_4 }, // D11
  { PORTA,  21, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER,  EXTERNAL_INT_5 }, // D12

  // 13 (LED)
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH3, TC4_CH1, EXTERNAL_INT_7 }, 


  // 14..23 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // A0 (DAC0)
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // A1 (DAC1)
  { PORTA,  6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // A2
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // A3
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // A4
  { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // A5

  { PORTB,  1, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },  // A6, D20 - vbatt
  { PORTB,  4, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },  // A7, D21 - Light
  { PORTB,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel15, NOT_ON_PWM,  NOT_ON_TIMER, EXTERNAL_INT_2 }, // A8 / D2
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel14, NOT_ON_PWM,  NOT_ON_TIMER, EXTERNAL_INT_2 }, // A9 / D3

  // 24..25 I2C pins (SDA/SCL)
  // ----------------------
  { PORTA,  12, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0,  EXTERNAL_INT_12 }, // SDA
  { PORTA,  13, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1,  EXTERNAL_INT_13 }, // SCL


  // 26..28 - SPI pins (MISO,MOSI,SCK)
  // ----------------------
  { PORTB, 22, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // MISO: SERCOM5.2
  { PORTB, 23, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // MOSI: SERCOM5.3
  { PORTA, 22, PIO_SERCOM_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH2, TC4_CH0, EXTERNAL_INT_6 }, // SCK: SERCOM 5.1

  // 29..31 - USB
  // --------------------
  { NOT_A_PORT, PIN_NOT_A_PIN, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable DOES NOT EXIST ON THIS BOARD
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 32 (AREF)
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 33..34 - Alternate use of A0 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VOUT0
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VOUT1

  // ----------------------
  // 35..40 QSPI (SCK, CS, IO0, IO1, IO2, IO3)
  { PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 8, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 9, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
  { PORTA, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },


  // 41..46 - TFT SPI port + control pins
  // --------------------
  { PORTA,  0, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_0 },         // D41 TFT MOSI (SERCOM1.0)
  { PORTA,  1, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_1 },         // D42 TFT SCK (SERCOM1.1)
  { NOT_A_PORT, PIN_NOT_A_PIN, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // D43 TFT MISO (unused)
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },  // D44 TFT CS
  { PORTB, 31, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, // D45 TFT DC
  { PORTB, 30, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, // D46 TFT RST

  { PORTB, 14, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_15 },  // D47 Backlight (not connected rev B)


  { PORTA, 15, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15  }, // D48 - Cap Pin
  { PORTB,  5, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },  // D49 Speaker enable

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom4, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;

void SERCOM4_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM4_3_Handler()
{
  Serial1.IrqHandler();
}