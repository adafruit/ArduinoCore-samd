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
#include "Arduino.h"
/*
 * Pins descriptions
 */
const PinDescription g_APinDescription[]=
{
  // Fake DAC A0 pin just so we can compile stuff
  { PORTA,  2, PIO_ANALOG, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel0, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_2 }, // A0 / D0 / DAC

  // touch / A1
  { PORTA,  7, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel7, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7 }, // TCC1/WO[1]

  // NeoPixel / D2
  { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

  // I2C SDA D3 & SCL D4
  { PORTA, 4, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // D4 / SDA / PWM
  { PORTA, 5, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // D5 / SCL / PWM

  // USB pins D5, D6, D7
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TC3, TC4, TC5 } ;

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
