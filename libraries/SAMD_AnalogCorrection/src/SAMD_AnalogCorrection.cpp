/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#include "SAMD_AnalogCorrection.h"

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB (also causes the Arduino builder to link
// the TinyUSB library). Outside PlatformIO this include must stay plain and
// unconditional: the Arduino builder discovers the library from it, and its
// dependency-detection pass cannot parse a __has_include() expression.
#ifdef PLATFORMIO
// PlatformIO does not give framework-bundled libraries the lib_deps include
// paths, so the header can be unreachable here; skip it instead of failing.
#if !defined(__has_include) || __has_include(<Adafruit_TinyUSB.h>)
#include <Adafruit_TinyUSB.h>
#endif
#else
#include <Adafruit_TinyUSB.h>
#endif
#endif

void analogReadCorrection (int offset, uint16_t gain)
{
  Adc *adc;
#if defined (__SAMD51__)
adc = ADC0;
#else
adc = ADC;
#endif
  // Set correction values
  adc->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
  adc->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);

  // Enable digital correction logic
  adc->CTRLB.bit.CORREN = 1;

#if defined (__SAMD51__)
  while(adc->SYNCBUSY.bit.OFFSETCORR || adc->SYNCBUSY.bit.GAINCORR);
#else
  while(adc->STATUS.bit.SYNCBUSY);
#endif
}

