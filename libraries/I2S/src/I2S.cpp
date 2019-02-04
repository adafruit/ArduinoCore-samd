/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

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

#include <Arduino.h>
#include <wiring_private.h>

#include "utility/DMA.h"

#if defined(__SAMD51__)

#include "utility/SAMD51_I2SDevice.h"

static I2SDevice_SAMD51 i2sd(*I2S);

#else

#include "utility/SAMD21_I2SDevice.h"

static I2SDevice_SAMD21G18x i2sd(*I2S);

#endif

#include "I2S.h"

int I2SClass::_beginCount = 0;


I2SClass::I2SClass(uint8_t deviceIndex, uint8_t clockGenerator, uint8_t sdPin, uint8_t sckPin, uint8_t fsPin) :
  _deviceIndex(deviceIndex),
  _clockGenerator(clockGenerator),
  _sdPin(sdPin),
  _sckPin(sckPin),
  _fsPin(fsPin),

  _state(I2S_STATE_IDLE),
  _dmaChannel(-1),
  _bitsPerSample(0),
  _dmaTransferInProgress(false),

  _onTransmit(NULL),
  _onReceive(NULL)
{

}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample)
{
  // master mode (driving clock and frame select pins - output)
  return begin(mode, sampleRate, bitsPerSample, true);
}

int I2SClass::begin(int mode, int bitsPerSample)
{
  // slave mode (not driving clock and frame select pin - input)
  return begin(mode, 0, bitsPerSample, false);
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, bool driveClock)
{
  if (_state != I2S_STATE_IDLE) {
    return 0;
  }

  i2sd.disable();
  switch (mode) {
    case I2S_PHILIPS_MODE:
    case I2S_RIGHT_JUSTIFIED_MODE:
    case I2S_LEFT_JUSTIFIED_MODE:
      break;

    default:
      // invalid mode
      return 0;
  }

  switch (bitsPerSample) {
    case 8:
    case 16:
    case 32:
      _bitsPerSample = bitsPerSample;
      break;

    default:
      // invalid bits per sample
      return 0;
  }

  // try to allocate a DMA channel
  DMA.begin();
  _dmaChannel = DMA.allocateChannel();

  if (_dmaChannel < 0) {
    // no DMA channel available
    return 0;
  }

  if (_beginCount == 0) {
    // enable the I2S interface
#if defined(__SAMD51__)
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_I2S;
#else
    PM->APBCMASK.reg |= PM_APBCMASK_I2S;
#endif

    // reset the device
    i2sd.reset();
  }

  _beginCount++;

  if (driveClock) 
  {
  // set up clock
    enableClock(sampleRate * 2 * _bitsPerSample); // this library assumes 2 channels, stereo audio
    i2sd.setSerialClockSelectMasterClockDiv(_deviceIndex);
    i2sd.setFrameSyncSelectSerialClockDiv(_deviceIndex);
  } else {
    // use input signal from SCK and FS pins
    i2sd.setSerialClockSelectPin(_deviceIndex);
    i2sd.setFrameSyncSelectPin(_deviceIndex);
  }

  // disable device before continuing
  i2sd.disable();

  if (mode == I2S_PHILIPS_MODE) {
    i2sd.set1BitDelay(_deviceIndex);
  } else {
    i2sd.set0BitDelay(_deviceIndex);
  }
  i2sd.setNumberOfSlots(_deviceIndex, 1);
  i2sd.setSlotSize(_deviceIndex, bitsPerSample);
  i2sd.setDataSize(_deviceIndex, bitsPerSample);

#if defined(__SAMD51__)
  pinPeripheral(_sckPin, PIO_I2S);
  pinPeripheral(_fsPin,  PIO_I2S);
#else // SAMD21
  pinPeripheral(_sckPin, PIO_COM);
  pinPeripheral(_fsPin,  PIO_COM);
#endif

 // Serial.println ("mode: " + String(mode));
  if (mode == I2S_RIGHT_JUSTIFIED_MODE) {
    i2sd.setSlotAdjustedRight(_deviceIndex);
  } else {
    i2sd.setSlotAdjustedLeft(_deviceIndex);
  }

  i2sd.setClockUnit(_deviceIndex);

#if defined(__SAMD51__)
  pinPeripheral(_sdPin, PIO_I2S);
#else // SAMD21
  pinPeripheral(_sdPin, PIO_COM);
#endif
/*
  Serial.println("_sdPin:  " + String(_sdPin));
  Serial.println("_sckPin: " + String(_sckPin));
  Serial.println("_fsPin:  " + String(_fsPin));
  while(true);
*/
  // done configure enable
  i2sd.enable();

  _doubleBuffer.reset();

  return 1;
  
}

void I2SClass::end()
{
  if (_dmaChannel > -1) {
    DMA.freeChannel(_dmaChannel);
  }

  _state = I2S_STATE_IDLE;
  _dmaTransferInProgress = false;

  i2sd.disableSerializer(_deviceIndex);
  i2sd.disableClockUnit(_deviceIndex);

  // set the pins back to input mode
  pinMode(_sdPin, INPUT);
  pinMode(_fsPin, INPUT);
  pinMode(_sckPin, INPUT);

  disableClock();

  _beginCount--;

  if (_beginCount == 0) {
    i2sd.disable();

    // disable the I2S interface
#if defined(__SAMD51__)
  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_I2S);
#else
    PM->APBCMASK.reg &= ~PM_APBCMASK_I2S;
#endif
  }
}

int I2SClass::available()
{
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

//  printRegisters();

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t avail;

  // disable interrupts,
  __disable_irq();

  avail = _doubleBuffer.available();

  if (_dmaTransferInProgress == false && _doubleBuffer.available() == 0) {
    // no DMA transfer in progress, start a receive process
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

    // switch to the next buffer for user output (will be empty)
    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return avail;
}

union i2s_sample_t {
  uint8_t b8;
  int16_t b16;
  int32_t b32;
};

int I2SClass::read()
{
  i2s_sample_t sample;
  sample.b32 = 0;

  read(&sample, _bitsPerSample / 8);

  if (_bitsPerSample == 32) {
    return sample.b32;
  } else if (_bitsPerSample == 16) {
    return sample.b16;
  } else if (_bitsPerSample == 8) {
    return sample.b8;
  } else {
    return 0;
  }
}

int I2SClass::peek()
{
  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  i2s_sample_t sample;

  sample.b32 = 0;

  // disable interrupts,
  __disable_irq();

  _doubleBuffer.peek(&sample, _bitsPerSample / 8);

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  if (_bitsPerSample == 32) {
    return sample.b32;
  } else if (_bitsPerSample == 16) {
    return sample.b16;
  } else if (_bitsPerSample == 8) {
    return sample.b8;
  } else {
    return 0;
  }
}

void I2SClass::flush()
{
  // do nothing, writes are DMA triggered
}

size_t I2SClass::write(uint8_t data)
{
  return write((int32_t)data);
}

size_t I2SClass::write(const uint8_t *buffer, size_t size)
{
  return write((const void*)buffer, size);
}

int I2SClass::availableForWrite()
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t space;

  // disable interrupts,
  __disable_irq();

  space = _doubleBuffer.availableForWrite();

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return space;
}
/*/ ---
void I2SClass::read(int32_t *left, int32_t *right)
{
  if (_state != I2S_STATE_RECEIVER)
    enableReceiver();

  i2sd.read(left, right);
}
// ---*/
int I2SClass::read(void* buffer, size_t size)
{
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

//  i2sd.printRegisters();
//  while(true);
  
  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();

  int read = _doubleBuffer.read(buffer, size);

  if (_dmaTransferInProgress == false && _doubleBuffer.available() == 0) {
    // no DMA transfer in progress, start a receive process
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

    // switch to the next buffer for user output (will be empty)
    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return size;
}

size_t I2SClass::write(int sample)
{
  return write((int32_t)sample);
}

size_t I2SClass::write(int32_t sample)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  // this is a blocking write
  while(!i2sd.txReady(_deviceIndex));

  i2sd.writeData(_deviceIndex, sample);

  i2sd.clearTxReady(_deviceIndex);

  return 1;
}

size_t I2SClass::write(const void *buffer, size_t size)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t written;

  // disable interrupts,
  __disable_irq();

  written = _doubleBuffer.write(buffer, size);

  if (_dmaTransferInProgress == false && _doubleBuffer.available()) {
    // no DMA transfer in progress, start a transmit process
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, _doubleBuffer.data(), i2sd.data(_deviceIndex), _doubleBuffer.available());

    // switch to the next buffer for input
    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return written;
}

void I2SClass::onTransmit(void(*function)(void))
{
  _onTransmit = function;
}

void I2SClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void I2SClass::enableClock(int divider)
{
#if defined(__SAMD51__)

  // divider = sampleRate * numChannels * _bitsPerSample  (44100 * 2 * 32)
  uint32_t mckFreq  = (divider / 2 / _bitsPerSample) * 256; // (fs_freq * mck_mult)
  uint32_t sckFreq  = divider; // (fs_freq * I2S_NUM_SLOTS * bitsPerSample)

  uint32_t gclkval  = GCLK_PCHCTRL_GEN_GCLK1_Val;
  uint32_t gclkFreq = VARIANT_GCLK1_FREQ;

  uint8_t mckoutdiv = min( (gclkFreq/mckFreq) - 1, 63);
  uint8_t mckdiv = min( (gclkFreq/sckFreq) - 1, 63 );

  // divider it too big, use 120Mhz oscillator instead?
  if (((VARIANT_GCLK1_FREQ/mckFreq) - 1) > 63) {
    gclkval = GCLK_PCHCTRL_GEN_GCLK4_Val;
    gclkFreq = 12000000; // 120Mhz
  }
/*
Serial.println ("divider: " + String(divider));
  Serial.println ("mckoutdiv: " + String(mckoutdiv));
   Serial.println ("mckdiv: " + String(mckdiv));
  */
  //while (GCLK->SYNCBUSY.reg); 
  GCLK->PCHCTRL[I2S_GCLK_ID_0].reg = gclkval | GCLK_PCHCTRL_CHEN; 
  GCLK->PCHCTRL[I2S_GCLK_ID_1].reg = gclkval | GCLK_PCHCTRL_CHEN;

  i2sd.setMasterClockOutputDivisionFactor(_deviceIndex, mckoutdiv);
  i2sd.setMasterClockDivisionFactor(_deviceIndex, mckdiv);
  i2sd.setMasterClockEnable(_deviceIndex);
  i2sd.setFrameSyncPulse(_deviceIndex, I2S_CLKCTRL_FSWIDTH_HALF_Val);

#else // SAMD21
  int div = SystemCoreClock / divider;
  int src = GCLK_GENCTRL_SRC_DFLL48M_Val;                 // <--- ERROR SAMD51

  if (div > 255) {
    // divider is too big, use 8 MHz oscillator instead
    div = 8000000 / divider;
    src = GCLK_GENCTRL_SRC_OSC8M_Val;                     // <--- ERROR SAMD51
  }

  // configure the clock divider
  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
  GCLK->GENDIV.bit.ID = _clockGenerator;                  // <--- ERROR SAMD51
  GCLK->GENDIV.bit.DIV = div;                             // <--- ERROR SAMD51

  // use the DFLL as the source
  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.ID = _clockGenerator;                 // <--- ERROR SAMD51   
  GCLK->GENCTRL.bit.SRC = src;                            // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.IDC = 1;                              // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.GENEN = 1;                            // <--- ERROR SAMD51

  // enable
  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceIndex);       // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;                // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.CLKEN = 1;                            // <--- ERROR SAMD51

  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
#endif
}

void I2SClass::disableClock()
{
#if defined(__SAMD51__)
  //GCLK->PCHCTRL[I2S_GCLK_ID_0].reg = GCLK_PCHCTRL_RESETVALUE;
  //GCLK->PCHCTRL[I2S_GCLK_ID_1].reg = GCLK_PCHCTRL_RESETVALUE;
#else // SAMD21
  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.ID = _clockGenerator;                 // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;   // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.IDC = 1;                              // <--- ERROR SAMD51
  GCLK->GENCTRL.bit.GENEN = 0;                            // <--- ERROR SAMD51

  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceIndex);       // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;                // <--- ERROR SAMD51
  GCLK->CLKCTRL.bit.CLKEN = 0;                            // <--- ERROR SAMD51

  while (GCLK->STATUS.bit.SYNCBUSY);                      // <--- ERROR SAMD51
#endif
}

void I2SClass::enableTransmitter()
{
#if defined (__SAMD51__)
  i2sd.setTxMode(_deviceIndex, _bitsPerSample);
#else
  i2sd.setTxMode(_deviceIndex);
#endif
  i2sd.enableClockUnit(_deviceIndex);
  i2sd.enableSerializer(_deviceIndex);

  DMA.incSrc(_dmaChannel);
  DMA.onTransferComplete(_dmaChannel, I2SClass::onDmaTransferComplete);
#if defined (__SAMD51__)
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex, true));
#else
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex));
#endif
  DMA.setTransferWidth(_dmaChannel, _bitsPerSample);

  _state = I2S_STATE_TRANSMITTER;
}

void I2SClass::enableReceiver()
{
#if defined (__SAMD51__)
  i2sd.setRxMode(_deviceIndex, _bitsPerSample);
#else
  i2sd.setRxMode(_deviceIndex);
#endif
  i2sd.enableClockUnit(_deviceIndex);
  i2sd.enableSerializer(_deviceIndex);

  DMA.incDst(_dmaChannel);
  DMA.onTransferComplete(_dmaChannel, I2SClass::onDmaTransferComplete);
#if defined (__SAMD51__)
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex, false));
#else
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex));
#endif 
  DMA.setTransferWidth(_dmaChannel, _bitsPerSample);

  _state = I2S_STATE_RECEIVER;
}

void I2SClass::onTransferComplete(void)
{
  if (_state == I2S_STATE_TRANSMITTER) {
    // transmit complete

    if (_doubleBuffer.available()) {
      // output is available to transfer, start the DMA process for the current buffer

      DMA.transfer(_dmaChannel, _doubleBuffer.data(), i2sd.data(_deviceIndex), _doubleBuffer.available());

      // swap to the next user buffer for input
      _doubleBuffer.swap();
    } else {
      // no user data buffered to send
      _dmaTransferInProgress = false;
    }

    // call the users transmit callback if provided
    if (_onTransmit) {
      _onTransmit();
    }
  } else {
    // receive complete

    if (_doubleBuffer.available() == 0) {
      // the user has read all the current input, start the DMA process to fill it again
      DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

      // swap to the next buffer that has previously been filled, so that the user can read it
      _doubleBuffer.swap(_doubleBuffer.availableForWrite());
    } else {
      // user has not read current data, no free buffer to transfer into
      _dmaTransferInProgress = false;
    }

    // call the users receveive callback if provided
    if (_onReceive) {
      _onReceive();
    }
  }
}

void I2SClass::onDmaTransferComplete(int channel)
{
#if I2S_INTERFACES_COUNT > 0
  if (I2S._dmaChannel == channel) {
    I2S.onTransferComplete();
  }
#endif
}


#if defined(__SAMD51__)

  #if I2S_INTERFACES_COUNT > 0
  I2SClass I2S(I2S_DEVICE, I2S_CLOCK_GENERATOR, PIN_I2S_SDI, PIN_I2S_SCK, PIN_I2S_FS);
  #endif

#else 

  #if I2S_INTERFACES_COUNT > 0
  I2SClass I2S(I2S_DEVICE, I2S_CLOCK_GENERATOR, PIN_I2S_SD, PIN_I2S_SCK, PIN_I2S_FS);
  #endif

#endif