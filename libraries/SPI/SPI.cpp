/*
 * SPI Master library for Arduino Zero.
 * Copyright (c) 2015 Arduino LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "SPI.h"
#include <Arduino.h>
#include <wiring_private.h>
#include <assert.h>

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

SPIClass::SPIClass(SERCOM *p_sercom, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, SercomSpiTXPad PadTx, SercomRXPad PadRx)
{
  initialized = false;
  assert(p_sercom != NULL);
  _p_sercom = p_sercom;

  // pins
  _uc_pinMiso = uc_pinMISO;
  _uc_pinSCK = uc_pinSCK;
  _uc_pinMosi = uc_pinMOSI;

  // SERCOM pads
  _padTx=PadTx;
  _padRx=PadRx;
}

void SPIClass::begin()
{
  init();

  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

  config(DEFAULT_SPI_SETTINGS);
}

void SPIClass::init()
{
  if (initialized)
    return;
  interruptMode = SPI_IMODE_NONE;
  interruptSave = 0;
  interruptMask = 0;
  initialized = true;
}

void SPIClass::config(SPISettings settings)
{
  _p_sercom->disableSPI();

  _p_sercom->initSPI(_padTx, _padRx, SPI_CHAR_SIZE_8_BITS, settings.bitOrder);
  _p_sercom->initSPIClock(settings.dataMode, settings.clockFreq);

  _p_sercom->enableSPI();
}

void SPIClass::end()
{
  _p_sercom->resetSPI();
  initialized = false;
}

#ifndef interruptsStatus
#define interruptsStatus() __interruptsStatus()
static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
static inline unsigned char __interruptsStatus(void)
{
  // See http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/CHDBIBGJ.html
  return (__get_PRIMASK() ? 0 : 1);
}
#endif

void SPIClass::usingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EXTERNAL_INT_NMI))
    return;

  uint8_t irestore = interruptsStatus();
  noInterrupts();

  if (interruptNumber >= EXTERNAL_NUM_INTERRUPTS)
    interruptMode = SPI_IMODE_GLOBAL;
  else
  {
    interruptMode |= SPI_IMODE_EXTINT;
    interruptMask |= (1 << g_APinDescription[interruptNumber].ulExtInt);
  }

  if (irestore)
    interrupts();
}

void SPIClass::notUsingInterrupt(int interruptNumber)
{
  if ((interruptNumber == NOT_AN_INTERRUPT) || (interruptNumber == EXTERNAL_INT_NMI))
    return;

  if (interruptMode & SPI_IMODE_GLOBAL)
    return; // can't go back, as there is no reference count

  uint8_t irestore = interruptsStatus();
  noInterrupts();

  interruptMask &= ~(1 << g_APinDescription[interruptNumber].ulExtInt);

  if (interruptMask == 0)
    interruptMode = SPI_IMODE_NONE;

  if (irestore)
    interrupts();
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      interruptSave = interruptsStatus();
      noInterrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
  }

  config(settings);
}

void SPIClass::endTransaction(void)
{
  if (interruptMode != SPI_IMODE_NONE)
  {
    if (interruptMode & SPI_IMODE_GLOBAL)
    {
      if (interruptSave)
        interrupts();
    }
    else if (interruptMode & SPI_IMODE_EXTINT)
      EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
  }
}

void SPIClass::setBitOrder(BitOrder order)
{
  if (order == LSBFIRST) {
    _p_sercom->setDataOrderSPI(LSB_FIRST);
  } else {
    _p_sercom->setDataOrderSPI(MSB_FIRST);
  }
}

void SPIClass::setDataMode(uint8_t mode)
{
  switch (mode)
  {
    case SPI_MODE0:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_0);
      break;

    case SPI_MODE1:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_1);
      break;

    case SPI_MODE2:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_2);
      break;

    case SPI_MODE3:
      _p_sercom->setClockModeSPI(SERCOM_SPI_MODE_3);
      break;

    default:
      break;
  }
}

void SPIClass::setClockDivider(uint8_t div)
{
  if(div < SPI_MIN_CLOCK_DIVIDER) {
    _p_sercom->setBaudrateSPI(SPI_MIN_CLOCK_DIVIDER);
  } else {
    _p_sercom->setBaudrateSPI(div);
  }
}

byte SPIClass::transfer(uint8_t data)
{
  return _p_sercom->transferDataSPI(data);
}

uint16_t SPIClass::transfer16(uint16_t data) {
  union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

  t.val = data;

  if (_p_sercom->getDataOrderSPI() == LSB_FIRST) {
    t.lsb = transfer(t.lsb);
    t.msb = transfer(t.msb);
  } else {
    t.msb = transfer(t.msb);
    t.lsb = transfer(t.lsb);
  }

  return t.val;
}

void SPIClass::transfer(void *buf, size_t count)
{
  uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
  for (size_t i=0; i<count; i++) {
    *buffer = transfer(*buffer);
    buffer++;
  }
}

// Non-DMA transfer function.
// this was removed from the dma function and made its own
void SPIClass::transfer(void* txbuf, void* rxbuf, size_t count)
{
    uint8_t *txbuf8 = (uint8_t *)txbuf,
            *rxbuf8 = (uint8_t *)rxbuf;
    if(rxbuf8) {
        if(txbuf8) {
            // Writing and reading simultaneously
            while(count--) {
                *rxbuf8++ = _p_sercom->transferDataSPI(*txbuf8++);
            }
        } else {
            // Reading only
            while(count--) {
                *rxbuf8++ = _p_sercom->transferDataSPI(0xFF);
            }
        }
    } else if(txbuf) {
        // Writing only
        while(count--) {
            (void)_p_sercom->transferDataSPI(*txbuf8++);
        }
    }
}

// Pointer to SPIClass object, one per DMA channel.
static SPIClass *spiPtr[DMAC_CH_NUM] = { 0 }; // Legit inits list to NULL

// dma callback when the read part is completed
void SPIClass::dmaCallback_read(Adafruit_ZeroDMA *dma)
{
	// dmaCallback() receives an Adafruit_ZeroDMA object. From this we can get
	// a channel number (0 to DMAC_CH_NUM-1, always unique per ZeroDMA object),
	// then locate the originating SPIClass object using array lookup, setting
	uint8_t channel = dma->getChannel();

	// flag this part of the dma done
	spiPtr[channel]->dma_read_done = true;

	// see if the entire transaction is completed
	checkDmaComplete(channel);
}

// dma callback when the write part is completed
void SPIClass::dmaCallback_write(Adafruit_ZeroDMA *dma)
{
	// dmaCallback() receives an Adafruit_ZeroDMA object. From this we can get
	// a channel number (0 to DMAC_CH_NUM-1, always unique per ZeroDMA object),
	// then locate the originating SPIClass object using array lookup, setting
	uint8_t channel = dma->getChannel();

	// flag this part of the dma done
	spiPtr[channel]->dma_write_done = true;

	// see if the entire transaction is completed
	checkDmaComplete(channel);

}

// see if the entire dma transaction is completed
// will automatically initiate another dma if we have bytes remaining to transfer
void SPIClass::checkDmaComplete(uint8_t channel)
{
	// read and write dmas are both done
	if(spiPtr[channel]->dma_read_done && spiPtr[channel]->dma_write_done)
	{
		// are more bytes that need to be transfered
		// fire another dma transaction
		if( (spiPtr[channel]->dma_bytes_remaining) > 0)
		{
			// initiate another transfer for the next section of bytes
			// update buffer pointers offsets
			// use the same user callback as last time if any
			void* txbuf = spiPtr[channel]->txbuf_last + DMA_MAX_TRANSFER_SIZE;
			void* rxbuf = spiPtr[channel]->rxbuf_last + DMA_MAX_TRANSFER_SIZE;
			spiPtr[channel]->transfer(txbuf, rxbuf, spiPtr[channel]->dma_bytes_remaining, spiPtr[channel]->userDmaCallback );
		}
		// the transfer is completed, no bytes remaining
		else
		{
			// flag as completed for anything poling for completion
			spiPtr[channel]->dma_complete = true;

			// call the callback function the user specified if any
			if(spiPtr[channel]->userDmaCallback != NULL)
			{
				spiPtr[channel]->userDmaCallback();
			}

		}

	}
}

// dma transfer function for spi with poll for completion
void SPIClass::transfer(const void* txbuf, void* rxbuf, uint32_t count, bool block)
{
	// start the dma transfer, but do not specify a user callback function, will poll for completion instead
	transfer(txbuf, rxbuf, count, NULL);

	// if this function should automatically wait for completion, otherwise user must do manually
	if(block)
	{
		waitForTransfer();
	}
}

// Waits for a prior in-background DMA transfer to complete.
void SPIClass::waitForTransfer(void)
{
    while( !dma_complete )
    {
    	// do nothing, wait for transfer completion
    }
}

// dma transfer function for spi
// this function does not block, and dma will transfer in the background
// the callback parameter should be passed in by the user, it is called when the dma is done
void SPIClass::transfer(void* txbuf, void* rxbuf, uint32_t count, void (*functionToCallWhenComplete)(void) )
{
	// remember these buffer pointers
	// will reuse if we have to do multiple dma transactions and pointer math
	txbuf_last = txbuf;
	rxbuf_last = rxbuf;

	// save this function to call when the entire dma is done
	userDmaCallback = functionToCallWhenComplete;

	// Maximum bytes per DMA descriptor is 65,535 (NOT 65,536).
	// We could set up a descriptor chain, but that gets more
	// complex. For now, instead, break up long transfers into
	// chunks of 65,535 bytes max...these transfers are all
	// blocking, regardless of the "block" argument, except
	// for the last one which will observe the background request.
	// The fractional part is done first, so for any "partially
	// blocking" transfers like these at least it's the largest
	// single-descriptor transfer possible that occurs in the
	// background, rather than the tail end.
	uint16_t  bytesThisPass;

	if(count > DMA_MAX_TRANSFER_SIZE)
	{
		// Too big for 1 descriptor
		// will need to do multiple dma transfers
		bytesThisPass = DMA_MAX_TRANSFER_SIZE;

		// remember bytes remaining for future transfers
		dma_bytes_remaining = count - DMA_MAX_TRANSFER_SIZE;
	}
	else
	{
		// can do everything in one dma transfer
		bytesThisPass 		= count;
		dma_bytes_remaining = 0;
	}

	//******************************
    // If the RX DMA channel is not yet allocated...
    if(readChannel.getChannel() >= DMAC_CH_NUM)
    {
        if(readChannel.allocate() == DMA_STATUS_OK)
        {
            readDescriptor = readChannel.addDescriptor(
                (void *)getDataRegister(), // Source address (SPI data reg)
				rxbuf,                     // Dest address
				bytesThisPass,             // bytes to transfer
                DMA_BEAT_SIZE_BYTE,        // Bytes/hwords/words
                false,                     // Don't increment source address
                true);                     // Increment dest address
            readChannel.setTrigger(getDMAC_ID_RX());
            readChannel.setAction(DMA_TRIGGER_ACTON_BEAT);
            readChannel.setCallback(dmaCallback_read, DMA_CALLBACK_TRANSFER_DONE);
            spiPtr[readChannel.getChannel()] = this;
        }
    }
    else
    {
    	// update to use the currently passed buffers
    	readChannel.changeDescriptor(
    			readDescriptor,
				(void *)getDataRegister(),	// Source address (SPI data reg)
				rxbuf, 						// Dest address
				bytesThisPass);				// bytes to transfer
    }

    // If the TX DMA channel is not yet allocated...
    if(writeChannel.getChannel() >= DMAC_CH_NUM)
    {
        if(writeChannel.allocate() == DMA_STATUS_OK)
        {
            writeDescriptor = writeChannel.addDescriptor(
                txbuf,                     // Source address
                (void *)getDataRegister(), // Dest (SPI data register)
				bytesThisPass,             // bytes to transfer
                DMA_BEAT_SIZE_BYTE,        // Bytes/hwords/words
                true,                      // Increment source address
                false);                    // Don't increment dest address
            writeChannel.setTrigger(getDMAC_ID_TX());
            writeChannel.setAction(DMA_TRIGGER_ACTON_BEAT);
            writeChannel.setCallback(dmaCallback_write, DMA_CALLBACK_TRANSFER_DONE);
            spiPtr[writeChannel.getChannel()] = this;
        }
    }
    else
    {
    	// update to use the currently passed buffers
    	writeChannel.changeDescriptor(
    			writeDescriptor,
				txbuf,						// Source address
				(void *)getDataRegister(),	// Dest (SPI data register)
				bytesThisPass);				// bytes to transfer
    }

    //******************************
    // clear the flags
	dma_read_done  = false;
	dma_write_done = false;
	dma_complete   = false;

    // fire the dma transactions
	readChannel.startJob();
	writeChannel.startJob();
}

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

// SPI DMA lookup works on both SAMD21 and SAMD51

static const struct {
  volatile uint32_t *data_reg;
  int                dmac_id_tx;
  int                dmac_id_rx;
} sercomData[] = {
  { &SERCOM0->SPI.DATA.reg, SERCOM0_DMAC_ID_TX, SERCOM0_DMAC_ID_RX },
  { &SERCOM1->SPI.DATA.reg, SERCOM1_DMAC_ID_TX, SERCOM1_DMAC_ID_RX },
  { &SERCOM2->SPI.DATA.reg, SERCOM2_DMAC_ID_TX, SERCOM2_DMAC_ID_RX },
  { &SERCOM3->SPI.DATA.reg, SERCOM3_DMAC_ID_TX, SERCOM3_DMAC_ID_RX },
#if defined(SERCOM4)
  { &SERCOM4->SPI.DATA.reg, SERCOM4_DMAC_ID_TX, SERCOM4_DMAC_ID_RX },
#endif
#if defined(SERCOM5)
  { &SERCOM5->SPI.DATA.reg, SERCOM5_DMAC_ID_TX, SERCOM5_DMAC_ID_RX },
#endif
#if defined(SERCOM6)
  { &SERCOM6->SPI.DATA.reg, SERCOM6_DMAC_ID_TX, SERCOM6_DMAC_ID_RX },
#endif
#if defined(SERCOM7)
  { &SERCOM7->SPI.DATA.reg, SERCOM7_DMAC_ID_TX, SERCOM7_DMAC_ID_RX },
#endif
};

volatile uint32_t *SPIClass::getDataRegister(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].data_reg: NULL;
}

int SPIClass::getDMAC_ID_TX(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].dmac_id_tx : -1;
}

int SPIClass::getDMAC_ID_RX(void) {
  int8_t idx = _p_sercom->getSercomIndex();
  return (idx >= 0) ? sercomData[idx].dmac_id_rx : -1;
}

#if defined(__SAMD51__)

// Set the SPI device's SERCOM clock CORE and SLOW clock sources.
// SercomClockSource values are an enumeration in SERCOM.h.
// This works on SAMD51 only.  On SAMD21, a dummy function is declared
// in SPI.h which compiles to nothing, so user code doesn't need to check
// and conditionally compile lines for different architectures.
void SPIClass::setClockSource(SercomClockSource clk) {
  int8_t idx = _p_sercom->getSercomIndex();
  _p_sercom->setClockSource(idx, clk, true);  // true  = set core clock
  _p_sercom->setClockSource(idx, clk, false); // false = set slow clock
}

#endif // end __SAMD51__

#if SPI_INTERFACES_COUNT > 0
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   *
   * The SPI PAD values can be found in cores/arduino/SERCOM.h:
   *   - SercomSpiTXPad
   *   - SercomRXPad
   */
  #ifndef PERIPH_SPI
    #define PERIPH_SPI           sercom4
    #define PAD_SPI_TX           SPI_PAD_2_SCK_3
    #define PAD_SPI_RX           SERCOM_RX_PAD_0
  #endif // PERIPH_SPI
  SPIClass SPI (&PERIPH_SPI,  PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI,  PAD_SPI_TX,  PAD_SPI_RX);
#endif
#if SPI_INTERFACES_COUNT > 1
  SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);
#endif
#if SPI_INTERFACES_COUNT > 2
  SPIClass SPI2(&PERIPH_SPI2, PIN_SPI2_MISO, PIN_SPI2_SCK, PIN_SPI2_MOSI, PAD_SPI2_TX, PAD_SPI2_RX);
#endif
#if SPI_INTERFACES_COUNT > 3
  SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_MOSI, PAD_SPI3_TX, PAD_SPI3_RX);
#endif
#if SPI_INTERFACES_COUNT > 4
  SPIClass SPI4(&PERIPH_SPI4, PIN_SPI4_MISO, PIN_SPI4_SCK, PIN_SPI4_MOSI, PAD_SPI4_TX, PAD_SPI4_RX);
#endif
#if SPI_INTERFACES_COUNT > 5
  SPIClass SPI5(&PERIPH_SPI5, PIN_SPI5_MISO, PIN_SPI5_SCK, PIN_SPI5_MOSI, PAD_SPI5_TX, PAD_SPI5_RX);
#endif

