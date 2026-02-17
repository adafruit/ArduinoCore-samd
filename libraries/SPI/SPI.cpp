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

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

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
  
  // Transaction pool initialization
  txnPoolHead = 0;
}

void SPIClass::begin()
{
  if(!initialized) {
    interruptMode = SPI_IMODE_NONE;
    interruptSave = 0;
    interruptMask = 0;
    initialized = true;
  }

#ifdef USE_ZERODMA
  _p_sercom->dmaInit(_p_sercom->getSercomIndex());
#endif

  // PIO init
  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

  config(DEFAULT_SPI_SETTINGS);
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
  // Add DMA deallocation here
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

void SPIClass::transfer(const void *txbuf, void *rxbuf, size_t count,
  bool block, void (*onComplete)(void* user, int status), void* user) {

  if((!txbuf && !rxbuf) || !count) { // Validate inputs
    return;
  }
  // OK to assume now that txbuf and/or rxbuf are non-NULL, an if/else is
  // often sufficient, don't need else-ifs for everything buffer related.

  SercomTxn* txn = allocateTxn();
  txn->txPtr = static_cast<const uint8_t*>(txbuf);
  txn->rxPtr = static_cast<uint8_t*>(rxbuf);
  txn->length = count;
  txn->onComplete = onComplete ? onComplete : &SPIClass::onTxnComplete;
  txn->user = onComplete ? user : this;
  txnDone = false;
  txnStatus = 0;

  if (!_p_sercom->enqueueSPI(txn)) {
    if (onComplete)
      onComplete(user, static_cast<int>(SercomSpiError::UNKNOWN_ERROR));
    return;
  }

  if (!onComplete && block) {
    while (!txnDone) ;
  }
}

void SPIClass::onService(void)
{
  // SPI interrupt service handler - moved from SERCOM::serviceSPI()
  if (!_p_sercom->isActiveSPI() || _p_sercom->getCurrentTxnSPI() == nullptr)
    return;

  uint8_t flags = _p_sercom->getINTFLAG();

  if (flags & SERCOM_SPI_INTFLAG_ERROR)
  {
    _p_sercom->setReturnValueSPI(SercomSpiError::BUF_OVERFLOW);
    _p_sercom->clearINTFLAG();
    _p_sercom->deferStopSPI(SercomSpiError::BUF_OVERFLOW);
    return;
  }

  if (flags & SERCOM_SPI_INTFLAG_RXC) {
    // Read completes after write, so read previous byte
    bool hasMore = _p_sercom->readDataSPI();
    
    if (!hasMore) {
      _p_sercom->disableInterrupts(SERCOM_SPI_INTENCLR_DRE | SERCOM_SPI_INTENCLR_RXC | SERCOM_SPI_INTENCLR_ERROR);
      _p_sercom->setReturnValueSPI(SercomSpiError::SUCCESS);
      _p_sercom->deferStopSPI(SercomSpiError::SUCCESS);
      return;
    }
  }

  if (flags & SERCOM_SPI_INTFLAG_DRE) {
    bool hasMore = _p_sercom->sendDataSPI();
    if (!hasMore)
      _p_sercom->disableInterrupts(SERCOM_SPI_INTENCLR_DRE);
  }
}

SercomTxn* SPIClass::allocateTxn() {
  // Simple round-robin allocation from pool
  SercomTxn* txn = &txnPool[txnPoolHead];
  txnPoolHead = (txnPoolHead + 1) % TXN_POOL_SIZE;
  return txn;
}

void SPIClass::onTxnComplete(void* user, int status)
{
  if (!user)
    return;
  SPIClass* self = static_cast<SPIClass*>(user);
  self->txnStatus = status;
  self->txnDone = true;
}

// Waits for a prior in-background transfer to complete.
void SPIClass::waitForTransfer(void) {
  while(!txnDone);
}

/* returns the current DMA transfer status to allow non-blocking polling */
bool SPIClass::isBusy(void) {
  return !txnDone;
}


// End DMA-based SPI transfer() code ---------------------------------------

void SPIClass::attachInterrupt() {
  // Should be enableInterrupt()
}

void SPIClass::detachInterrupt() {
  // Should be disableInterrupt()
}

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

  #ifndef SPI_IT_HANDLER
    #define SPI_IT_HANDLER      SERCOM4_Handler
  #endif
  void SPI_IT_HANDLER(void) __attribute__ ((weak));
  void SPI_IT_HANDLER(void) { SPI.onService(); }

  #if defined(__SAMD51__)
    #ifndef SPI_IT_HANDLER_0
      #define SPI_IT_HANDLER_0 SERCOM4_0_Handler
      #define SPI_IT_HANDLER_1 SERCOM4_1_Handler
      #define SPI_IT_HANDLER_2 SERCOM4_2_Handler
      #define SPI_IT_HANDLER_3 SERCOM4_3_Handler
    #endif
    void SPI_IT_HANDLER_0(void) __attribute__ ((weak));
    void SPI_IT_HANDLER_1(void) __attribute__ ((weak));
    void SPI_IT_HANDLER_2(void) __attribute__ ((weak));
    void SPI_IT_HANDLER_3(void) __attribute__ ((weak));
    void SPI_IT_HANDLER_0(void) { SPI.onService(); }
    void SPI_IT_HANDLER_1(void) { SPI.onService(); }
    void SPI_IT_HANDLER_2(void) { SPI.onService(); }
    void SPI_IT_HANDLER_3(void) { SPI.onService(); }
  #endif
#endif
#if SPI_INTERFACES_COUNT > 1
  SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI, PAD_SPI1_TX, PAD_SPI1_RX);

  #if defined(SPI1_IT_HANDLER)
    void SPI1_IT_HANDLER(void) __attribute__ ((weak));
    void SPI1_IT_HANDLER(void) { SPI1.onService(); }
  #endif

  #if defined(__SAMD51__) && defined(SPI1_IT_HANDLER_0)
    void SPI1_IT_HANDLER_0(void) __attribute__ ((weak));
    void SPI1_IT_HANDLER_1(void) __attribute__ ((weak));
    void SPI1_IT_HANDLER_2(void) __attribute__ ((weak));
    void SPI1_IT_HANDLER_3(void) __attribute__ ((weak));
    void SPI1_IT_HANDLER_0(void) { SPI1.onService(); }
    void SPI1_IT_HANDLER_1(void) { SPI1.onService(); }
    void SPI1_IT_HANDLER_2(void) { SPI1.onService(); }
    void SPI1_IT_HANDLER_3(void) { SPI1.onService(); }
  #endif
#endif
#if SPI_INTERFACES_COUNT > 2
  SPIClass SPI2(&PERIPH_SPI2, PIN_SPI2_MISO, PIN_SPI2_SCK, PIN_SPI2_MOSI, PAD_SPI2_TX, PAD_SPI2_RX);

  #if defined(SPI2_IT_HANDLER)
    void SPI2_IT_HANDLER(void) { SPI2.onService(); }
  #endif

  #if defined(__SAMD51__) && defined(SPI2_IT_HANDLER_0)
    void SPI2_IT_HANDLER_0(void) { SPI2.onService(); }
    void SPI2_IT_HANDLER_1(void) { SPI2.onService(); }
    void SPI2_IT_HANDLER_2(void) { SPI2.onService(); }
    void SPI2_IT_HANDLER_3(void) { SPI2.onService(); }
  #endif
#endif
#if SPI_INTERFACES_COUNT > 3
  SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_MOSI, PAD_SPI3_TX, PAD_SPI3_RX);

  #if defined(SPI3_IT_HANDLER)
    void SPI3_IT_HANDLER(void) { SPI3.onService(); }
  #endif

  #if defined(__SAMD51__) && defined(SPI3_IT_HANDLER_0)
    void SPI3_IT_HANDLER_0(void) { SPI3.onService(); }
    void SPI3_IT_HANDLER_1(void) { SPI3.onService(); }
    void SPI3_IT_HANDLER_2(void) { SPI3.onService(); }
    void SPI3_IT_HANDLER_3(void) { SPI3.onService(); }
  #endif
#endif
#if SPI_INTERFACES_COUNT > 4
  SPIClass SPI4(&PERIPH_SPI4, PIN_SPI4_MISO, PIN_SPI4_SCK, PIN_SPI4_MOSI, PAD_SPI4_TX, PAD_SPI4_RX);

  #if defined(SPI4_IT_HANDLER)
    void SPI4_IT_HANDLER(void) { SPI4.onService(); }
  #endif

  #if defined(__SAMD51__) && defined(SPI4_IT_HANDLER_0)
    void SPI4_IT_HANDLER_0(void) { SPI4.onService(); }
    void SPI4_IT_HANDLER_1(void) { SPI4.onService(); }
    void SPI4_IT_HANDLER_2(void) { SPI4.onService(); }
    void SPI4_IT_HANDLER_3(void) { SPI4.onService(); }
  #endif
#endif
#if SPI_INTERFACES_COUNT > 5
  SPIClass SPI5(&PERIPH_SPI5, PIN_SPI5_MISO, PIN_SPI5_SCK, PIN_SPI5_MOSI, PAD_SPI5_TX, PAD_SPI5_RX);

  #if defined(SPI5_IT_HANDLER)
    void SPI5_IT_HANDLER(void) { SPI5.onService(); }
  #endif

  #if defined(__SAMD51__) && defined(SPI5_IT_HANDLER_0)
    void SPI5_IT_HANDLER_0(void) { SPI5.onService(); }
    void SPI5_IT_HANDLER_1(void) { SPI5.onService(); }
    void SPI5_IT_HANDLER_2(void) { SPI5.onService(); }
    void SPI5_IT_HANDLER_3(void) { SPI5.onService(); }
  #endif
#endif
