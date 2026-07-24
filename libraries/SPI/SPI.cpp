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
#endif // USE_TINYUSB

#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

// Default SERCOM ISR symbols are derived from PERIPH_SPI* routing.
// Variants may override the SPI*_IT_HANDLER* macros when split vectors need
// custom ownership.
#define SPI_SERCOM_INDEX_sercom0 0
#define SPI_SERCOM_INDEX_sercom1 1
#define SPI_SERCOM_INDEX_sercom2 2
#define SPI_SERCOM_INDEX_sercom3 3
#define SPI_SERCOM_INDEX_sercom4 4
#define SPI_SERCOM_INDEX_sercom5 5
#if defined(SERCOM6) || defined(SERCOM6_REGS)
#define SPI_SERCOM_INDEX_sercom6 6
#endif // SERCOM6 || SERCOM6_REGS
#if defined(SERCOM7) || defined(SERCOM7_REGS)
#define SPI_SERCOM_INDEX_sercom7 7
#endif // SERCOM7 || SERCOM7_REGS

#define SPI_SERCOM_INDEX(token) SPI_SERCOM_INDEX_##token
#define SPI_SERCOM_HANDLER_FROM_INDEX(idx) SPI_SERCOM_HANDLER_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER_FROM_INDEX_2(idx) SERCOM##idx##_Handler
#define SPI_SERCOM_HANDLER_FROM_TOKEN(token)                                   \
  SPI_SERCOM_HANDLER_FROM_INDEX(SPI_SERCOM_INDEX(token))

#define SPI_SERCOM_HANDLER0_FROM_INDEX(idx)                                    \
  SPI_SERCOM_HANDLER0_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER0_FROM_INDEX_2(idx) SERCOM##idx##_0_Handler
#define SPI_SERCOM_HANDLER1_FROM_INDEX(idx)                                    \
  SPI_SERCOM_HANDLER1_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER1_FROM_INDEX_2(idx) SERCOM##idx##_1_Handler
#define SPI_SERCOM_HANDLER2_FROM_INDEX(idx)                                    \
  SPI_SERCOM_HANDLER2_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER2_FROM_INDEX_2(idx) SERCOM##idx##_2_Handler
#define SPI_SERCOM_HANDLER3_FROM_INDEX(idx)                                    \
  SPI_SERCOM_HANDLER3_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER3_FROM_INDEX_2(idx) SERCOM##idx##_3_Handler
#define SPI_SERCOM_HANDLER_OTHER_FROM_INDEX(idx)                               \
  SPI_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx)
#define SPI_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx) SERCOM##idx##_OTHER_Handler

#define SPI_SERCOM_HANDLER0_FROM_TOKEN(token)                                  \
  SPI_SERCOM_HANDLER0_FROM_INDEX(SPI_SERCOM_INDEX(token))
#define SPI_SERCOM_HANDLER1_FROM_TOKEN(token)                                  \
  SPI_SERCOM_HANDLER1_FROM_INDEX(SPI_SERCOM_INDEX(token))
#define SPI_SERCOM_HANDLER2_FROM_TOKEN(token)                                  \
  SPI_SERCOM_HANDLER2_FROM_INDEX(SPI_SERCOM_INDEX(token))
#define SPI_SERCOM_HANDLER3_FROM_TOKEN(token)                                  \
  SPI_SERCOM_HANDLER3_FROM_INDEX(SPI_SERCOM_INDEX(token))
#define SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(token)                             \
  SPI_SERCOM_HANDLER_OTHER_FROM_INDEX(SPI_SERCOM_INDEX(token))

#define SPI_DEFINE_SINGLE_HANDLER(handler, instance)                           \
  void handler(void) __attribute__((weak));                                    \
  void handler(void) { instance.onService(); }

#define SPI_DEFINE_SAMD51_E51_HANDLERS(handler0, handler1, handler2, handler3, \
                                       instance)                               \
  void handler0(void) __attribute__((weak));                                   \
  void handler1(void) __attribute__((weak));                                   \
  void handler2(void) __attribute__((weak));                                   \
  void handler3(void) __attribute__((weak));                                   \
  void handler0(void) { instance.onService(); }                                \
  void handler1(void) { instance.onService(); }                                \
  void handler2(void) { instance.onService(); }                                \
  void handler3(void) { instance.onService(); }

#define SPI_DEFINE_SAME53_E54_HANDLERS(handler0, handler1, handler2,           \
                                       handlerOther, instance)                 \
  void handler0(void) __attribute__((weak));                                   \
  void handler1(void) __attribute__((weak));                                   \
  void handler2(void) __attribute__((weak));                                   \
  void handlerOther(void) __attribute__((weak));                               \
  void handler0(void) { instance.onService(); }                                \
  void handler1(void) { instance.onService(); }                                \
  void handler2(void) { instance.onService(); }                                \
  void handlerOther(void) { instance.onService(); }

#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI_DEFINE_SERCOM_HANDLERS(prefix, instance, periph)                   \
  SPI_DEFINE_SAMD51_E51_HANDLERS(prefix##_IT_HANDLER_0,                        \
                                 prefix##_IT_HANDLER_1,                        \
                                 prefix##_IT_HANDLER_2,                        \
                                 prefix##_IT_HANDLER_3, instance)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI_DEFINE_SERCOM_HANDLERS(prefix, instance, periph)                   \
  SPI_DEFINE_SAME53_E54_HANDLERS(prefix##_IT_HANDLER_0,                        \
                                 prefix##_IT_HANDLER_1,                        \
                                 prefix##_IT_HANDLER_2,                        \
                                 prefix##_IT_HANDLER_OTHER, instance)
#else
#define SPI_DEFINE_SERCOM_HANDLERS(prefix, instance, periph)                   \
  SPI_DEFINE_SINGLE_HANDLER(prefix##_IT_HANDLER, instance)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

namespace sercomPinMux {
bool spiMosiPinValidForRoute(uint8_t arduinoPin, uint8_t sercomIndex,
                             uint8_t mosiPad) {
  if (arduinoPin >= PINS_COUNT || mosiPad > 3)
    return false;

  const char portLetter =
      static_cast<char>('A' + g_APinDescription[arduinoPin].ulPort);
  const uint8_t portPin =
      static_cast<uint8_t>(g_APinDescription[arduinoPin].ulPin);

#define SERCOM_PINMUX_EMIT_SPI 1
#define SERCOM_SPI_PIN(PORT_LETTER, PORT_PIN, S0, P0, S1, P1)                   \
  if ((portLetter == (PORT_LETTER)) && (portPin == (PORT_PIN)) &&               \
      (((S0) == sercomIndex && (P0) == mosiPad) ||                              \
       ((S1) == sercomIndex && (P1) == mosiPad)))                               \
    return true;
#include <SercomPinMux.inc>
#undef SERCOM_SPI_PIN
#undef SERCOM_PINMUX_EMIT_SPI

  return false;
}
} // namespace sercomPinMux

namespace {
static uint8_t spiMosiPadFromTxPad(SercomSpiTXPad txPad) {
  switch (txPad) {
    case SPI_PAD_0_SCK_1:
    case SPI_PAD_0_SCK_3:
      return 0;
    case SPI_PAD_2_SCK_3:
      return 2;
    case SPI_PAD_3_SCK_1:
      return 3;
    default:
      return 255;
  }
}
} // namespace

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

bool SPIClass::begin()
{
  if(!initialized) {
    interruptMode = SPI_IMODE_NONE;
    interruptSave = 0;
    interruptMask = 0;
    initialized = true;
  }

#ifdef USE_ZERODMA
  _p_sercom->dmaInit(_p_sercom->getSercomIndex());
#endif // USE_ZERODMA

  // PIO init
  const uint8_t sercomIndex = static_cast<uint8_t>(_p_sercom->getSercomIndex());
  const uint8_t mosiPad = spiMosiPadFromTxPad(_padTx);
  if (!sercomPinMux::spiMosiPinValidForRoute(_uc_pinMosi, sercomIndex, mosiPad))
    return false;

  pinPeripheral(_uc_pinMiso, g_APinDescription[_uc_pinMiso].ulPinType);
  pinPeripheral(_uc_pinSCK, g_APinDescription[_uc_pinSCK].ulPinType);
  pinPeripheral(_uc_pinMosi, g_APinDescription[_uc_pinMosi].ulPinType);

  config(DEFAULT_SPI_SETTINGS);
  return true;
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
#endif // !interruptsStatus

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
    else if (interruptMode & SPI_IMODE_EXTINT) {
#if defined(__SAME53__) || defined(__SAME54__)
      EIC_REGS->EIC_INTENCLR = EIC_INTENCLR_EXTINT(interruptMask);
#else
      EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
#endif // __SAME53__ / __SAME54__
    }
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
    else if (interruptMode & SPI_IMODE_EXTINT) {
#if defined(__SAME53__) || defined(__SAME54__)
      EIC_REGS->EIC_INTENSET = EIC_INTENSET_EXTINT(interruptMask);
#else
      EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
#endif // __SAME53__ / __SAME54__
    }
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
  txn->chainNext = false;
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

#if defined(__SAME53__) || defined(__SAME54__)
  const bool isError = flags & SERCOM_SPIM_INTFLAG_ERROR_Msk;
  const bool isRxc = flags & SERCOM_SPIM_INTFLAG_RXC_Msk;
  const bool isDre = flags & SERCOM_SPIM_INTFLAG_DRE_Msk;
  const uint8_t disableDoneMask = SERCOM_SPIM_INTENCLR_DRE_Msk |
                                  SERCOM_SPIM_INTENCLR_RXC_Msk |
                                  SERCOM_SPIM_INTENCLR_ERROR_Msk;
  const uint8_t disableDreMask = SERCOM_SPIM_INTENCLR_DRE_Msk;
#else
  const bool isError = flags & SERCOM_SPI_INTFLAG_ERROR;
  const bool isRxc = flags & SERCOM_SPI_INTFLAG_RXC;
  const bool isDre = flags & SERCOM_SPI_INTFLAG_DRE;
  const uint8_t disableDoneMask = SERCOM_SPI_INTENCLR_DRE |
                                  SERCOM_SPI_INTENCLR_RXC |
                                  SERCOM_SPI_INTENCLR_ERROR;
  const uint8_t disableDreMask = SERCOM_SPI_INTENCLR_DRE;
#endif // __SAME53__ / __SAME54__

  if (isError)
  {
    _p_sercom->setReturnValueSPI(SercomSpiError::BUF_OVERFLOW);
    _p_sercom->clearINTFLAG();
    _p_sercom->deferStopSPI(SercomSpiError::BUF_OVERFLOW);
    return;
  }

  if (isRxc) {
    // Read completes after write, so read previous byte
    bool hasMore = _p_sercom->readDataSPI();

    if (!hasMore) {
      _p_sercom->disableInterrupts(disableDoneMask);
      _p_sercom->setReturnValueSPI(SercomSpiError::SUCCESS);
      _p_sercom->deferStopSPI(SercomSpiError::SUCCESS);
      return;
    }
  }

  if (isDre) {
    bool hasMore = _p_sercom->sendDataSPI();
    if (!hasMore)
      _p_sercom->disableInterrupts(disableDreMask);
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
#define PERIPH_SPI sercom4
#define PAD_SPI_TX SPI_PAD_2_SCK_3
#define PAD_SPI_RX SERCOM_RX_PAD_0
#endif // PERIPH_SPI

SPIClass SPI(&PERIPH_SPI, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_MOSI, PAD_SPI_TX,
             PAD_SPI_RX);
#ifndef SPI_IT_HANDLER
#define SPI_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI)
#endif // !SPI_IT_HANDLER
#ifndef SPI_IT_HANDLER_0
#define SPI_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI)
#define SPI_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI)
#define SPI_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI, SPI, PERIPH_SPI)
#endif // SPI_INTERFACES_COUNT > 0

#if SPI_INTERFACES_COUNT > 1
SPIClass SPI1(&PERIPH_SPI1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI,
              PAD_SPI1_TX, PAD_SPI1_RX);
#ifndef SPI1_IT_HANDLER
#define SPI1_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI1)
#endif // !SPI1_IT_HANDLER
#ifndef SPI1_IT_HANDLER_0
#define SPI1_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI1)
#define SPI1_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI1)
#define SPI1_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI1)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI1_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI1)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI1_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI1)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI1_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI1, SPI1, PERIPH_SPI1)
#endif // SPI_INTERFACES_COUNT > 1

#if SPI_INTERFACES_COUNT > 2
SPIClass SPI2(&PERIPH_SPI2, PIN_SPI2_MISO, PIN_SPI2_SCK, PIN_SPI2_MOSI,
              PAD_SPI2_TX, PAD_SPI2_RX);
#ifndef SPI2_IT_HANDLER
#define SPI2_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI2)
#endif // !SPI2_IT_HANDLER
#ifndef SPI2_IT_HANDLER_0
#define SPI2_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI2)
#define SPI2_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI2)
#define SPI2_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI2)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI2_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI2)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI2_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI2)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI2_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI2, SPI2, PERIPH_SPI2)
#endif // SPI_INTERFACES_COUNT > 2

#if SPI_INTERFACES_COUNT > 3
SPIClass SPI3(&PERIPH_SPI3, PIN_SPI3_MISO, PIN_SPI3_SCK, PIN_SPI3_MOSI,
              PAD_SPI3_TX, PAD_SPI3_RX);
#ifndef SPI3_IT_HANDLER
#define SPI3_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI3)
#endif // !SPI3_IT_HANDLER
#ifndef SPI3_IT_HANDLER_0
#define SPI3_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI3)
#define SPI3_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI3)
#define SPI3_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI3)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI3_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI3)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI3_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI3)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI3_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI3, SPI3, PERIPH_SPI3)
#endif // SPI_INTERFACES_COUNT > 3

#if SPI_INTERFACES_COUNT > 4
SPIClass SPI4(&PERIPH_SPI4, PIN_SPI4_MISO, PIN_SPI4_SCK, PIN_SPI4_MOSI,
              PAD_SPI4_TX, PAD_SPI4_RX);
#ifndef SPI4_IT_HANDLER
#define SPI4_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI4)
#endif // !SPI4_IT_HANDLER
#ifndef SPI4_IT_HANDLER_0
#define SPI4_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI4)
#define SPI4_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI4)
#define SPI4_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI4)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI4_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI4)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI4_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI4)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI4_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI4, SPI4, PERIPH_SPI4)
#endif // SPI_INTERFACES_COUNT > 4

#if SPI_INTERFACES_COUNT > 5
SPIClass SPI5(&PERIPH_SPI5, PIN_SPI5_MISO, PIN_SPI5_SCK, PIN_SPI5_MOSI,
              PAD_SPI5_TX, PAD_SPI5_RX);
#ifndef SPI5_IT_HANDLER
#define SPI5_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI5)
#endif // !SPI5_IT_HANDLER
#ifndef SPI5_IT_HANDLER_0
#define SPI5_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI5)
#define SPI5_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI5)
#define SPI5_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI5)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI5_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI5)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI5_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI5)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI5_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI5, SPI5, PERIPH_SPI5)
#endif // SPI_INTERFACES_COUNT > 5

#if SPI_INTERFACES_COUNT > 6
SPIClass SPI6(&PERIPH_SPI6, PIN_SPI6_MISO, PIN_SPI6_SCK, PIN_SPI6_MOSI,
              PAD_SPI6_TX, PAD_SPI6_RX);
#ifndef SPI6_IT_HANDLER
#define SPI6_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI6)
#endif // !SPI6_IT_HANDLER
#ifndef SPI6_IT_HANDLER_0
#define SPI6_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI6)
#define SPI6_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI6)
#define SPI6_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI6)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI6_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI6)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI6_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI6)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI6_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI6, SPI6, PERIPH_SPI6)
#endif // SPI_INTERFACES_COUNT > 6

#if SPI_INTERFACES_COUNT > 7
SPIClass SPI7(&PERIPH_SPI7, PIN_SPI7_MISO, PIN_SPI7_SCK, PIN_SPI7_MOSI,
              PAD_SPI7_TX, PAD_SPI7_RX);
#ifndef SPI7_IT_HANDLER
#define SPI7_IT_HANDLER SPI_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SPI7)
#endif // !SPI7_IT_HANDLER
#ifndef SPI7_IT_HANDLER_0
#define SPI7_IT_HANDLER_0 SPI_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SPI7)
#define SPI7_IT_HANDLER_1 SPI_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SPI7)
#define SPI7_IT_HANDLER_2 SPI_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SPI7)
#if defined(__SAMD51__) || defined(__SAME51__)
#define SPI7_IT_HANDLER_3 SPI_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SPI7)
#elif defined(__SAME53__) || defined(__SAME54__)
#define SPI7_IT_HANDLER_OTHER SPI_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SPI7)
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__
#endif // !SPI7_IT_HANDLER_0
SPI_DEFINE_SERCOM_HANDLERS(SPI7, SPI7, PERIPH_SPI7)
#endif // SPI_INTERFACES_COUNT > 7
