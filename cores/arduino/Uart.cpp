/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "Uart.h"
#include "Arduino.h"
#include "wiring_private.h"

#define NO_RTS_PIN 255
#define NO_CTS_PIN 255
#define RTS_RX_THRESHOLD 10

// Default SERCOM ISR symbols are derived from PERIPH_SERIAL* routing.
// Variants may override SERIAL*_IT_HANDLER* macros when split vectors need
// custom ownership.
#define UART_SERCOM_INDEX_sercom0 0
#define UART_SERCOM_INDEX_sercom1 1
#define UART_SERCOM_INDEX_sercom2 2
#define UART_SERCOM_INDEX_sercom3 3
#define UART_SERCOM_INDEX_sercom4 4
#define UART_SERCOM_INDEX_sercom5 5
#if defined(SERCOM6) || defined(SERCOM6_REGS)
#define UART_SERCOM_INDEX_sercom6 6
#endif // SERCOM6 || SERCOM6_REGS
#if defined(SERCOM7) || defined(SERCOM7_REGS)
#define UART_SERCOM_INDEX_sercom7 7
#endif // SERCOM7 || SERCOM7_REGS

#define UART_SERCOM_INDEX(token) UART_SERCOM_INDEX_##token
#define UART_SERCOM_HANDLER_FROM_INDEX(idx)                                    \
  UART_SERCOM_HANDLER_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER_FROM_INDEX_2(idx) SERCOM##idx##_Handler
#define UART_SERCOM_HANDLER_FROM_TOKEN(token)                                  \
  UART_SERCOM_HANDLER_FROM_INDEX(UART_SERCOM_INDEX(token))

#define UART_SERCOM_HANDLER0_FROM_INDEX(idx)                                   \
  UART_SERCOM_HANDLER0_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER0_FROM_INDEX_2(idx) SERCOM##idx##_0_Handler
#define UART_SERCOM_HANDLER1_FROM_INDEX(idx)                                   \
  UART_SERCOM_HANDLER1_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER1_FROM_INDEX_2(idx) SERCOM##idx##_1_Handler
#define UART_SERCOM_HANDLER2_FROM_INDEX(idx)                                   \
  UART_SERCOM_HANDLER2_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER2_FROM_INDEX_2(idx) SERCOM##idx##_2_Handler
#define UART_SERCOM_HANDLER3_FROM_INDEX(idx)                                   \
  UART_SERCOM_HANDLER3_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER3_FROM_INDEX_2(idx) SERCOM##idx##_3_Handler
#define UART_SERCOM_HANDLER_OTHER_FROM_INDEX(idx)                              \
  UART_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx)
#define UART_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx) SERCOM##idx##_OTHER_Handler

#define UART_SERCOM_HANDLER0_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER0_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER1_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER1_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER2_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER2_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER3_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER3_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(token)                            \
  UART_SERCOM_HANDLER_OTHER_FROM_INDEX(UART_SERCOM_INDEX(token))

#define UART_DEFINE_SINGLE_HANDLER(handler, instance)                          \
  void handler(void) { instance.IrqHandler(); }

#define UART_DEFINE_SAMD51_E51_HANDLERS(handler0, handler1, handler2,          \
                                        handler3, instance)                    \
  void handler0(void) { instance.IrqHandler(); }                               \
  void handler1(void) { instance.IrqHandler(); }                               \
  void handler2(void) { instance.IrqHandler(); }                               \
  void handler3(void) { instance.IrqHandler(); }

#define UART_DEFINE_SAME53_E54_HANDLERS(handler0, handler1, handler2,          \
                                        handlerOther, instance)                \
  void handler0(void) { instance.IrqHandler(); }                               \
  void handler1(void) { instance.IrqHandler(); }                               \
  void handler2(void) { instance.IrqHandler(); }                               \
  void handlerOther(void) { instance.IrqHandler(); }

#ifdef ARDUINO_SAMD51_E51
#define UART_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  UART_DEFINE_SAMD51_E51_HANDLERS(prefix##_IT_HANDLER_0,                       \
                                  prefix##_IT_HANDLER_1,                       \
                                  prefix##_IT_HANDLER_2,                       \
                                  prefix##_IT_HANDLER_3, instance)
#elif defined(ARDUINO_SAME53_E54)
#define UART_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  UART_DEFINE_SAME53_E54_HANDLERS(prefix##_IT_HANDLER_0,                       \
                                  prefix##_IT_HANDLER_1,                       \
                                  prefix##_IT_HANDLER_2,                       \
                                  prefix##_IT_HANDLER_OTHER, instance)
#else
#define UART_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  UART_DEFINE_SINGLE_HANDLER(prefix##_IT_HANDLER, instance)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX) :
  Uart(_s, _pinRX, _pinTX, _padRX, _padTX, NO_RTS_PIN, NO_CTS_PIN)
{
}

Uart::Uart(SERCOM *_s, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX, uint8_t _pinRTS, uint8_t _pinCTS)
{
  sercom = _s;
  uc_pinRX = _pinRX;
  uc_pinTX = _pinTX;
  uc_padRX = _padRX ;
  uc_padTX = _padTX;
  uc_pinRTS = _pinRTS;
  uc_pinCTS = _pinCTS;
  txnPoolHead = 0;
}

void Uart::begin(unsigned long baudrate)
{
  begin(baudrate, SERIAL_8N1);
}

void Uart::begin(unsigned long baudrate, uint16_t config)
{
  pinPeripheral(uc_pinRX, g_APinDescription[uc_pinRX].ulPinType);
  pinPeripheral(uc_pinTX, g_APinDescription[uc_pinTX].ulPinType);

  if (uc_padTX == UART_TX_RTS_CTS_PAD_0_2_3) {
    if (uc_pinCTS != NO_CTS_PIN) {
      pinPeripheral(uc_pinCTS, g_APinDescription[uc_pinCTS].ulPinType);
    }
  }

  if (uc_pinRTS != NO_RTS_PIN) {
    pinMode(uc_pinRTS, OUTPUT);

    EPortType rtsPort = g_APinDescription[uc_pinRTS].ulPort;
#if defined(ARDUINO_SAME53_E54)
    pul_outsetRTS = &PORT_REGS->GROUP[rtsPort].PORT_OUTSET;
    pul_outclrRTS = &PORT_REGS->GROUP[rtsPort].PORT_OUTCLR;
#else
    pul_outsetRTS = &PORT->Group[rtsPort].OUTSET.reg;
    pul_outclrRTS = &PORT->Group[rtsPort].OUTCLR.reg;
#endif // ARDUINO_SAME53_E54
    ul_pinMaskRTS = (1ul << g_APinDescription[uc_pinRTS].ulPin);

    *pul_outclrRTS = ul_pinMaskRTS;
  }

  sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
  sercom->initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
  sercom->initPads(uc_padTX, uc_padRX);

  sercom->enableUART();
}

void Uart::end()
{
  sercom->resetUART();
  rxBuffer.clear();
  txBuffer.clear();
}

void Uart::flush()
{
  while(txBuffer.available()); // wait until TX buffer is empty

  sercom->flushUART();
}

void Uart::IrqHandler()
{
  if (sercom->isFrameErrorUART()) {
    // frame error, next byte is invalid so read and discard it
    sercom->readDataUART();

    sercom->clearFrameErrorUART();
  }

  if (sercom->availableDataUART()) {
    rxBuffer.store_char(sercom->readDataUART());

    if (uc_pinRTS != NO_RTS_PIN) {
      // RX buffer space is below the threshold, de-assert RTS
      if (rxBuffer.availableForStore() < RTS_RX_THRESHOLD) {
        *pul_outsetRTS = ul_pinMaskRTS;
      }
    }
  }

  if (sercom->isDataRegisterEmptyUART()) {
    if (txBuffer.available()) {
      uint8_t data = txBuffer.read_char();

      sercom->writeDataUART(data);
    } else {
      sercom->disableDataRegisterEmptyInterruptUART();
    }
  }

  if (sercom->isUARTError()) {
    sercom->acknowledgeUARTError();
    // TODO: if (sercom->isBufferOverflowErrorUART()) ....
    // TODO: if (sercom->isParityErrorUART()) ....
    sercom->clearStatusUART();
  }
}

int Uart::available()
{
  return rxBuffer.available();
}

int Uart::availableForWrite()
{
  return txBuffer.availableForStore();
}

int Uart::peek()
{
  return rxBuffer.peek();
}

int Uart::read()
{
  int c = rxBuffer.read_char();

  if (uc_pinRTS != NO_RTS_PIN) {
    // if there is enough space in the RX buffer, assert RTS
    if (rxBuffer.availableForStore() > RTS_RX_THRESHOLD) {
      *pul_outclrRTS = ul_pinMaskRTS;
    }
  }

  return c;
}

size_t Uart::write(const uint8_t data)
{
  if (sercom->isDataRegisterEmptyUART() && txBuffer.available() == 0) {
    sercom->writeDataUART(data);
  } else {
    // spin lock until a spot opens up in the buffer
    while(txBuffer.isFull()) {
      uint8_t interruptsEnabled = ((__get_PRIMASK() & 0x1) == 0);

      if (interruptsEnabled) {
        uint32_t exceptionNumber = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

        if (exceptionNumber == 0 ||
              NVIC_GetPriority((IRQn_Type)(exceptionNumber - 16)) > SERCOM_NVIC_PRIORITY) {
          // no exception or called from an ISR with lower priority,
          // wait for free buffer spot via IRQ
          continue;
        }
      }

      // interrupts are disabled or called from ISR with higher or equal priority than the SERCOM IRQ
      // manually call the UART IRQ handler when the data register is empty
      if (sercom->isDataRegisterEmptyUART()) {
        IrqHandler();
      }
    }

    txBuffer.store_char(data);

    sercom->enableDataRegisterEmptyInterruptUART();
  }

  return 1;
}

size_t Uart::write(const uint8_t* buffer, size_t size,
                    void (*onComplete)(void* user, int status),
                    void* user)
{
  if (buffer == nullptr || size == 0)
    return 0;

  if (onComplete == nullptr) {
    // Synchronous path: block until complete
#ifdef USE_ZERODMA
    SercomTxn* txn = allocateTxn();
    txn->txPtr = buffer;
    txn->rxPtr = nullptr;
    txn->length = size;
    txn->onComplete = &Uart::onTxnComplete;
    txn->user = this;
    txnDone = false;
    txnStatus = 0;

    if (sercom->enqueueUART(txn)) {
      while (!txnDone) ;
      return size;
    }
#endif // USE_ZERODMA
    // Fallback: byte-by-byte
    for (size_t i = 0; i < size; ++i)
      write(buffer[i]);
    return size;
  } else {
    // Asynchronous path: enqueue and return immediately
#ifdef USE_ZERODMA
    SercomTxn* txn = allocateTxn();
    txn->txPtr = buffer;
    txn->rxPtr = nullptr;
    txn->length = size;
    txn->onComplete = onComplete;
    txn->user = user;
    txnDone = false;
    txnStatus = 0;

    if (!sercom->enqueueUART(txn))
      return 0;

    return size;
#else
    (void)onComplete;
    (void)user;
    for (size_t i = 0; i < size; ++i)
      write(buffer[i]);
    return size;
#endif // USE_ZERODMA
  }
}

size_t Uart::read(uint8_t* buffer, size_t size, void (*onComplete)(void* user, int status), void* user)
{
  if (buffer == nullptr || size == 0)
    return 0;

  if (onComplete == nullptr) {
    // Synchronous path: read from ring buffer
    size_t readCount = 0;
    while (readCount < size) {
      int c = read();
      if (c >= 0)
        buffer[readCount++] = static_cast<uint8_t>(c);
    }
    return readCount;
  }

#ifdef USE_ZERODMA
  // Asynchronous path: use DMA
  pendingRxCb = onComplete;
  pendingRxUser = user;
  rxExternalActive = true;

  // Disable RXC interrupt; DMA takes over
  sercom->disableReceiveCompleteInterruptUART();

  SercomTxn* txn = allocateTxn();
  txn->txPtr = nullptr;
  txn->rxPtr = buffer;
  txn->length = size;
  txn->onComplete = &Uart::onTxnComplete;
  txn->user = this;
  txnDone = false;
  txnStatus = 0;

  if (!sercom->enqueueUART(txn)) {
    // Enqueue failed; restore RXC interrupt and clear pending state
    sercom->enableReceiveCompleteInterruptUART();
    rxExternalActive = false;
    pendingRxCb = nullptr;
    pendingRxUser = nullptr;
    return 0;
  }

  return size;
#else
  (void)onComplete;
  (void)user;
  return 0;
#endif // USE_ZERODMA
}

SercomTxn* Uart::allocateTxn() {
  // Simple round-robin allocation from pool
  SercomTxn* txn = &txnPool[txnPoolHead];
  txnPoolHead = (txnPoolHead + 1) % TXN_POOL_SIZE;
  return txn;
}

void Uart::onTxnComplete(void* user, int status)
{
  if (!user)
    return;
  Uart* self = static_cast<Uart*>(user);
  self->txnStatus = status;
  self->txnDone = true;
  if (self->rxExternalActive) {
    self->rxExternalActive = false;
    self->sercom->enableReceiveCompleteInterruptUART();
    if (self->pendingRxCb) {
      void (*cb)(void*, int) = self->pendingRxCb;
      void* cbUser = self->pendingRxUser;
      self->pendingRxCb = nullptr;
      self->pendingRxUser = nullptr;
      cb(cbUser, status);
    }
  }
}

SercomNumberStopBit Uart::extractNbStopBit(uint16_t config)
{
  switch(config & HARDSER_STOP_BIT_MASK)
  {
    case HARDSER_STOP_BIT_1:
    default:
      return SERCOM_STOP_BIT_1;

    case HARDSER_STOP_BIT_2:
      return SERCOM_STOP_BITS_2;
  }
}

SercomUartCharSize Uart::extractCharSize(uint16_t config)
{
  switch(config & HARDSER_DATA_MASK)
  {
    case HARDSER_DATA_5:
      return UART_CHAR_SIZE_5_BITS;

    case HARDSER_DATA_6:
      return UART_CHAR_SIZE_6_BITS;

    case HARDSER_DATA_7:
      return UART_CHAR_SIZE_7_BITS;

    case HARDSER_DATA_8:
    default:
      return UART_CHAR_SIZE_8_BITS;

  }
}

SercomParityMode Uart::extractParity(uint16_t config)
{
  switch(config & HARDSER_PARITY_MASK)
  {
    case HARDSER_PARITY_NONE:
    default:
      return SERCOM_NO_PARITY;

    case HARDSER_PARITY_EVEN:
      return SERCOM_EVEN_PARITY;

    case HARDSER_PARITY_ODD:
      return SERCOM_ODD_PARITY;
  }
}

#if defined(PERIPH_SERIAL) && !defined(UART_VARIANT_OWNS_SERIAL)
Uart Serial(&PERIPH_SERIAL, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX,
            PAD_SERIAL_TX);
#ifndef SERIAL_IT_HANDLER
#define SERIAL_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL)
#endif // !SERIAL_IT_HANDLER
#ifndef SERIAL_IT_HANDLER_0
#define SERIAL_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL)
#define SERIAL_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL)
#define SERIAL_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL, Serial)
#endif // PERIPH_SERIAL && !UART_VARIANT_OWNS_SERIAL

#if defined(PERIPH_SERIAL_UART) && !defined(UART_VARIANT_OWNS_SERIAL_UART)
Uart SerialUART(&PERIPH_SERIAL_UART, PIN_SERIAL_UART_RX, PIN_SERIAL_UART_TX,
                PAD_SERIAL_UART_RX, PAD_SERIAL_UART_TX);
#ifndef SERIAL_UART_IT_HANDLER
#define SERIAL_UART_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL_UART)
#endif // !SERIAL_UART_IT_HANDLER
#ifndef SERIAL_UART_IT_HANDLER_0
#define SERIAL_UART_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL_UART)
#define SERIAL_UART_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL_UART)
#define SERIAL_UART_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL_UART)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL_UART_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL_UART)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL_UART_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL_UART)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL_UART_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL_UART, SerialUART)
#endif // PERIPH_SERIAL_UART && !UART_VARIANT_OWNS_SERIAL_UART

#if defined(PERIPH_SERIAL1) && !defined(UART_VARIANT_OWNS_SERIAL1)
#ifdef PIN_SERIAL1_RTS
Uart Serial1(&PERIPH_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX,
             PAD_SERIAL1_TX, PIN_SERIAL1_RTS, PIN_SERIAL1_CTS);
#else
Uart Serial1(&PERIPH_SERIAL1, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX,
             PAD_SERIAL1_TX);
#endif

#ifndef SERIAL1_IT_HANDLER
#define SERIAL1_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL1)
#endif // !SERIAL1_IT_HANDLER
#ifndef SERIAL1_IT_HANDLER_0
#define SERIAL1_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL1)
#define SERIAL1_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL1)
#define SERIAL1_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL1)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL1_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL1)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL1_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL1)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL1_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL1, Serial1)
#endif // PERIPH_SERIAL1 && !UART_VARIANT_OWNS_SERIAL1

#if defined(PERIPH_SERIAL2) && !defined(UART_VARIANT_OWNS_SERIAL2)
#ifdef PIN_SERIAL2_RTS
Uart Serial2(&PERIPH_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX,
             PAD_SERIAL2_TX, PIN_SERIAL2_RTS, PIN_SERIAL2_CTS);
#else
Uart Serial2(&PERIPH_SERIAL2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX,
             PAD_SERIAL2_TX);
#endif

#ifndef SERIAL2_IT_HANDLER
#define SERIAL2_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL2)
#endif // !SERIAL2_IT_HANDLER
#ifndef SERIAL2_IT_HANDLER_0
#define SERIAL2_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL2)
#define SERIAL2_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL2)
#define SERIAL2_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL2)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL2_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL2)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL2_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL2)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL2_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL2, Serial2)
#endif // PERIPH_SERIAL2 && !UART_VARIANT_OWNS_SERIAL2

#if defined(PERIPH_SERIAL3) && !defined(UART_VARIANT_OWNS_SERIAL3)
Uart Serial3(&PERIPH_SERIAL3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX,
             PAD_SERIAL3_TX);
#ifndef SERIAL3_IT_HANDLER
#define SERIAL3_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL3)
#endif // !SERIAL3_IT_HANDLER
#ifndef SERIAL3_IT_HANDLER_0
#define SERIAL3_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL3)
#define SERIAL3_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL3)
#define SERIAL3_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL3)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL3_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL3)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL3_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL3)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL3_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL3, Serial3)
#endif // PERIPH_SERIAL3 && !UART_VARIANT_OWNS_SERIAL3

#if defined(PERIPH_SERIAL4) && !defined(UART_VARIANT_OWNS_SERIAL4)
Uart Serial4(&PERIPH_SERIAL4, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX,
             PAD_SERIAL4_TX);
#ifndef SERIAL4_IT_HANDLER
#define SERIAL4_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL4)
#endif // !SERIAL4_IT_HANDLER
#ifndef SERIAL4_IT_HANDLER_0
#define SERIAL4_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL4)
#define SERIAL4_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL4)
#define SERIAL4_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL4)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL4_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL4)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL4_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL4)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL4_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL4, Serial4)
#endif // PERIPH_SERIAL4 && !UART_VARIANT_OWNS_SERIAL4

#if defined(PERIPH_SERIAL5) && !defined(UART_VARIANT_OWNS_SERIAL5)
Uart Serial5(&PERIPH_SERIAL5, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX,
             PAD_SERIAL_TX);
#ifndef SERIAL5_IT_HANDLER
#define SERIAL5_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL5)
#endif // !SERIAL5_IT_HANDLER
#ifndef SERIAL5_IT_HANDLER_0
#define SERIAL5_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL5)
#define SERIAL5_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL5)
#define SERIAL5_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL5)
#ifdef ARDUINO_SAMD51_E51
#define SERIAL5_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL5)
#elif defined(ARDUINO_SAME53_E54)
#define SERIAL5_IT_HANDLER_OTHER UART_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_SERIAL5)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !SERIAL5_IT_HANDLER_0
UART_DEFINE_SERCOM_HANDLERS(SERIAL5, Serial5)
#endif // PERIPH_SERIAL5 && !UART_VARIANT_OWNS_SERIAL5
