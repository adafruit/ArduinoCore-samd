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
    pul_outsetRTS = &PORT->Group[rtsPort].OUTSET.reg;
    pul_outclrRTS = &PORT->Group[rtsPort].OUTCLR.reg;
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
#endif
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
#endif
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
  sercom->getSercom()->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;

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
    sercom->getSercom()->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
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
#endif
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
    self->sercom->getSercom()->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
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
