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

// Derive SERCOM ISR symbols from PERIPH_SERIAL* tokens (e.g. sercom4 ->
// SERCOM4_Handler) so variants only need to describe the UART routing.
#define UART_SERCOM_INDEX_sercom0 0
#define UART_SERCOM_INDEX_sercom1 1
#define UART_SERCOM_INDEX_sercom2 2
#define UART_SERCOM_INDEX_sercom3 3
#define UART_SERCOM_INDEX_sercom4 4
#define UART_SERCOM_INDEX_sercom5 5
#if defined(SERCOM6)
#define UART_SERCOM_INDEX_sercom6 6
#endif
#if defined(SERCOM7)
#define UART_SERCOM_INDEX_sercom7 7
#endif

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

#define UART_SERCOM_HANDLER0_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER0_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER1_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER1_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER2_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER2_FROM_INDEX(UART_SERCOM_INDEX(token))
#define UART_SERCOM_HANDLER3_FROM_TOKEN(token)                                 \
  UART_SERCOM_HANDLER3_FROM_INDEX(UART_SERCOM_INDEX(token))

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
#endif
void SERIAL_IT_HANDLER()
{
  Serial.IrqHandler();
}

#ifdef FAMILY_SAMD5X
#ifndef SERIAL_IT_HANDLER_0
#define SERIAL_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL)
#define SERIAL_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL)
#define SERIAL_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL)
#define SERIAL_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL)
#endif
void SERIAL_IT_HANDLER_0() { Serial.IrqHandler(); }
void SERIAL_IT_HANDLER_1() { Serial.IrqHandler(); }
void SERIAL_IT_HANDLER_2() { Serial.IrqHandler(); }
void SERIAL_IT_HANDLER_3() { Serial.IrqHandler(); }
#endif
#endif

#if defined(PERIPH_SERIAL_UART) && !defined(UART_VARIANT_OWNS_SERIAL_UART)
Uart SerialUART(&PERIPH_SERIAL_UART, PIN_SERIAL_UART_RX, PIN_SERIAL_UART_TX,
                PAD_SERIAL_UART_RX, PAD_SERIAL_UART_TX);

#ifndef SERIAL_UART_IT_HANDLER
#define SERIAL_UART_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL_UART)
#endif
#ifndef FAMILY_SAMD5X
void SERIAL_UART_IT_HANDLER()
{
  SerialUART.IrqHandler();
}
#else
#ifndef SERIAL_UART_IT_HANDLER_0
#define SERIAL_UART_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL_UART)
#define SERIAL_UART_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL_UART)
#define SERIAL_UART_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL_UART)
#define SERIAL_UART_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL_UART)
#endif
void SERIAL_UART_IT_HANDLER_0() { SerialUART.IrqHandler(); }
void SERIAL_UART_IT_HANDLER_1() { SerialUART.IrqHandler(); }
void SERIAL_UART_IT_HANDLER_2() { SerialUART.IrqHandler(); }
void SERIAL_UART_IT_HANDLER_3() { SerialUART.IrqHandler(); }
#endif
#endif

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
#endif
#ifndef FAMILY_SAMD5X
void SERIAL1_IT_HANDLER()
{
  Serial1.IrqHandler();
}
#else
#ifndef SERIAL1_IT_HANDLER_0
#define SERIAL1_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL1)
#define SERIAL1_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL1)
#define SERIAL1_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL1)
#define SERIAL1_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL1)
#endif
void SERIAL1_IT_HANDLER_0() { Serial1.IrqHandler(); }
void SERIAL1_IT_HANDLER_1() { Serial1.IrqHandler(); }
void SERIAL1_IT_HANDLER_2() { Serial1.IrqHandler(); }
void SERIAL1_IT_HANDLER_3() { Serial1.IrqHandler(); }
#endif
#endif

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
#endif
#ifndef FAMILY_SAMD5X
void SERIAL2_IT_HANDLER()
{
  Serial2.IrqHandler();
}
#else
#ifndef SERIAL2_IT_HANDLER_0
#define SERIAL2_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL2)
#define SERIAL2_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL2)
#define SERIAL2_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL2)
#define SERIAL2_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL2)
#endif
void SERIAL2_IT_HANDLER_0() { Serial2.IrqHandler(); }
void SERIAL2_IT_HANDLER_1() { Serial2.IrqHandler(); }
void SERIAL2_IT_HANDLER_2() { Serial2.IrqHandler(); }
void SERIAL2_IT_HANDLER_3() { Serial2.IrqHandler(); }
#endif
#endif

#if defined(PERIPH_SERIAL3) && !defined(UART_VARIANT_OWNS_SERIAL3)
Uart Serial3(&PERIPH_SERIAL3, PIN_SERIAL3_RX, PIN_SERIAL3_TX, PAD_SERIAL3_RX,
             PAD_SERIAL3_TX);

#ifndef SERIAL3_IT_HANDLER
#define SERIAL3_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL3)
#endif
#ifndef FAMILY_SAMD5X
void SERIAL3_IT_HANDLER()
{
  Serial3.IrqHandler();
}
#else
#ifndef SERIAL3_IT_HANDLER_0
#define SERIAL3_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL3)
#define SERIAL3_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL3)
#define SERIAL3_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL3)
#define SERIAL3_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL3)
#endif
void SERIAL3_IT_HANDLER_0() { Serial3.IrqHandler(); }
void SERIAL3_IT_HANDLER_1() { Serial3.IrqHandler(); }
void SERIAL3_IT_HANDLER_2() { Serial3.IrqHandler(); }
void SERIAL3_IT_HANDLER_3() { Serial3.IrqHandler(); }
#endif
#endif

#if defined(PERIPH_SERIAL4) && !defined(UART_VARIANT_OWNS_SERIAL4)
Uart Serial4(&PERIPH_SERIAL4, PIN_SERIAL4_RX, PIN_SERIAL4_TX, PAD_SERIAL4_RX,
             PAD_SERIAL4_TX);

#ifndef SERIAL4_IT_HANDLER
#define SERIAL4_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL4)
#endif
#ifndef FAMILY_SAMD5X
void SERIAL4_IT_HANDLER()
{
  Serial4.IrqHandler();
}
#else
#ifndef SERIAL4_IT_HANDLER_0
#define SERIAL4_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL4)
#define SERIAL4_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL4)
#define SERIAL4_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL4)
#define SERIAL4_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL4)
#endif
void SERIAL4_IT_HANDLER_0() { Serial4.IrqHandler(); }
void SERIAL4_IT_HANDLER_1() { Serial4.IrqHandler(); }
void SERIAL4_IT_HANDLER_2() { Serial4.IrqHandler(); }
void SERIAL4_IT_HANDLER_3() { Serial4.IrqHandler(); }
#endif
#endif

#if defined(PERIPH_SERIAL5) && !defined(UART_VARIANT_OWNS_SERIAL5)
Uart Serial5(&PERIPH_SERIAL5, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX,
             PAD_SERIAL_TX);

#ifndef SERIAL5_IT_HANDLER
#define SERIAL5_IT_HANDLER UART_SERCOM_HANDLER_FROM_TOKEN(PERIPH_SERIAL5)
#endif
void SERIAL5_IT_HANDLER()
{
  Serial5.IrqHandler();
}

#ifdef FAMILY_SAMD5X
#ifndef SERIAL5_IT_HANDLER_0
#define SERIAL5_IT_HANDLER_0 UART_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_SERIAL5)
#define SERIAL5_IT_HANDLER_1 UART_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_SERIAL5)
#define SERIAL5_IT_HANDLER_2 UART_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_SERIAL5)
#define SERIAL5_IT_HANDLER_3 UART_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_SERIAL5)
#endif
void SERIAL5_IT_HANDLER_0() { Serial5.IrqHandler(); }
void SERIAL5_IT_HANDLER_1() { Serial5.IrqHandler(); }
void SERIAL5_IT_HANDLER_2() { Serial5.IrqHandler(); }
void SERIAL5_IT_HANDLER_3() { Serial5.IrqHandler(); }
#endif
#endif
