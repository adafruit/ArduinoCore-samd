/*
 * TWI/I2C library for Arduino Zero
 * Copyright (c) 2015 Arduino LLC. All rights reserved.
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

extern "C" {
#include <string.h>
}

#include <Arduino.h>
#include <wiring_private.h>

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif // USE_TINYUSB

#include "Wire.h"

// Default SERCOM ISR symbols are derived from PERIPH_WIRE* routing.
// Variants may override WIRE*_IT_HANDLER* macros when split vectors need
// custom ownership.
#define WIRE_SERCOM_INDEX_sercom0 0
#define WIRE_SERCOM_INDEX_sercom1 1
#define WIRE_SERCOM_INDEX_sercom2 2
#define WIRE_SERCOM_INDEX_sercom3 3
#define WIRE_SERCOM_INDEX_sercom4 4
#define WIRE_SERCOM_INDEX_sercom5 5
#if defined(SERCOM6) || defined(SERCOM6_REGS)
#define WIRE_SERCOM_INDEX_sercom6 6
#endif // SERCOM6 || SERCOM6_REGS
#if defined(SERCOM7) || defined(SERCOM7_REGS)
#define WIRE_SERCOM_INDEX_sercom7 7
#endif // SERCOM7 || SERCOM7_REGS

#define WIRE_SERCOM_INDEX(token) WIRE_SERCOM_INDEX_##token
#define WIRE_SERCOM_HANDLER_FROM_INDEX(idx)                                    \
  WIRE_SERCOM_HANDLER_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER_FROM_INDEX_2(idx) SERCOM##idx##_Handler
#define WIRE_SERCOM_HANDLER_FROM_TOKEN(token)                                  \
  WIRE_SERCOM_HANDLER_FROM_INDEX(WIRE_SERCOM_INDEX(token))

#define WIRE_SERCOM_HANDLER0_FROM_INDEX(idx)                                   \
  WIRE_SERCOM_HANDLER0_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER0_FROM_INDEX_2(idx) SERCOM##idx##_0_Handler
#define WIRE_SERCOM_HANDLER1_FROM_INDEX(idx)                                   \
  WIRE_SERCOM_HANDLER1_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER1_FROM_INDEX_2(idx) SERCOM##idx##_1_Handler
#define WIRE_SERCOM_HANDLER2_FROM_INDEX(idx)                                   \
  WIRE_SERCOM_HANDLER2_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER2_FROM_INDEX_2(idx) SERCOM##idx##_2_Handler
#define WIRE_SERCOM_HANDLER3_FROM_INDEX(idx)                                   \
  WIRE_SERCOM_HANDLER3_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER3_FROM_INDEX_2(idx) SERCOM##idx##_3_Handler
#define WIRE_SERCOM_HANDLER_OTHER_FROM_INDEX(idx)                              \
  WIRE_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx)
#define WIRE_SERCOM_HANDLER_OTHER_FROM_INDEX_2(idx) SERCOM##idx##_OTHER_Handler

#define WIRE_SERCOM_HANDLER0_FROM_TOKEN(token)                                 \
  WIRE_SERCOM_HANDLER0_FROM_INDEX(WIRE_SERCOM_INDEX(token))
#define WIRE_SERCOM_HANDLER1_FROM_TOKEN(token)                                 \
  WIRE_SERCOM_HANDLER1_FROM_INDEX(WIRE_SERCOM_INDEX(token))
#define WIRE_SERCOM_HANDLER2_FROM_TOKEN(token)                                 \
  WIRE_SERCOM_HANDLER2_FROM_INDEX(WIRE_SERCOM_INDEX(token))
#define WIRE_SERCOM_HANDLER3_FROM_TOKEN(token)                                 \
  WIRE_SERCOM_HANDLER3_FROM_INDEX(WIRE_SERCOM_INDEX(token))
#define WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(token)                            \
  WIRE_SERCOM_HANDLER_OTHER_FROM_INDEX(WIRE_SERCOM_INDEX(token))

#define WIRE_DEFINE_SINGLE_HANDLER(handler, instance)                          \
  void handler(void) { instance.onService(); }

#define WIRE_DEFINE_SAMD51_E51_HANDLERS(handler0, handler1, handler2,          \
                                        handler3, instance)                    \
  void handler0(void) { instance.onService(); }                                \
  void handler1(void) { instance.onService(); }                                \
  void handler2(void) { instance.onService(); }                                \
  void handler3(void) { instance.onService(); }

#define WIRE_DEFINE_SAME53_E54_HANDLERS(handler0, handler1, handler2,          \
                                        handlerOther, instance)                \
  void handler0(void) { instance.onService(); }                                \
  void handler1(void) { instance.onService(); }                                \
  void handler2(void) { instance.onService(); }                                \
  void handlerOther(void) { instance.onService(); }

#ifdef ARDUINO_SAMD51_E51
#define WIRE_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  WIRE_DEFINE_SAMD51_E51_HANDLERS(prefix##_IT_HANDLER_0,                       \
                                  prefix##_IT_HANDLER_1,                       \
                                  prefix##_IT_HANDLER_2,                       \
                                  prefix##_IT_HANDLER_3, instance)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  WIRE_DEFINE_SAME53_E54_HANDLERS(prefix##_IT_HANDLER_0,                       \
                                  prefix##_IT_HANDLER_1,                       \
                                  prefix##_IT_HANDLER_2,                       \
                                  prefix##_IT_HANDLER_OTHER, instance)
#else
#define WIRE_DEFINE_SERCOM_HANDLERS(prefix, instance)                          \
  WIRE_DEFINE_SINGLE_HANDLER(prefix##_IT_HANDLER, instance)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54

namespace sercomPinMux {
bool wirePinValidForSercom(uint8_t arduinoPin, uint8_t sercomIndex) {
  if (arduinoPin >= PINS_COUNT)
    return false;

  const char portLetter =
      static_cast<char>('A' + g_APinDescription[arduinoPin].ulPort);
  const uint8_t portPin =
      static_cast<uint8_t>(g_APinDescription[arduinoPin].ulPin);

#define SERCOM_PINMUX_EMIT_I2C 1
#define SERCOM_I2C_PIN(PORT_LETTER, PORT_PIN, S0, P0, S1, P1)                  \
  if ((portLetter == (PORT_LETTER)) && (portPin == (PORT_PIN)) &&              \
      (((S0) == sercomIndex) || ((S1) == sercomIndex)))                        \
    return true;
#include <SercomPinMux.inc>
#undef SERCOM_I2C_PIN
#undef SERCOM_PINMUX_EMIT_I2C

  return false;
}
} // namespace sercomPinMux

TwoWire::TwoWire(SERCOM * s, uint8_t pinSDA, uint8_t pinSCL)
{
  this->sercom = s;
  this->_uc_pinSDA=pinSDA;
  this->_uc_pinSCL=pinSCL;
  transmissionBegun = false;
  rxLength = 0;
  rxIndex = 0;
  masterIndex = 0;
  awaitingAddressAck = false;
  txnDone = false;
  txnStatus = 0;
  rxBufferPtr = rxBuffer;
  rxBufferCapacity = WIRE_BUFFER_LENGTH;
  txBufferCapacity = WIRE_BUFFER_LENGTH;
  pendingReceive = false;
  pendingReceiveLength = 0;
  txnPoolHead = 0;
}

bool TwoWire::begin(void) {
  //Master Mode
  const uint8_t sercomIndex = static_cast<uint8_t>(sercom->getSercomIndex());
  if (!sercomPinMux::wirePinValidForSercom(_uc_pinSDA, sercomIndex) ||
      !sercomPinMux::wirePinValidForSercom(_uc_pinSCL, sercomIndex))
    return false;

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);

  sercom->initMasterWIRE(TWI_CLOCK);
  sercom->enableWIRE();
  return true;
}

bool TwoWire::begin(uint16_t address, bool enableGeneralCall, uint8_t speed,
                    bool enable10Bit) {
  //Slave mode
  const uint8_t sercomIndex = static_cast<uint8_t>(sercom->getSercomIndex());
  if (!sercomPinMux::wirePinValidForSercom(_uc_pinSDA, sercomIndex) ||
      !sercomPinMux::wirePinValidForSercom(_uc_pinSCL, sercomIndex))
    return false;

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);

  sercom->initSlaveWIRE(address, enableGeneralCall, speed, enable10Bit);
  sercom->enableWIRE();
  sercom->registerReceiveWIRE(&TwoWire::onDeferredReceive, this);
  return true;
}

bool TwoWire::begin(uint8_t address, bool enableGeneralCall) {
  return begin(static_cast<uint16_t>(address), enableGeneralCall);
}

void TwoWire::setClock(uint32_t baudrate) {
  sercom->setBaudrateWIRE(baudrate);
}

void TwoWire::end() {
  sercom->resetWIRE();   // SWRST: resets hardware + clears state + drains queue
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool stopBit, uint8_t* rxBuffer,
                             void (*onComplete)(void* user, int status), void* user)
{
  if(quantity == 0)
    return 0;

  loader = SercomTxn{};

  if (rxBuffer != nullptr) {
    loader.rxPtr = rxBuffer;
    loader.length = quantity;
  } else {
    loader.rxPtr = this->rxBuffer;
    loader.length = ( quantity > WIRE_BUFFER_LENGTH) ? WIRE_BUFFER_LENGTH : quantity;
  }

  loader.config = I2C_CFG_READ | (stopBit ? I2C_CFG_STOP : 0);
  loader.address = address;

  // Allocate fresh transaction from pool and copy loader data
  SercomTxn* txn = allocateTxn();
  *txn = loader;
  txn->chainNext = false;

  // For caller callbacks, pass txn as user so callback can access
  // txn->rxPtr/length directly.
  if (onComplete) {
    txn->onComplete = onComplete;
    txn->user = (user == nullptr) ? txn : user;
  } else {
    txn->onComplete = &TwoWire::onTxnComplete;
    txn->user = this;
  }

  awaitingAddressAck = true;
  txnDone = false;

  // Enqueue the pool transaction, not the loader
  if (!sercom->enqueueWIRE(txn))
    return 0;

  if (!onComplete) {
    while (!txnDone)
      yield();

    if (txnStatus != static_cast<int>(SercomWireError::SUCCESS))
      return 0;

    rxBufferPtr = loader.rxPtr;
    rxLength = loader.length;
    rxIndex = 0;
    return rxLength;
  }

  return loader.length;
}

SercomTxn* TwoWire::allocateTxn() {
  // Simple round-robin allocation from pool
  SercomTxn* txn = &txnPool[txnPoolHead];
  txnPoolHead = (txnPoolHead + 1) % TXN_POOL_SIZE;
  *txn = SercomTxn{};  // Clear the transaction
  return txn;
}

void TwoWire::freeTxn(SercomTxn* txn) {
  // Transactions are freed when removed from SERCOM queue
  // Pool allocation is round-robin, so no explicit free needed
  (void)txn;
}

void TwoWire::beginTransmission(uint8_t address) {
  // Initialize loader as staging area for building transaction
  loader = SercomTxn{};
  loader.txPtr = nullptr;
  loader.address = address;
  txBufferCapacity = WIRE_BUFFER_LENGTH;
  transmissionBegun = true;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit, void (*onComplete)(void* user, int status), void* user)
{
  transmissionBegun = false ;

  // Allocate a fresh transaction from the pool and copy staged data from loader
  SercomTxn* txn = allocateTxn();
  *txn = loader;  // Copy staged transaction data
  txn->chainNext = false;

  // Set parameters that weren't known during beginTransmission/write
  txn->config = stopBit ? I2C_CFG_STOP : 0;
  if (onComplete) {
    txn->onComplete = onComplete;
    txn->user = (user == nullptr) ? txn : user;
  } else {
    txn->onComplete = &TwoWire::onTxnComplete;
    txn->user = this;
  }

  awaitingAddressAck = true;
  txnDone = false;

  // Enqueue the pool transaction, not the loader
  if (!sercom->enqueueWIRE(txn)) {
    if (onComplete)
      return static_cast<uint8_t>(SercomWireError::QUEUE_FULL);
    return 4;
  }

  if (!onComplete) {
    while (!txnDone)
      yield();

    SercomWireError err = static_cast<SercomWireError>(txnStatus);
    switch (err) {
      case SercomWireError::SUCCESS:
        return 0;
      case SercomWireError::DATA_TOO_LONG:
        return 1;
      case SercomWireError::NACK_ON_ADDRESS:
        return 2;
      case SercomWireError::NACK_ON_DATA:
        return 3;
      default:
        return 4;
    }
  }

  return 0;
}

size_t TwoWire::write(uint8_t ucData)
{
  if (!transmissionBegun)
    return 0;

  // Check buffer full
  if (loader.length >= txBufferCapacity)
    return 0;

  // Initialize to internal buffer if first write
  if (loader.txPtr == nullptr)
    loader.txPtr = txBuffer;

  // Append to current buffer (internal or external)
  if (loader.txPtr == txBuffer)
    txBuffer[loader.length++] = ucData;
  else
    const_cast<uint8_t*>(loader.txPtr)[loader.length++] = ucData;

  return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity, bool setExternal)
{
  if (!transmissionBegun)
    return 0;

  if (quantity == 0)
    return 0;

  // External path: require external buffer (zero-copy)
  if (setExternal) {
    if (loader.txPtr == nullptr) {
      loader.txPtr = data;
      txBufferCapacity = quantity;  // Treat quantity as both length and capacity
      loader.length = quantity;
      return quantity;
    }

    // Prevent switching from internal buffer to external mid-transaction
    if (loader.txPtr == txBuffer)
      return 0;
  }

  // Sync path: prefer internal buffer unless it overflows
  if (loader.txPtr == nullptr) {
    if (quantity <= WIRE_BUFFER_LENGTH) {
      loader.txPtr = txBuffer;
      txBufferCapacity = WIRE_BUFFER_LENGTH;
      memcpy(txBuffer, data, quantity);
      loader.length = quantity;
      return quantity;
    }

    // Large write: require external buffer
    loader.txPtr = data;
    txBufferCapacity = quantity;
    loader.length = quantity;
    return quantity;
  }

  // Appending to existing buffer
  size_t available = txBufferCapacity - loader.length;
  if (quantity > available)
    quantity = available;

  if (quantity == 0)
    return 0;

  if (loader.txPtr == txBuffer)
    memcpy(txBuffer + loader.length, data, quantity);
  else
    memcpy(const_cast<uint8_t*>(loader.txPtr) + loader.length, data, quantity);

  loader.length += quantity;
  return quantity;
}

int TwoWire::available(void)
{
  return (rxLength > rxIndex) ? (int)(rxLength - rxIndex) : 0;
}

int TwoWire::read(void)
{
  if (rxIndex >= rxLength)
    return -1;
  return rxBufferPtr[rxIndex++];
}

int TwoWire::peek(void)
{
  if (rxIndex >= rxLength)
    return -1;
  return rxBufferPtr[rxIndex];
}

void TwoWire::flush(void)
{
  // Do nothing, use endTransmission(..) to force
  // data transfer.
}

void TwoWire::onReceive(void(*function)(int))
{
  onReceiveCallback = function;
}

void TwoWire::onRequest(void(*function)(void))
{
  onRequestCallback = function;
}

void TwoWire::setRxBuffer(uint8_t* buffer, size_t length)
{
  if (buffer == nullptr || length == 0)
  {
    clearRxBuffer();
    return;
  }
  rxBufferPtr = buffer;
  rxBufferCapacity = length;
}

void TwoWire::setTxBuffer(uint8_t* buffer, size_t length)
{
  if (buffer == nullptr || length == 0) {
    loader.txPtr = nullptr;
    txBufferCapacity = WIRE_BUFFER_LENGTH;
    loader.length = 0;
    return;
  }

  loader.txPtr = buffer;
  txBufferCapacity = length;
  loader.length = 0;
}

void TwoWire::clearRxBuffer(void)
{
  if (rxBufferPtr && rxBufferCapacity > 0)
    memset(rxBufferPtr, 0, rxBufferCapacity);
  rxLength = 0;
  rxIndex = 0;
}

void TwoWire::resetRxBuffer(void)
{
  rxBufferPtr = rxBuffer;
  rxBufferCapacity = WIRE_BUFFER_LENGTH;
  clearRxBuffer();
}

uint8_t* TwoWire::getRxBuffer(void)
{
  return rxBufferPtr;
}

size_t TwoWire::getRxLength(void) const
{
  return rxLength;
}

void TwoWire::onTxnComplete(void* user, int status)
{
  if (!user)
    return;
  TwoWire* self = static_cast<TwoWire*>(user);
  self->txnStatus = status;
  self->txnDone = true;
}

void TwoWire::onDeferredReceive(void* user, int length)
{
  if (!user)
    return;
  TwoWire* self = static_cast<TwoWire*>(user);
  if (!self->pendingReceive)
    return;
  if (self->onReceiveCallback)
    self->onReceiveCallback(length);
  self->rxLength = 0;
  self->rxIndex = 0;
  self->pendingReceive = false;
  self->pendingReceiveLength = 0;
}

#if WIRE_INTERFACES_COUNT > 0
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   */
  #ifndef PERIPH_WIRE
    #define PERIPH_WIRE          sercom3
    #define WIRE_IT_HANDLER      SERCOM3_Handler
  #endif // PERIPH_WIRE
TwoWire Wire(&PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL);
#ifndef WIRE_IT_HANDLER
#define WIRE_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE)
#endif // !WIRE_IT_HANDLER
#ifndef WIRE_IT_HANDLER_0
#define WIRE_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE)
#define WIRE_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE)
#define WIRE_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE)
#ifdef ARDUINO_SAMD51_E51
#define WIRE_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE, Wire)
#endif // WIRE_INTERFACES_COUNT > 0

#if WIRE_INTERFACES_COUNT > 1
  TwoWire Wire1(&PERIPH_WIRE1, PIN_WIRE1_SDA, PIN_WIRE1_SCL);
  #ifndef WIRE1_IT_HANDLER
#define WIRE1_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE1)
#endif // !WIRE1_IT_HANDLER
#ifndef WIRE1_IT_HANDLER_0
#define WIRE1_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE1)
#define WIRE1_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE1)
#define WIRE1_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE1)
#ifdef ARDUINO_SAMD51_E51
#define WIRE1_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE1)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE1_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE1)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE1_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE1, Wire1)
#endif // WIRE_INTERFACES_COUNT > 1

#if WIRE_INTERFACES_COUNT > 2
  TwoWire Wire2(&PERIPH_WIRE2, PIN_WIRE2_SDA, PIN_WIRE2_SCL);
  #ifndef WIRE2_IT_HANDLER
#define WIRE2_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE2)
#endif // !WIRE2_IT_HANDLER
#ifndef WIRE2_IT_HANDLER_0
#define WIRE2_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE2)
#define WIRE2_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE2)
#define WIRE2_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE2)
#ifdef ARDUINO_SAMD51_E51
#define WIRE2_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE2)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE2_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE2)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE2_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE2, Wire2)
#endif // WIRE_INTERFACES_COUNT > 2

#if WIRE_INTERFACES_COUNT > 3
  TwoWire Wire3(&PERIPH_WIRE3, PIN_WIRE3_SDA, PIN_WIRE3_SCL);
  #ifndef WIRE3_IT_HANDLER
#define WIRE3_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE3)
#endif // !WIRE3_IT_HANDLER
#ifndef WIRE3_IT_HANDLER_0
#define WIRE3_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE3)
#define WIRE3_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE3)
#define WIRE3_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE3)
#ifdef ARDUINO_SAMD51_E51
#define WIRE3_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE3)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE3_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE3)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE3_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE3, Wire3)
#endif // WIRE_INTERFACES_COUNT > 3

#if WIRE_INTERFACES_COUNT > 4
  TwoWire Wire4(&PERIPH_WIRE4, PIN_WIRE4_SDA, PIN_WIRE4_SCL);
  #ifndef WIRE4_IT_HANDLER
#define WIRE4_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE4)
#endif // !WIRE4_IT_HANDLER
#ifndef WIRE4_IT_HANDLER_0
#define WIRE4_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE4)
#define WIRE4_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE4)
#define WIRE4_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE4)
#ifdef ARDUINO_SAMD51_E51
#define WIRE4_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE4)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE4_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE4)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE4_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE4, Wire4)
#endif // WIRE_INTERFACES_COUNT > 4

#if WIRE_INTERFACES_COUNT > 5
  TwoWire Wire5(&PERIPH_WIRE5, PIN_WIRE5_SDA, PIN_WIRE5_SCL);
  #ifndef WIRE5_IT_HANDLER
#define WIRE5_IT_HANDLER WIRE_SERCOM_HANDLER_FROM_TOKEN(PERIPH_WIRE5)
#endif // !WIRE5_IT_HANDLER
#ifndef WIRE5_IT_HANDLER_0
#define WIRE5_IT_HANDLER_0 WIRE_SERCOM_HANDLER0_FROM_TOKEN(PERIPH_WIRE5)
#define WIRE5_IT_HANDLER_1 WIRE_SERCOM_HANDLER1_FROM_TOKEN(PERIPH_WIRE5)
#define WIRE5_IT_HANDLER_2 WIRE_SERCOM_HANDLER2_FROM_TOKEN(PERIPH_WIRE5)
#ifdef ARDUINO_SAMD51_E51
#define WIRE5_IT_HANDLER_3 WIRE_SERCOM_HANDLER3_FROM_TOKEN(PERIPH_WIRE5)
#elif defined(ARDUINO_SAME53_E54)
#define WIRE5_IT_HANDLER_OTHER WIRE_SERCOM_HANDLER_OTHER_FROM_TOKEN(PERIPH_WIRE5)
#endif // ARDUINO_SAMD51_E51 / ARDUINO_SAME53_E54
#endif // !WIRE5_IT_HANDLER_0
WIRE_DEFINE_SERCOM_HANDLERS(WIRE5, Wire5)
#endif // WIRE_INTERFACES_COUNT > 5
