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
#endif

#include "Wire.h"

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

void TwoWire::begin(void) {
  //Master Mode
  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);

  sercom->initMasterWIRE(TWI_CLOCK);
  sercom->enableWIRE();
}

void TwoWire::begin(uint16_t address, bool enableGeneralCall, uint8_t speed, bool enable10Bit) {
  //Slave mode
  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);

  sercom->initSlaveWIRE(address, enableGeneralCall, speed, enable10Bit);
  sercom->enableWIRE();
  sercom->registerReceiveWIRE(&TwoWire::onDeferredReceive, this);
}

void TwoWire::begin(uint8_t address, bool enableGeneralCall) {
  begin(static_cast<uint16_t>(address), enableGeneralCall);
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
  loader.onComplete = onComplete ? onComplete : &TwoWire::onTxnComplete;
  
  // Allocate fresh transaction from pool and copy loader data
  SercomTxn* txn = allocateTxn();
  *txn = loader;
  txn->chainNext = false;
  
  // For async callbacks, pass txn as user so callback can access txn->rxPtr/length directly
  // For sync calls, pass 'this' so onTxnComplete can update txnStatus/txnDone
  if (onComplete)
    txn->user = (user == nullptr) ? txn : user;
  else
    txn->user = this;
  
  awaitingAddressAck = true;
  txnDone = false;

  // Enqueue the pool transaction, not the loader
  if (!sercom->enqueueWIRE(txn))
    return 0;

  if (!onComplete) {
    // Wait for transaction to complete (onTxnComplete sets txnDone) with timeout
    uint32_t startMillis = millis();
    const uint32_t timeout = 1000; // 1 second timeout
    while (!txnDone) {
      if (millis() - startMillis > timeout)
        return 0;

      yield();
    }

    if (txnStatus != static_cast<int>(SercomWireError::SUCCESS))
      return 0;

    // Set up pointers for Wire.available()/read() to access the data
    // txn->rxPtr already points to this->rxBuffer (from loader copy)
    rxBufferPtr = loader.rxPtr;
    rxLength = quantity;
    rxIndex = 0;
    return rxLength;
  }

  return quantity;
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
  txn->onComplete = onComplete ? onComplete : &TwoWire::onTxnComplete;
  if (onComplete)
    txn->user = (user == nullptr) ? txn : user;
  else
    txn->user = this;
  
  awaitingAddressAck = true;
  txnDone = false;

  // Enqueue the pool transaction, not the loader
  if (!sercom->enqueueWIRE(txn))
    return static_cast<uint8_t>(SercomWireError::QUEUE_FULL);

  if (!onComplete) {
    // Wait for transaction to complete (onTxnComplete sets txnDone) with timeout
    uint32_t startMillis = millis();
    const uint32_t timeout = 1000; // 1 second timeout
    while (!txnDone) {
      if (millis() - startMillis > timeout)
        return 4; // OTHER error
  
      yield();
    }

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
      default: // OTHER
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

  void WIRE_IT_HANDLER(void) {
    Wire.onService();
  }

  #if defined(__SAMD51__)
    void WIRE_IT_HANDLER_0(void) { Wire.onService(); }
    void WIRE_IT_HANDLER_1(void) { Wire.onService(); }
    void WIRE_IT_HANDLER_2(void) { Wire.onService(); }
    void WIRE_IT_HANDLER_3(void) { Wire.onService(); }
  #endif // __SAMD51__
#endif

#if WIRE_INTERFACES_COUNT > 1
  TwoWire Wire1(&PERIPH_WIRE1, PIN_WIRE1_SDA, PIN_WIRE1_SCL);

  void WIRE1_IT_HANDLER(void) {
    Wire1.onService();
  }

  #if defined(__SAMD51__)
    void WIRE1_IT_HANDLER_0(void) { Wire1.onService(); }
    void WIRE1_IT_HANDLER_1(void) { Wire1.onService(); }
    void WIRE1_IT_HANDLER_2(void) { Wire1.onService(); }
    void WIRE1_IT_HANDLER_3(void) { Wire1.onService(); }
  #endif // __SAMD51__
#endif

#if WIRE_INTERFACES_COUNT > 2
  TwoWire Wire2(&PERIPH_WIRE2, PIN_WIRE2_SDA, PIN_WIRE2_SCL);

  void WIRE2_IT_HANDLER(void) {
    Wire2.onService();
  }

  #if defined(__SAMD51__)
    void WIRE2_IT_HANDLER_0(void) { Wire2.onService(); }
    void WIRE2_IT_HANDLER_1(void) { Wire2.onService(); }
    void WIRE2_IT_HANDLER_2(void) { Wire2.onService(); }
    void WIRE2_IT_HANDLER_3(void) { Wire2.onService(); }
  #endif // __SAMD51__
#endif

#if WIRE_INTERFACES_COUNT > 3
  TwoWire Wire3(&PERIPH_WIRE3, PIN_WIRE3_SDA, PIN_WIRE3_SCL);

  void WIRE3_IT_HANDLER(void) {
    Wire3.onService();
  }

  #if defined(__SAMD51__)
    void WIRE3_IT_HANDLER_0(void) { Wire3.onService(); }
    void WIRE3_IT_HANDLER_1(void) { Wire3.onService(); }
    void WIRE3_IT_HANDLER_2(void) { Wire3.onService(); }
    void WIRE3_IT_HANDLER_3(void) { Wire3.onService(); }
  #endif // __SAMD51__
#endif

#if WIRE_INTERFACES_COUNT > 4
  TwoWire Wire4(&PERIPH_WIRE4, PIN_WIRE4_SDA, PIN_WIRE4_SCL);

  void WIRE4_IT_HANDLER(void) {
    Wire4.onService();
  }

  #if defined(__SAMD51__)
    void WIRE4_IT_HANDLER_0(void) { Wire4.onService(); }
    void WIRE4_IT_HANDLER_1(void) { Wire4.onService(); }
    void WIRE4_IT_HANDLER_2(void) { Wire4.onService(); }
    void WIRE4_IT_HANDLER_3(void) { Wire4.onService(); }
  #endif // __SAMD51__
#endif

#if WIRE_INTERFACES_COUNT > 5
  TwoWire Wire5(&PERIPH_WIRE5, PIN_WIRE5_SDA, PIN_WIRE5_SCL);

  void WIRE5_IT_HANDLER(void) {
    Wire5.onService();
  }

  #if defined(__SAMD51__)
    void WIRE5_IT_HANDLER_0(void) { Wire5.onService(); }
    void WIRE5_IT_HANDLER_1(void) { Wire5.onService(); }
    void WIRE5_IT_HANDLER_2(void) { Wire5.onService(); }
    void WIRE5_IT_HANDLER_3(void) { Wire5.onService(); }
  #endif // __SAMD51__
#endif
