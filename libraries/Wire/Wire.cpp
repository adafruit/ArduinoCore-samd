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
  txLength = 0;
  txIndex = 0;
  masterIndex = 0;
  awaitingAddressAck = false;
  txnDone = false;
  txnStatus = 0;
  rxBufferPtr = rxBuffer;
  rxBufferCapacity = WIRE_RX_BUFFER_LENGTH;
  pendingReceive = false;
  pendingReceiveLength = 0;
}

void TwoWire::begin(void) {
  //Master Mode
  sercom->initMasterWIRE(TWI_CLOCK);
  sercom->enableWIRE();
  sercom->registerWireReceive(&TwoWire::onDeferredReceive, this);

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);
}

void TwoWire::begin(uint8_t address, bool enableGeneralCall) {
  //Slave mode
  sercom->initSlaveWIRE(address, enableGeneralCall);
  sercom->enableWIRE();
  sercom->registerWireReceive(&TwoWire::onDeferredReceive, this);

  pinPeripheral(_uc_pinSDA, g_APinDescription[_uc_pinSDA].ulPinType);
  pinPeripheral(_uc_pinSCL, g_APinDescription[_uc_pinSCL].ulPinType);
}

void TwoWire::setClock(uint32_t baudrate) {
  sercom->disableWIRE();
  sercom->initMasterWIRE(baudrate);
  sercom->enableWIRE();
}

void TwoWire::end() {
  sercom->disableWIRE();
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity, bool stopBit)
{
  return requestFrom(address, quantity, static_cast<uint8_t*>(nullptr), stopBit);
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity, uint8_t* rxBuffer, bool stopBit)
{
  if(quantity == 0)
  {
    return 0;
  }

  if (rxBuffer != nullptr) {
    rxBufferPtr = rxBuffer;
    rxBufferCapacity = quantity;
  } else {
    rxBufferPtr = this->rxBuffer;
    rxBufferCapacity = WIRE_RX_BUFFER_LENGTH;
    if (quantity > rxBufferCapacity)
      quantity = rxBufferCapacity;
  }

  rxLength = 0;
  rxIndex = 0;

  txn.config = I2C_CFG_READ | (stopBit ? I2C_CFG_STOP : 0);
  txn.address = address;
  txn.length = quantity;
  txn.txPtr = nullptr;
  txn.rxPtr = rxBufferPtr;
  txn.onComplete = &TwoWire::onTxnComplete;
  txn.user = this;
  txnDone = false;
  txnStatus = static_cast<int>(SercomWireError::SUCCESS);
  masterIndex = 0;
  awaitingAddressAck = true;

  if (!sercom->enqueueWIRE(&txn))
    return 0;

  while (!txnDone) {
    yield();
  }

  if (txnStatus != static_cast<int>(SercomWireError::SUCCESS))
    return 0;

  rxLength = quantity;
  return rxLength;
}

uint8_t TwoWire::requestFrom(uint8_t address, size_t quantity)
{
  return requestFrom(address, quantity, true);
}

void TwoWire::beginTransmission(uint8_t address) {
  // save address of target and clear buffer
  txAddress = address;
  txLength = 0;
  txIndex = 0;

  transmissionBegun = true;
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t TwoWire::endTransmission(bool stopBit)
{
  transmissionBegun = false ;

  txn.config = stopBit ? I2C_CFG_STOP : 0;
  txn.address = txAddress;
  txn.length = txLength;
  txn.txPtr = txBuffer;
  txn.rxPtr = nullptr;
  txn.onComplete = &TwoWire::onTxnComplete;
  txn.user = this;
  txnDone = false;
  txnStatus = static_cast<int>(SercomWireError::SUCCESS);
  masterIndex = 0;
  awaitingAddressAck = true;

  if (!sercom->enqueueWIRE(&txn))
    return static_cast<uint8_t>(SercomWireError::QUEUE_FULL);

  while (!txnDone) {
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
    default:
      return 4;
  }
}

uint8_t TwoWire::endTransmission()
{
  return endTransmission(true);
}

size_t TwoWire::write(uint8_t ucData)
{
  // No writing, without begun transmission or a full buffer
  if ( !transmissionBegun || txLength >= WIRE_TX_BUFFER_LENGTH )
  {
    return 0 ;
  }

  txBuffer[txLength++] = ucData;

  return 1 ;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  //Try to store all data
  for(size_t i = 0; i < quantity; ++i)
  {
    //Return the number of data stored, when the buffer is full (if write return 0)
    if(!write(data[i]))
      return i;
  }

  //All data stored
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
  rxBufferCapacity = WIRE_RX_BUFFER_LENGTH;
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

void TwoWire::onService(void)
{
  SercomTxn* t = &txn;
  uint8_t flags = (uint8_t)sercom->getINTFLAG();

  if (flags == 0)
    return;

  if (sercom->isRXNackReceivedWIRE())
  {
    sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
    SercomWireError err = awaitingAddressAck ? SercomWireError::NACK_ON_ADDRESS
                                              : SercomWireError::NACK_ON_DATA;
    sercom->deferStopWIRE(err);
    return;
  }

  if (sercom->isMasterWIRE())
  {
    if (flags & SERCOM_I2CM_INTFLAG_ERROR)
    {
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
      uint16_t status = (uint16_t)sercom->getSTATUS();
      uint8_t busState = (status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos;
      SercomWireError err;

      if (status & SERCOM_I2CM_STATUS_ARBLOST)
        err = SercomWireError::ARBITRATION_LOST;
      else if (status & SERCOM_I2CM_STATUS_BUSERR)
        err = SercomWireError::BUS_ERROR;
      else if (status & SERCOM_I2CM_STATUS_MEXTTOUT)
        err = SercomWireError::MASTER_TIMEOUT;
      else if (status & SERCOM_I2CM_STATUS_SEXTTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;
      else if (status & SERCOM_I2CM_STATUS_LENERR)
        err = SercomWireError::LENGTH_ERROR;
      else if (busState == 0x0)
        err = SercomWireError::BUS_STATE_UNKNOWN;
      else
        err = SercomWireError::UNKNOWN_ERROR;

      sercom->clearINTFLAG();
      sercom->deferStopWIRE(err);
      return;
    }

    bool ok = (t->config & I2C_CFG_READ) ? sercom->readDataWIRE()
                                         : sercom->sendDataWIRE();

    if (ok){
      awaitingAddressAck = false;
#ifdef USE_ZERODMA
    if (t->length > 0 && t->length < 256 && (t->config & I2C_CFG_STOP)) 
      return;
#endif
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_READ);
      return;
    }

    // Transaction complete
    if (t->config & I2C_CFG_STOP)
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
    else
      sercom->clearINTFLAG();

    sercom->deferStopWIRE(SercomWireError::SUCCESS);
    return;
  }

  if ( sercom->isSlaveWIRE() )
  {
    if (flags & SERCOM_I2CS_INTFLAG_ERROR)
    {
      uint16_t status = (uint16_t)sercom->getSTATUS();
      SercomWireError err;

      if (status & SERCOM_I2CS_STATUS_BUSERR)
        err = SercomWireError::BUS_ERROR;
      else if (status & SERCOM_I2CS_STATUS_COLL)
        err = SercomWireError::ARBITRATION_LOST;
      else if (status & SERCOM_I2CS_STATUS_SEXTTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;
      else if (status & SERCOM_I2CS_STATUS_LOWTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;
      else
        err = SercomWireError::UNKNOWN_ERROR;

      sercom->clearINTFLAG();
      sercom->deferStopWIRE(err);
      return;
    }

    if(sercom->isStopDetectedWIRE() || 
        (sercom->isAddressMatch() && sercom->isRestartDetectedWIRE() && !sercom->isMasterReadOperationWIRE())) //Stop or Restart detected
    {
      sercom->prepareAckBitWIRE();
      sercom->prepareCommandBitsWire(0x03);

      pendingReceive = true;
      pendingReceiveLength = available();
      sercom->deferWireReceive(pendingReceiveLength);
    }
    else if(sercom->isAddressMatch())  //Address Match
    {
      sercom->prepareAckBitWIRE();
      sercom->prepareCommandBitsWire(0x03);

      if(sercom->isMasterReadOperationWIRE()) //Is a request ?
      {
        txLength = 0;
        txIndex = 0;

        transmissionBegun = true;

        // onRequestCallback runs in ISR context here. Deferring to PendSV
        // would require stalling DRDY or returning 0xFF until the buffer is filled.
        if(onRequestCallback)
          onRequestCallback();

        t->txPtr = txBuffer;
        t->rxPtr = nullptr;
        t->length = txLength;
        t->config = I2C_CFG_READ;
        bool useDma = (txLength > 0 && txLength < 256);
        sercom->setWireTxn(t, txLength, useDma);
      } else {
        rxLength = 0;
        rxIndex = 0;
        t->txPtr = nullptr;
        t->rxPtr = rxBufferPtr;
        t->length = rxBufferCapacity;
        t->config = 0;
        bool useDma = (rxBufferPtr != rxBuffer) && (rxBufferCapacity > 0 && rxBufferCapacity < 256);
        sercom->setWireTxn(t, rxBufferCapacity, useDma);
        if (useDma)
          rxLength = rxBufferCapacity;
      }
    }
    else if(sercom->isDataReadyWIRE())
    {
      bool ok = sercom->isMasterReadOperationWIRE()
        ? sercom->sendDataWIRE()
        : sercom->readDataWIRE();

      if (ok) {
#ifdef USE_ZERODMA
        if (sercom->isWireDma())
          return;
#endif
        if (!sercom->isMasterReadOperationWIRE())
          rxLength++;
        sercom->prepareCommandBitsWire(0x03);
        return;
      }

      if (!sercom->isMasterReadOperationWIRE()) {
        sercom->prepareNackBitWIRE();
        sercom->prepareCommandBitsWire(0x03);
        return;
      }

      sercom->I2CS.DATA.reg = 0xFF;
      sercom->prepareCommandBitsWire(0x03);
      return;
    }
  }
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
