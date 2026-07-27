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

#ifndef TwoWire_h
#define TwoWire_h

#include "Stream.h"
#include "variant.h"

#include "RingBuffer.h"
#include "SERCOM.h"
#include <stddef.h>

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1

// NOTE: SAMD21/SAMD51 silicon errata: when I2C master uses SCLSM=1, CTRLB.CMD
// (STOP/RESTART) is ignored, so interrupt-driven byte mode cannot reliably end
// transfers or issue repeated starts. Hs-mode requires SCLSM=1, therefore Hs-mode
// is DMA-only and STOP-only (no repeated starts). The non-DMA Wire path should
// not enable Hs-mode. Also per errata, do not enable QCEN when SCLSM=1 (bus error).

class TwoWire : public Stream
{
public:
  using RequestCompletion = void (*)(void *user, SercomWireError result);
  TwoWire(SERCOM *s, uint8_t pinSDA, uint8_t pinSCL);
  bool begin();
  bool begin(uint8_t, bool enableGeneralCall = false);
  bool begin(uint16_t, bool enableGeneralCall, uint8_t speed = 0x0,
             bool enable10Bit = false);
  void end();
  void setClock(uint32_t);

  void beginTransmission(uint8_t);
  // If onComplete is nullptr, this blocks for legacy sync behavior.
  // If onComplete is non-null, this enqueues and returns immediately (async).
  uint8_t endTransmission(bool stopBit = true,
                          void (*onComplete)(void *user, int status) = nullptr,
                          void *user = nullptr);
  bool abortTransmission(
      int status = static_cast<int>(SercomWireError::MASTER_TIMEOUT));

  // If onComplete is nullptr, this blocks for legacy sync behavior.
  // If onComplete is non-null, this enqueues and returns immediately (async).
  // If rxBuffer is nullptr, the internal buffer is used; otherwise rxBuffer is
  // used.
  uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit = true,
                      uint8_t *rxBuffer = nullptr,
                      void (*onComplete)(void *user, int status) = nullptr,
                      void *user = nullptr);

  size_t write(uint8_t data);
  // 3-arg write: when setExternal=true, data is used directly (zero-copy) and
  // quantity is treated as both length and capacity; subsequent write() calls
  // return 0. For streaming > WIRE_BUFFER_LENGTH or async usage, call
  // setTxBuffer() before write() on every transaction.
  size_t write(const uint8_t *data, size_t quantity, bool setExternal = false);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void (*)(int));
    void onReceive(void (*)(SercomWireError, int));
    void onRequest(void (*)(void));
    void onRequest(void (*)(void), RequestCompletion completion,
                   void *user = nullptr);
    void setRxBuffer(uint8_t *buffer, size_t length);
    void setTxBuffer(uint8_t *buffer, size_t length);
    void clearRxBuffer(void);
    void resetRxBuffer(void);
    uint8_t *getRxBuffer(void);
    size_t getRxLength(void) const;
    inline SERCOM *getSercom(void) { return sercom; }
    inline const SERCOM *getSercom(void) const { return sercom; }

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    inline void onService(void);

private:
  SERCOM *sercom;
    uint8_t _uc_pinSDA;
    uint8_t _uc_pinSCL;

    bool transmissionBegun;

    // RX/TX buffers (sync compatibility, async staging)
    static constexpr size_t WIRE_BUFFER_LENGTH = 255;
    uint8_t rxBuffer[WIRE_BUFFER_LENGTH];
    uint8_t txBuffer[WIRE_BUFFER_LENGTH];
  uint8_t *rxBufferPtr;
    size_t rxBufferCapacity;
    size_t rxLength;
    size_t rxIndex;
    size_t txBufferCapacity;
    size_t masterIndex;
    bool awaitingAddressAck;
    volatile bool txnDone;
    volatile int txnStatus;
    SercomTxn slaveTxn;
    SercomTxn loader; // Staging area for building transactions

    // Transaction pool for async operations (matches SERCOM queue depth)
    static constexpr size_t TXN_POOL_SIZE = 8;
    SercomTxn txnPool[TXN_POOL_SIZE];
    uint8_t txnPoolHead;

    SercomTxn *allocateTxn();
    void freeTxn(SercomTxn *txn);

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);
    void (*onReceiveResultCallback)(SercomWireError, int);
    RequestCompletion requestCompletionCallback;
    void *requestCompletionUser;

    static void onTxnComplete(void *user, int status);
    static void onDeferredReceive(void *user);
    static void onDeferredRequest(void *user);
    static void onSlaveReceiveComplete(void *user, int status);
    static void onSlaveRequestComplete(void *user, int status);

    // TWI clock frequency
    static const uint32_t TWI_CLOCK = 100000;
};

#if WIRE_INTERFACES_COUNT > 0
extern TwoWire Wire;
#endif
#if WIRE_INTERFACES_COUNT > 1
extern TwoWire Wire1;
#endif
#if WIRE_INTERFACES_COUNT > 2
extern TwoWire Wire2;
#endif
#if WIRE_INTERFACES_COUNT > 3
extern TwoWire Wire3;
#endif
#if WIRE_INTERFACES_COUNT > 4
extern TwoWire Wire4;
#endif
#if WIRE_INTERFACES_COUNT > 5
extern TwoWire Wire5;
#endif

inline void TwoWire::onService(void)
{
  uint8_t flags = (uint8_t)sercom->getINTFLAG();
  uint16_t status = (uint16_t)sercom->getSTATUS();
  bool isMaster = sercom->isMasterWIRE();

  if (!isMaster && !sercom->isSlaveWIRE()) {
    sercom->clearINTFLAG();
    return;
  }

  if (flags == 0)
    return;

#if defined(__SAME53__) || defined(__SAME54__)
  const bool rxNack = status & SERCOM_I2CM_STATUS_RXNACK_Msk;
  const bool wireError = flags & SERCOM_I2CM_INTFLAG_ERROR_Msk;
  const bool slaveDrdy =
      !isMaster && (flags & SERCOM_I2CS_INTFLAG_DRDY_Msk);
#else
  const bool rxNack = status & SERCOM_I2CM_STATUS_RXNACK;
  const bool wireError = flags & SERCOM_I2CM_INTFLAG_ERROR;
  const bool slaveDrdy = !isMaster && (flags & SERCOM_I2CS_INTFLAG_DRDY);
#endif // __SAME53__ / __SAME54__

  SercomTxn *activeTxn = sercom->getCurrentTxnWIRE();
  const bool rxSlaveNack =
      rxNack && activeTxn && (activeTxn->config & I2C_CFG_READ) != 0;
  const bool isRxNack = isMaster ? rxNack : rxSlaveNack;

  if (isRxNack || wireError) {
#if defined(__SAME53__) || defined(__SAME54__)
    const bool arbitrationLost = status & SERCOM_I2CM_STATUS_ARBLOST_Msk;
    const bool busError = status & SERCOM_I2CM_STATUS_BUSERR_Msk;
    const bool lowTimeout = status & SERCOM_I2CM_STATUS_LOWTOUT_Msk;
    const bool slaveExtendTimeout = status & SERCOM_I2CM_STATUS_SEXTTOUT_Msk;
    const bool masterExtendTimeout =
        isMaster && (status & SERCOM_I2CM_STATUS_MEXTTOUT_Msk);
    const bool lengthError =
        isMaster && (status & SERCOM_I2CM_STATUS_LENERR_Msk);
#else
    const bool arbitrationLost = status & SERCOM_I2CM_STATUS_ARBLOST;
    const bool busError = status & SERCOM_I2CM_STATUS_BUSERR;
    const bool lowTimeout = status & SERCOM_I2CM_STATUS_LOWTOUT;
    const bool slaveExtendTimeout = status & SERCOM_I2CM_STATUS_SEXTTOUT;
    const bool masterExtendTimeout =
        isMaster && (status & SERCOM_I2CM_STATUS_MEXTTOUT);
    const bool lengthError = isMaster && (status & SERCOM_I2CM_STATUS_LENERR);
#endif // __SAME53__ / __SAME54__
    const bool masterTimeout = isMaster && (masterExtendTimeout || lowTimeout);
    const bool slaveTimeout = slaveExtendTimeout || (!isMaster && lowTimeout);

    SercomWireError error = SercomWireError::UNKNOWN_ERROR;

    if (isRxNack) {
      error = isMaster && awaitingAddressAck ? SercomWireError::NACK_ON_ADDRESS
                                             : SercomWireError::NACK_ON_DATA;
    } else if (arbitrationLost) {
      error = SercomWireError::ARBITRATION_LOST;
    } else if (busError) {
      error = SercomWireError::BUS_ERROR;
    } else if (masterTimeout) {
      error = SercomWireError::MASTER_TIMEOUT;
    } else if (slaveTimeout) {
      error = SercomWireError::SLAVE_TIMEOUT;
    } else if (lengthError) {
      error = SercomWireError::LENGTH_ERROR;
    } else if (isMaster) {
#if defined(__SAME53__) || defined(__SAME54__)
      const uint8_t busState = (status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
                               SERCOM_I2CM_STATUS_BUSSTATE_Pos;
#else
      const uint8_t busState = (status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
                               SERCOM_I2CM_STATUS_BUSSTATE_Pos;
#endif // __SAME53__ / __SAME54__
      if (busState == WIRE_UNKNOWN_STATE)
        error = SercomWireError::BUS_STATE_UNKNOWN;
    }

    if (isMaster && !arbitrationLost)
      sercom->prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
    if (wireError)
      sercom->clearINTFLAG();
    sercom->deferStopWIRE(error);
    return;
  }

  if (isMaster) {
    SercomTxn *txn = activeTxn;
    if (!txn) {
      sercom->clearINTFLAG();
      return;
    }

    bool isRead = (txn->config & I2C_CFG_READ);

    if (sercom->getTxnIndexWIRE() < sercom->getTxnLengthWIRE()) {
      bool more = isRead ? sercom->readDataWIRE() : sercom->sendDataWIRE();
      awaitingAddressAck = false;
      if (!isRead || more)
        return;
    }

    if ((txn->config & I2C_CFG_STOP) && !isRead)
      sercom->prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
    else
      sercom->clearINTFLAG();

    awaitingAddressAck = true;
    sercom->deferStopWIRE(SercomWireError::SUCCESS);
    return;
  } else {
#if defined(__SAME53__) || defined(__SAME54__)
    bool isMasterRead = (status & SERCOM_I2CS_STATUS_DIR_Msk);
    bool prec = (flags & SERCOM_I2CS_INTFLAG_PREC_Msk);
    bool amatch = (flags & SERCOM_I2CS_INTFLAG_AMATCH_Msk);
#else
    bool isMasterRead =
        (status & SERCOM_I2CS_STATUS_DIR); // Master Read / Slave Transmit
    bool prec = (flags & SERCOM_I2CS_INTFLAG_PREC);        // Stop detected
    bool amatch = (flags & SERCOM_I2CS_INTFLAG_AMATCH);    // Address Match detected
#endif // __SAME53__ / __SAME54__

    // A new address match terminates an active slave receive even when the
    // master uses a repeated START instead of STOP. Queue receive completion
    // before the follow-up setup so a combined write/read transaction publishes
    // its request bytes before onRequest prepares the response.
    if (amatch) {
      if (activeTxn && (activeTxn->config & I2C_CFG_READ) == 0)
        sercom->deferReceiveCompleteWIRE();
      if (isMasterRead)
        sercom->deferRequestWIRE();
      else
        sercom->deferReceiveWIRE();
    }

    // Preserve the interrupt-driven slave byte engine both when DMA is not
    // compiled and when this transaction is not DMA-eligible.
    if (slaveDrdy && !amatch && !sercom->isDmaWIRE()) {
      SercomTxn *slaveTxn = activeTxn;
      if (!slaveTxn) {
        sercom->clearINTFLAG();
        return;
      }

      if (slaveTxn->config & I2C_CFG_READ) {
        if (sercom->getTxnIndexWIRE() < sercom->getTxnLengthWIRE())
          sercom->sendDataWIRE();
        else {
          sercom->prepareSlaveCommandBitsWIRE(WIRE_SLAVE_ACT_COMPLETE);
          sercom->deferStopWIRE(SercomWireError::SUCCESS);
        }
      } else if (sercom->getTxnIndexWIRE() < sercom->getTxnLengthWIRE()) {
        sercom->readDataWIRE();
      } else {
        sercom->prepareNackBitWIRE();
      }
      return;
    }

    // A short slave receive ends at STOP before its capacity-sized descriptor
    // reaches zero. PREC finalizes the actual DMA byte count; it is not an
    // error and does not enter stopTransmissionWIRE().
    SercomTxn *slaveTxn = activeTxn;
    const bool activeSlaveReceive =
        prec && !amatch && slaveTxn &&
        (slaveTxn->config & I2C_CFG_READ) == 0;
    if (activeSlaveReceive)
      sercom->deferReceiveCompleteWIRE();
    else if (prec)
      sercom->clearINTFLAG();
  }
}

#endif
