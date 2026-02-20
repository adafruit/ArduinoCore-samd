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

#include "SERCOM.h"
#include "RingBuffer.h"
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
    TwoWire(SERCOM *s, uint8_t pinSDA, uint8_t pinSCL);
    void begin();
    void begin(uint8_t, bool enableGeneralCall = false);
    void begin(uint16_t, bool enableGeneralCall, uint8_t speed = 0x0, bool enable10Bit = false);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t);
    // If onComplete is nullptr, this blocks for legacy sync behavior.
    // If onComplete is non-null, this enqueues and returns immediately (async).
    uint8_t endTransmission(bool stopBit = true,
                            void (*onComplete)(void* user, int status) = nullptr,
                            void* user = nullptr);

    // If onComplete is nullptr, this blocks for legacy sync behavior.
    // If onComplete is non-null, this enqueues and returns immediately (async).
    // If rxBuffer is nullptr, the internal buffer is used; otherwise rxBuffer is used.
    uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit = true,
                        uint8_t* rxBuffer = nullptr,
                        void (*onComplete)(void* user, int status) = nullptr,
                        void* user = nullptr);

    size_t write(uint8_t data);
    // 3-arg write: when setExternal=true, data is used directly (zero-copy) and
    // quantity is treated as both length and capacity; subsequent write() calls return 0.
    // For streaming > WIRE_BUFFER_LENGTH or async usage, call setTxBuffer() before write()
    // on every transaction.
    size_t write(const uint8_t * data, size_t quantity, bool setExternal = false);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));
    void setRxBuffer(uint8_t* buffer, size_t length);
    void setTxBuffer(uint8_t* buffer, size_t length);
    void clearRxBuffer(void);
    void resetRxBuffer(void);
    uint8_t* getRxBuffer(void);
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
    SERCOM * sercom;
    uint8_t _uc_pinSDA;
    uint8_t _uc_pinSCL;

    bool transmissionBegun;

    // RX/TX buffers (sync compatibility, async staging)
    static constexpr size_t WIRE_BUFFER_LENGTH = 255;
    uint8_t rxBuffer[WIRE_BUFFER_LENGTH];
    uint8_t txBuffer[WIRE_BUFFER_LENGTH];
    uint8_t* rxBufferPtr;
    size_t rxBufferCapacity;
    size_t rxLength;
    size_t rxIndex;
    size_t txBufferCapacity;
    size_t masterIndex;
    bool awaitingAddressAck;
    volatile bool txnDone;
    volatile int txnStatus;
    bool pendingReceive;
    int pendingReceiveLength;
    SercomTxn slaveTxn;
    SercomTxn loader;  // Staging area for building transactions
    
    // Transaction pool for async operations (matches SERCOM queue depth)
    static constexpr size_t TXN_POOL_SIZE = 8;
    SercomTxn txnPool[TXN_POOL_SIZE];
    uint8_t txnPoolHead;
    
    SercomTxn* allocateTxn();
    void freeTxn(SercomTxn* txn);

    // Callback user functions
    void (*onRequestCallback)(void);
    void (*onReceiveCallback)(int);

    static void onTxnComplete(void* user, int status);
    static void onDeferredReceive(void* user, int length);

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

  if ((!isMaster && !sercom->isSlaveWIRE()) || flags == 0) {
      sercom->clearINTFLAG();
      return;
    }

  if (status & SERCOM_I2CM_STATUS_RXNACK) {
    sercom->prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
    SercomWireError err = awaitingAddressAck ? SercomWireError::NACK_ON_ADDRESS
                                             : SercomWireError::NACK_ON_DATA;
    sercom->deferStopWIRE(err);
    return;
  }

  if (isMaster) {
    SercomTxn* txn = sercom->getCurrentTxnWIRE();
    if (!txn) {
      sercom->clearINTFLAG();
      return;
    }

    if (flags & SERCOM_I2CM_INTFLAG_ERROR) {
      sercom->prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
      uint8_t busState = (status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos;
      SercomWireError err = SercomWireError::UNKNOWN_ERROR;

      if (status & SERCOM_I2CM_STATUS_ARBLOST)
        err = SercomWireError::ARBITRATION_LOST;
      if (status & SERCOM_I2CM_STATUS_BUSERR)
        err = SercomWireError::BUS_ERROR;
      if (status & SERCOM_I2CM_STATUS_MEXTTOUT)
        err = SercomWireError::MASTER_TIMEOUT;
      if (status & SERCOM_I2CM_STATUS_SEXTTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;
      if (status & SERCOM_I2CM_STATUS_LENERR)
        err = SercomWireError::LENGTH_ERROR;
      if (busState == 0x0)
        err = SercomWireError::BUS_STATE_UNKNOWN;
      
      sercom->clearINTFLAG();
      sercom->deferStopWIRE(err);
      return;
    }

    bool isRead = (txn->config & I2C_CFG_READ);

    if (sercom->getTxnIndexWIRE() < sercom->getTxnLengthWIRE()) {
      isRead ? sercom->readDataWIRE() : sercom->sendDataWIRE();
      awaitingAddressAck = false;
      return;
    }

    if ((txn->config & I2C_CFG_STOP) && !isRead)
      sercom->prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
    else
      sercom->clearINTFLAG();

    awaitingAddressAck = true;
    sercom->deferStopWIRE(SercomWireError::SUCCESS);
    return;
  }
  else {
    if (flags & SERCOM_I2CS_INTFLAG_ERROR) {
      SercomWireError err = SercomWireError::UNKNOWN_ERROR;;

      if (status & SERCOM_I2CS_STATUS_BUSERR)
        err = SercomWireError::BUS_ERROR;
      if (status & SERCOM_I2CS_STATUS_COLL)
        err = SercomWireError::ARBITRATION_LOST;
      if (status & SERCOM_I2CS_STATUS_SEXTTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;
      if (status & SERCOM_I2CS_STATUS_LOWTOUT)
        err = SercomWireError::SLAVE_TIMEOUT;

      sercom->clearINTFLAG();
      sercom->deferStopWIRE(err);
      return;
    }

    // To avoid unnecessary clock cycles for register reads, avoid using inline getters
    bool isMasterRead = (status & SERCOM_I2CS_STATUS_DIR);  // Master Read / Slave Transmit
    bool sr = (status & SERCOM_I2CS_STATUS_SR);             // Repeated Start detected
    bool prec = (flags & SERCOM_I2CS_INTFLAG_PREC);         // Stop detected
    bool amatch = (flags & SERCOM_I2CS_INTFLAG_AMATCH);     // Address Match detected
    bool drdy = (flags & SERCOM_I2CS_INTFLAG_DRDY);         // Data Ready detected
        
    // Stop or Restart detected - defer receive callback
    if(prec || (amatch && sr && !isMasterRead))
    {
      pendingReceive = true;
      pendingReceiveLength = available();
      sercom->deferReceiveWIRE(pendingReceiveLength);
      return;
    }
    
    // Address Match - setup transaction
    // AACKEN enabled: address ACK is automatic, no manual ACK/clear needed
    else if(amatch)
    {
      if(isMasterRead) // Master Read / Slave TX
      {
        // onRequestCallback runs in ISR context here. Deferring to PendSV
        // would require stalling DRDY or returning 0xFF until the buffer is filled.
        // onRequestCallback is what will set TwoWire::slaveTxn for the transaction.
        if(onRequestCallback)
          onRequestCallback();
        
        // Ensure callback actually set slaveTxn.length; if not, stall with 0-length txn
        if (slaveTxn.length == 0) 
          return;

        if (!(slaveTxn.config & I2C_CFG_READ)) 
          slaveTxn.config |= I2C_CFG_READ;
      }
      else // Master Write / Slave RX
      {
        // rxLength needs to be set in the rxCallback so the user has runtime control
        // setting rxLength to 0 will default to the internal buffer
        rxIndex = 0;
        slaveTxn.txPtr = nullptr;
        slaveTxn.rxPtr = rxLength ? rxBufferPtr : rxBuffer;
        slaveTxn.length = rxLength ? rxLength : WIRE_BUFFER_LENGTH;
        slaveTxn.config = 0;
      }

      sercom->setTxnWIRE(&slaveTxn);

      // SCLSM=0 (Smart Mode disabled): AMATCH and DRDY never fire together
      //   → return now, DRDY will fire in next interrupt
      // SCLSM=1 (Smart Mode enabled) + Master Read: AMATCH+DRDY fire together  
      //   → fall through to handle data immediately
      // SCLSM=1 + Master Write: DRDY not set yet
      //   → return now, DRDY fires later
      if (!drdy)
          return;
      // else: DRDY is set (SCLSM=1 Master Read case), fall through
    }

    // Data Ready - handle byte transfer
    if(drdy)
    {
      isMasterRead ? sercom->sendDataWIRE() : sercom->readDataWIRE();

      if (!isMasterRead)
        rxLength = sercom->getTxnIndexWIRE();      
    }
  }
}

#endif
