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

class TwoWire : public Stream
{
  public:
    TwoWire(SERCOM *s, uint8_t pinSDA, uint8_t pinSCL);
    void begin();
    void begin(uint8_t, bool enableGeneralCall = false);
    void end();
    void setClock(uint32_t);

    void beginTransmission(uint8_t);
    uint8_t endTransmission(bool stopBit);
    uint8_t endTransmission(void);

    uint8_t requestFrom(uint8_t address, size_t quantity, bool stopBit);
    uint8_t requestFrom(uint8_t address, size_t quantity, uint8_t* rxBuffer, bool stopBit = true);
    uint8_t requestFrom(uint8_t address, size_t quantity);

    size_t write(uint8_t data);
    size_t write(const uint8_t * data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void(*)(int));
    void onRequest(void(*)(void));
    void setRxBuffer(uint8_t* buffer, size_t length);
    void clearRxBuffer(void);
    void resetRxBuffer(void);
    uint8_t* getRxBuffer(void);
    size_t getRxLength(void) const;

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;

    void onService(void);

  private:
    SERCOM * sercom;
    uint8_t _uc_pinSDA;
    uint8_t _uc_pinSCL;

    bool transmissionBegun;

    // RX/TX buffers (sync compatibility, async staging)
    static constexpr size_t WIRE_TX_BUFFER_LENGTH = 255;
    static constexpr size_t WIRE_RX_BUFFER_LENGTH = SERIAL_BUFFER_SIZE;
    uint8_t rxBuffer[WIRE_RX_BUFFER_LENGTH];
    uint8_t txBuffer[WIRE_TX_BUFFER_LENGTH];
    uint8_t* rxBufferPtr;
    size_t rxBufferCapacity;
    size_t rxLength;
    size_t rxIndex;
    size_t txLength;
    size_t txIndex;
    size_t masterIndex;
    bool awaitingAddressAck;
    uint8_t txAddress;
    volatile bool txnDone;
    volatile int txnStatus;
    bool pendingReceive;
    int pendingReceiveLength;
    SercomTxn txn;

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

#endif
