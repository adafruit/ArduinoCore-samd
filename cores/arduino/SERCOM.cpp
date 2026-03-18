/*
  Copyright (c) 2014 Arduino.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#include "SERCOM.h"
#include "variant.h"
#include "Arduino.h"

#ifdef USE_ZERODMA
#include <Adafruit_ZeroDMA.h>
#endif

#ifndef WIRE_RISE_TIME_NANOSECONDS
// Default rise time in nanoseconds, based on 4.7K ohm pull up resistors
// you can override this value in your variant if needed
#define WIRE_RISE_TIME_NANOSECONDS 125
#endif

SERCOM::SERCOM(Sercom* s)
{
  sercom = s;
  int8_t idx = getSercomIndex();
  if (idx >= 0 && idx < (int8_t)kSercomCount)
    s_instances[idx] = this;

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
  // A briefly-available but now deprecated feature had the SPI clock source
  // set via a compile-time setting (MAX_SPI)...problem was this affected
  // ALL SERCOMs, whereas some (anything read/write, e.g. SD cards) should
  // not exceed the standard 24 MHz setting.  Newer code, if it needs faster
  // write-only SPI (e.g. to screen), should override the SERCOM clock on a
  // per-peripheral basis.  Nonetheless, we check SERCOM_SPI_FREQ_REF here
  // (MAX_SPI * 2) to retain compatibility with any interim projects that
  // might have relied on the compile-time setting.  But please, don't.
#if SERCOM_SPI_FREQ_REF == F_CPU // F_CPU clock = GCLK0
  clockSource = SERCOM_CLOCK_SOURCE_100M;
 #elif SERCOM_SPI_FREQ_REF == 48000000  // 48 MHz clock = GCLK1 (standard)
  clockSource = SERCOM_CLOCK_SOURCE_48M;
 #elif SERCOM_SPI_FREQ_REF == 100000000 // 100 MHz clock = GCLK2
  clockSource = SERCOM_CLOCK_SOURCE_100M;
 #endif
#endif // end __SAMD51__
}

void SERCOM::resetSERCOM()
{
  // UART, SPI, I2CS, and I2CM use the same SWRST and DBGCTRL bits, so this works for all modes
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
    ; // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0

  // DBGCTRL is not affected by SWRST, so explicitly clear it here to ensure debug behavior is
  // consistent after reset
  sercom->USART.DBGCTRL.bit.DBGSTOP = 0; 
}

/* =========================
 * ===== Sercom UART
 * =========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
  initClockNVIC();
  resetUART();

#ifdef USE_ZERODMA
  int8_t id = getSercomIndex();
  if (id >= 0) {
    dmaSetCallbacks(SERCOM::dmaTxCallbackUART, SERCOM::dmaRxCallbackUART);
    dmaInit(id);
  }
#endif // USE_ZERODMA

  registerService(getSercomIndex(), &SERCOM::stopTransmissionUART);

  //Setting the CTRLA register
  sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |
                            SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
                               SERCOM_USART_INTENSET_ERROR; //All others errors

  if ( mode == UART_INT_CLOCK )
  {
    uint16_t sampleRateValue;

    if (sampleRate == SAMPLE_RATE_x16) {
      sampleRateValue = 16;
    } else {
      sampleRateValue = 8;
    }

    // Asynchronous fractional mode (Table 24-2 in datasheet)
    //   BAUD = fref / (sampleRateValue * fbaud)
    // (multiply by 8, to calculate fractional piece)
    uint32_t baudTimes8 = (freqRef * 8) / (sampleRateValue * baudrate);

    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
  }
}

void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |=
    SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
    dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(charSize) |
    nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
    (parityMode == SERCOM_NO_PARITY ? 0 : parityMode) <<
      SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |
                             SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
}

void SERCOM::resetUART()
{
  resetSERCOM();
}

void SERCOM::flushUART()
{
  // Skip checking transmission completion if data register is empty
  if(isDataRegisterEmptyUART())
    return;

  // Wait for transmission to complete
  while(!sercom->USART.INTFLAG.bit.TXC);
}

void SERCOM::clearStatusUART()
{
  //Reset (with 0) the STATUS register
  sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool SERCOM::availableDataUART()
{
  //RXC : Receive Complete
  return sercom->USART.INTFLAG.bit.RXC;
}

bool SERCOM::isUARTError()
{
  return sercom->USART.INTFLAG.bit.ERROR;
}

void SERCOM::acknowledgeUARTError()
{
  sercom->USART.INTFLAG.bit.ERROR = 1;
}

bool SERCOM::isBufferOverflowErrorUART()
{
  //BUFOVF : Buffer Overflow
  return sercom->USART.STATUS.bit.BUFOVF;
}

bool SERCOM::isFrameErrorUART()
{
  //FERR : Frame Error
  return sercom->USART.STATUS.bit.FERR;
}

void SERCOM::clearFrameErrorUART()
{
  // clear FERR bit writing 1 status bit
  sercom->USART.STATUS.bit.FERR = 1;
}

bool SERCOM::isParityErrorUART()
{
  //PERR : Parity Error
  return sercom->USART.STATUS.bit.PERR;
}

bool SERCOM::isDataRegisterEmptyUART()
{
  //DRE : Data Register Empty
  return sercom->USART.INTFLAG.bit.DRE;
}

uint8_t SERCOM::readDataUART()
{
  return sercom->USART.DATA.bit.DATA;
}

int SERCOM::writeDataUART(uint8_t data)
{
  // Wait for data register to be empty
  while(!isDataRegisterEmptyUART());

  //Put data into DATA register
  sercom->USART.DATA.reg = (uint16_t)data;
  return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

bool SERCOM::startTransmissionUART(void)
{
  SercomTxn* txn = nullptr;
  if (!_txnQueue.peek(txn) || txn == nullptr)
    return false;

  _uart.currentTxn = txn;
  _uart.index = 0;
  _uart.length = txn->length;
  _uart.active = true;

#ifdef USE_ZERODMA
  _uart.useDma = _dmaConfigured;
#else
  _uart.useDma = false;
#endif

  if (!_uart.useDma)
    return false;

#ifdef USE_ZERODMA
  void* dataReg = (void*)&sercom->USART.DATA.reg;
  _uart.dmaNeedTx = (txn->txPtr != nullptr);
  _uart.dmaNeedRx = (txn->rxPtr != nullptr);
  _uart.dmaTxDone = !_uart.dmaNeedTx;
  _uart.dmaRxDone = !_uart.dmaNeedRx;

  DmaStatus st = DmaStatus::Ok;
  if (_uart.dmaNeedTx)
    st = dmaStartTx(txn->txPtr, dataReg, txn->length);
  else if (_uart.dmaNeedRx)
    st = dmaStartRx(txn->rxPtr, dataReg, txn->length);

  if (st != DmaStatus::Ok) {
    _uart.returnValue = SercomUartError::UNKNOWN_ERROR;
    deferStopUART(_uart.returnValue);
    return false;
  }
  return true;
#else
  return false;
#endif
}

bool SERCOM::enqueueUART(SercomTxn* txn)
{
  if (txn == nullptr)
    return false;
#ifdef USE_ZERODMA
  if (!_dmaConfigured)
    return false;
#endif
  if (_txnQueue.isFull())
    return false;  // Queue full; caller must retry at runtime
  if (!_txnQueue.store(txn))
    return false;
  if (!_uart.active) {
    if (!startTransmissionUART()) {
      SercomTxn* tmp = nullptr;
      _txnQueue.read(tmp);
      if (tmp && tmp->onComplete)
        tmp->onComplete(tmp->user, static_cast<int>(SercomUartError::UNKNOWN_ERROR));
      return false;
    }
  }
  return true;
}

void SERCOM::deferStopUART(SercomUartError error)
{
  _uart.returnValue = error;
  setPending((uint8_t)getSercomIndex());
}

SercomTxn* SERCOM::stopTransmissionUART(void)
{
  return stopTransmissionUART(_uart.returnValue);
}

SercomTxn* SERCOM::stopTransmissionUART(SercomUartError error)
{
  SercomTxn* txn = nullptr;
  if (!_txnQueue.peek(txn) || txn == nullptr)
    return nullptr;

  // Call completion callback before deciding to dequeue
  if (txn->onComplete)
    txn->onComplete(txn->user, static_cast<int>(error));

  // Check if callback wants to chain another phase
  if (txn->chainNext) {
    txn->chainNext = false; // reset for next iteration
    if (!startTransmissionUART()) {
      // Hardware start failed, force dequeue
      _txnQueue.read(txn);
      _uart.active = false;
      _uart.currentTxn = nullptr;
      return txn;
    }
    return txn;
  }

  // Normal completion: dequeue and start next transaction
  _txnQueue.read(txn);
  _uart.active = false;
  _uart.currentTxn = nullptr;

  SercomTxn* next = nullptr;
  if (_txnQueue.peek(next) && next)
    startTransmissionUART();

  return txn;
}

/* =========================
 * ===== Sercom SPI
 * =========================
*/
void SERCOM::initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
  initClockNVIC();
  resetSPI();

#ifdef USE_ZERODMA
  int8_t id = getSercomIndex();
  if (id >= 0) {
    dmaSetCallbacks(SERCOM::dmaTxCallbackSPI, SERCOM::dmaRxCallbackSPI);
    dmaInit(id);
  }
#endif // USE_ZERODMA

  registerService(getSercomIndex(), &SERCOM::stopTransmissionSPI);

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3) | // master mode
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;
#else
  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;
#endif

  //Setting the CTRLB register
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

  while( sercom->SPI.SYNCBUSY.bit.CTRLB == 1 );
}

bool SERCOM::startTransmissionSPI(void)
{
  SercomTxn* txn = nullptr;
  if (!_txnQueue.peek(txn) || txn == nullptr)
    return false;

  _spi.currentTxn = txn;
  _spi.index = 0;
  _spi.length = txn->length;
  _spi.active = true;

#ifdef USE_ZERODMA
  _spi.useDma = _dmaConfigured;
#else
  _spi.useDma = false;
#endif

  if (_spi.useDma) {
#ifdef USE_ZERODMA
    void* dataReg = (void*)&sercom->SPI.DATA.reg;
    _spi.dmaNeedTx = (txn->txPtr != nullptr);
    _spi.dmaNeedRx = (txn->rxPtr != nullptr);
    _spi.dmaTxDone = !_spi.dmaNeedTx;
    _spi.dmaRxDone = !_spi.dmaNeedRx;

    DmaStatus st = DmaStatus::Ok;
    if (_spi.dmaNeedTx && _spi.dmaNeedRx)
      st = dmaStartDuplex(txn->txPtr, txn->rxPtr, dataReg, dataReg, txn->length, nullptr);
    else if (_spi.dmaNeedTx)
      st = dmaStartTx(txn->txPtr, dataReg, txn->length);
    else
      st = dmaStartDuplex(nullptr, txn->rxPtr, dataReg, dataReg, txn->length, nullptr);

    if (st != DmaStatus::Ok) {
      _spi.returnValue = SercomSpiError::UNKNOWN_ERROR;
      deferStopSPI(_spi.returnValue);
      return false;
    }
    return true;
#endif
  }

  sercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_DRE |
                             SERCOM_SPI_INTENSET_RXC |
                             SERCOM_SPI_INTENSET_ERROR;
  return true;
}

bool SERCOM::enqueueSPI(SercomTxn* txn)
{
  if (txn == nullptr)
    return false;
  if (_txnQueue.isFull())
    return false;  // Queue full; caller must retry at runtime
  if (!_txnQueue.store(txn))
    return false;
  if (!_spi.active) {
    startTransmissionSPI();
  }
  return true;
}

void SERCOM::deferStopSPI(SercomSpiError error)
{
  _spi.returnValue = error;
  setPending((uint8_t)getSercomIndex());
}

SercomTxn* SERCOM::stopTransmissionSPI(void)
{
  return stopTransmissionSPI(_spi.returnValue);
}

SercomTxn* SERCOM::stopTransmissionSPI(SercomSpiError error)
{
  SercomTxn* txn = nullptr;
  if (!_txnQueue.peek(txn) || txn == nullptr)
    return nullptr;

  // Call completion callback before deciding to dequeue
  if (txn->onComplete)
    txn->onComplete(txn->user, static_cast<int>(error));

  // Check if callback wants to chain another phase
  if (txn->chainNext) {
    txn->chainNext = false; // reset for next iteration
    startTransmissionSPI(); // restart with updated context, same queue slot
    return txn;
  }

  // Normal completion: dequeue and start next transaction
  _txnQueue.read(txn);
  _spi.active = false;
  _spi.currentTxn = nullptr;

  SercomTxn* next = nullptr;
  if (_txnQueue.peek(next) && next)
    startTransmissionSPI();

  return txn;
}

void SERCOM::initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate)
{
  //Extract data from clockMode
  int cpha, cpol;

  if((clockMode & (0x1ul)) == 0 )
    cpha = 0;
  else
    cpha = 1;

  if((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg |= ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                           ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
}

void SERCOM::resetSPI()
{
  resetSERCOM();
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
  //Register enable-protected
  disableSPI();

  sercom->SPI.CTRLA.bit.DORD = dataOrder;

  enableSPI();
}

SercomDataOrder SERCOM::getDataOrderSPI()
{
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
  disableSPI(); // Register is enable-protected
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(freqRef / divider);
  enableSPI();
}

void SERCOM::setClockModeSPI(SercomSpiClockMode clockMode)
{
  int cpha, cpol;
  if((clockMode & (0x1ul)) == 0)
    cpha = 0;
  else
    cpha = 1;

  if((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  //Register enable-protected
  disableSPI();

  sercom->SPI.CTRLA.bit.CPOL = cpol;
  sercom->SPI.CTRLA.bit.CPHA = cpha;

  enableSPI();
}

uint8_t SERCOM::transferDataSPI(uint8_t data)
{
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while(sercom->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception

  return sercom->SPI.DATA.bit.DATA;  // Reading data
}

bool SERCOM::isBufferOverflowErrorSPI() { return sercom->SPI.STATUS.bit.BUFOVF; }
bool SERCOM::isDataRegisterEmptySPI() { return sercom->SPI.INTFLAG.bit.DRE; }

//bool SERCOM::isTransmitCompleteSPI()
//{
//  //TXC : Transmit complete
//  return sercom->SPI.INTFLAG.bit.TXC;
//}
//
//bool SERCOM::isReceiveCompleteSPI()
//{
//  //RXC : Receive complete
//  return sercom->SPI.INTFLAG.bit.RXC;
//}

uint8_t SERCOM::calculateBaudrateSynchronous(uint32_t baudrate)
{
  uint16_t b = freqRef / (2 * baudrate);
  if (b > 0)
    b--; // Don't -1 on baud calc if already at 0
  return b;
}


/* =========================
 * ===== Sercom WIRE
 * =========================
 */
void SERCOM::resetWIRE()
{
  clearQueueWIRE();  // Drain pending transactions from queue
  resetSERCOM();     // SWRST: hardware reset to default state
  _wire = WireConfig{};  // Reset software state
}

void SERCOM::clearQueueWIRE(void)
{
  // Drain all pending transactions from the queue without invoking callbacks
  // This is needed for test teardown to ensure no stale transactions carry over
  SercomTxn* txn = nullptr;
  int drained = 0;
  while (_txnQueue.read(txn)) {
    drained++;
    // Just discard - don't invoke callbacks during reset
  }
  
  // Ensure wire state is completely clean
  _wire.active = false;
  _wire.currentTxn = nullptr;
  _wire.txnIndex = 0;
  _wire.txnLength = 0;
  _wire.returnValue = SercomWireError::SUCCESS;
  _wire.retryCount = 0;
  
  // Clear deferred callbacks (from slave/receive operations)
  _wireDeferredCb = nullptr;
  _wireDeferredUser = nullptr;
  _wireDeferredLength = 0;
  _wireDeferredPending = false;
}

void SERCOM::initWIRE(void)
{
  if (_wire.inited)  // If already initialized, return
    return;

  uint8_t idx = getSercomIndex();
  initClockNVIC();
  registerService(idx, static_cast<ServiceFn>(&SERCOM::stopTransmissionWIRE));
  
#ifdef USE_ZERODMA
  dmaSetCallbacks(SERCOM::dmaTxCallbackWIRE, SERCOM::dmaRxCallbackWIRE);
  if (idx >= 0)
    dmaInit(idx);
#endif // USE_ZERODMA

  _wire.inited = true;  // Mark as initialized last
}

void SERCOM::initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall, uint8_t speed )
{
  initSlaveWIRE( ucAddress & 0x7Fu, enableGeneralCall, speed, false );
}

void SERCOM::initSlaveWIRE( uint16_t ucAddress, bool enableGeneralCall, uint8_t speed, bool enable10Bit )
{
  initWIRE();

  uint16_t mask = enable10Bit ? 0x03FFul : 0x007Ful;
  _wire.slaveSpeed = speed;
  _wire.addr = SERCOM_I2CS_ADDR_ADDR(ucAddress & mask) |       // select either 7 or 10-bits
               SERCOM_I2CS_ADDR_ADDRMASK(0x00ul) |             // 0x00, only match exact address
               (enable10Bit ? SERCOM_I2CS_ADDR_TENBITEN : 0) | // 10-bit addressing
               enableGeneralCall;                              // enable general call (address 0x00)
  setSlaveWIRE();
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
  initWIRE();

  setBaudrateWIRE(baudrate);
  setMasterWIRE();
}

void SERCOM::registerReceiveWIRE(void (*cb)(void* user, int length), void* user)
{
  _wireDeferredCb = cb;
  _wireDeferredUser = user;
}

void SERCOM::deferReceiveWIRE(int length)
{
  _wireDeferredLength = length;
  _wireDeferredPending = true;
  setPending((uint8_t)getSercomIndex());
}

void SERCOM::setMasterWIRE(void)
{
  // Errata: do not enable QCEN when SCLSM=1 (bus error). Hs-mode requires SCLSM=1,
  // so master Hs-mode must be DMA-only and STOP-only (no repeated starts).
  disableWIRE();
  bool sclsm = (_wire.masterSpeed == 0x2);
  sercom->I2CM.CTRLA.reg = _wire.ctrla                                  |
                           SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION) |
                           SERCOM_I2CM_CTRLA_SPEED(_wire.masterSpeed)   |
                           (sclsm ? SERCOM_I2CM_CTRLA_SCLSM : 0 );
  sercom->I2CM.CTRLB.reg = _wire.ctrlb;
  sercom->I2CM.BAUD.reg = _wire.baud;
  enableWIRE();
  // Disable slave interrupts.
  // Master interrupts are set in startTransmissionWIRE() when the transaction is enqueued,
  // so we don't want to enable them here.
  sercom->I2CS.INTENCLR.reg = SERCOM_I2CS_INTENSET_ERROR  |
                              SERCOM_I2CS_INTENSET_AMATCH |
                              SERCOM_I2CS_INTENSET_DRDY   |
                              SERCOM_I2CS_INTENSET_PREC;
}

void SERCOM::setSlaveWIRE(void)
{
  disableWIRE();
  bool sclsm = (_wire.slaveSpeed == 0x2);
  sercom->I2CS.CTRLA.reg = SERCOM_I2CS_CTRLA_MODE(I2C_SLAVE_OPERATION) |
                           SERCOM_I2CS_CTRLA_SPEED(_wire.slaveSpeed)   |
                           (sclsm ? SERCOM_I2CS_CTRLA_SCLSM : 0 );
  sercom->I2CS.CTRLB.reg = _wire.ctrlb | SERCOM_I2CS_CTRLB_AACKEN;
  sercom->I2CS.ADDR.reg = _wire.addr;
  enableWIRE();
  // Enable slave interrupts: address match, data ready, stop/restart.
  sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_ERROR  | // Error
                              SERCOM_I2CS_INTENSET_PREC   | // Stop
                              SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                              SERCOM_I2CS_INTENSET_DRDY;    // Data Ready
}

void SERCOM::setBaudrateWIRE(uint32_t baudrate)
{
// Determine speed mode based on requested baudrate
  const uint32_t topSpeeds[3] = {400000, 1000000, 3400000}; // {(sm/fm), (fm+), (hs)}
  uint8_t speedBit;

  if (baudrate <= topSpeeds[0])
    speedBit = 0; // Standard/Fast mode up to 400 khz
  else if (baudrate <= topSpeeds[1])
    speedBit = 1; // Fast mode+ up to 1 Mhz
  else 
    speedBit = 2; // High speed up to 3.4 Mhz

  _wire.masterSpeed = speedBit;

  uint32_t fREF = getSercomFreqRef();
  uint32_t minBaudrate = fREF / 512; // BAUD = 255: SAMD51(@100MHz) ~195kHz, SAMD21 ~94kHz
  uint32_t maxBaudrate = topSpeeds[speedBit];
  baudrate = max(minBaudrate, min(baudrate, maxBaudrate));

  if (speedBit == 0x2)
    _wire.baud = SERCOM_I2CM_BAUD_HSBAUD(fREF / (2 * baudrate) - 1);
  else
    _wire.baud = SERCOM_I2CM_BAUD_BAUD(fREF / (2 * baudrate) - 5 -
                 (fREF/1000000ul * WIRE_RISE_TIME_NANOSECONDS) / 2000);

  if (isMasterWIRE())
    setMasterWIRE();
}


SercomTxn* SERCOM::startTransmissionWIRE( void )
{
  // Writing ADDR.ADDR drives different behavior based on BUSSTATE:
  // UNKNOWN: MB and BUSERR assert and the transfer aborts.
  // BUSY: The host waits until the bus is IDLE.
  // IDLE: A START is generated, the address is sent, and on ACK the host holds SCL low with CLKHOLD
  // set and MB asserted.
  // OWNER: A repeated START is generated; if the prior transaction was a read, the ACK/NACK for the
  // read is sent before the repeated START. The repeated START ADDR write must occur while MB or SB
  // is set.
  // Writing ADDR also clears BUSERR, ARBLOST, MB, and SB.

  if (isBusUnknownWIRE()) {
    stopTransmissionWIRE(SercomWireError::BUS_STATE_UNKNOWN);
    return nullptr;
  }

  SercomTxn* txn = nullptr;

  if (!_txnQueue.peek(txn))
    return nullptr;

  if (txn != _wire.currentTxn)
    _wire.retryCount = 0;

  _wire.currentTxn = txn;
  _wire.txnIndex = 0;
  _wire.txnLength = txn->length;
  setDmaWIRE(false);  // Reset DMA mode - let code below decide if DMA is used
  
  const bool read = txn->config & I2C_CFG_READ;
  uint16_t addr = (txn->config & I2C_CFG_10BIT) ? I2C_ADDR(txn->address) : I2C_ADDR7(txn->address);
  addr = (uint16_t)((addr << 1) | (read ? 1u : 0u));
  bool hsMode = (_wire.masterSpeed == 0x2);
  uint32_t addrReg = SERCOM_I2CM_ADDR_ADDR(addr) |
                     ((txn->config & I2C_CFG_10BIT) ? SERCOM_I2CM_ADDR_TENBITEN : 0) |
                     (hsMode ? SERCOM_I2CM_ADDR_HS : 0);

  if (hsMode || sercom->I2CM.CTRLA.bit.SCLSM) {
#ifndef USE_ZERODMA
    stopTransmissionWIRE(SercomWireError::OTHER);
    return nullptr;
#endif
    if (txn->length >255) {
      stopTransmissionWIRE(SercomWireError::DATA_TOO_LONG);
      return nullptr;
    }

    if (sercom->I2CM.CTRLB.bit.QCEN) {
      stopTransmissionWIRE(SercomWireError::OTHER);
      return nullptr;
    }

    txn->config |= I2C_CFG_STOP;
    setDmaWIRE(true);
  }
#ifdef USE_ZERODMA
  else {
    setDmaWIRE(txn->length > 0 && txn->length < 256 &&
              (txn->config & I2C_CFG_STOP) &&
              !(txn->config & I2C_CFG_NODMA));
  }

  if (isDmaWIRE())
  {
    if (!_dmaConfigured)
      dmaInit(getSercomIndex());

    if (!_dmaConfigured || !_dmaTx || !_dmaRx) {
      stopTransmissionWIRE(SercomWireError::OTHER);
      return nullptr;
    }
    
    addrReg |= SERCOM_I2CM_ADDR_LENEN | SERCOM_I2CM_ADDR_LEN((uint8_t)txn->length);
  }
#endif

  // Send address (non-blocking; ISR handles ERROR/MB/SB)
  _wire.active = true;
  sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_ERROR | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_MB;
  sercom->I2CM.ADDR.reg = addrReg; // ADDR is write synchronized so just wait for the MB/SB to know when synced

  return txn;
}

bool SERCOM::enqueueWIRE(SercomTxn* txn)
{
  if (txn == nullptr)
    return false;
  if (_txnQueue.isFull())
    return false;  // Queue full; caller must retry at runtime
  if (!_txnQueue.store(txn))
    return false;
  if (!_wire.active)
    return startTransmissionWIRE() != nullptr;
  return true;
}

SercomTxn* SERCOM::stopTransmissionWIRE( void )
{
  return stopTransmissionWIRE( _wire.returnValue );
}

SercomTxn* SERCOM::stopTransmissionWIRE( SercomWireError error )
{
  // Policy: only auto-retry recoverable bus-state errors here. All other
  // errors are surfaced to the transaction callback for protocol handling.
  // Retry/backoff policy is intentionally deferred; a future change may add
  // a retry budget or tick-based delay if needed.

  SercomTxn* txn = _wire.currentTxn;
  SercomTxn* next = nullptr;

  constexpr uint8_t kMaxWireRetries = 3;

  if (error == SercomWireError::BUS_STATE_UNKNOWN) {
    if (_wire.retryCount < kMaxWireRetries) {
      ++_wire.retryCount;

      sercom->I2CM.STATUS.bit.BUSSTATE = 1;
      while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ;
      startTransmissionWIRE();

      return txn;
    }
  }

  if (error == SercomWireError::ARBITRATION_LOST || error == SercomWireError::BUS_ERROR) {
    if (_wire.retryCount < kMaxWireRetries) {
      ++_wire.retryCount;

      sercom->I2CM.STATUS.bit.ARBLOST = 1; // Clear arbitration lost flag
      sercom->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR;
      startTransmissionWIRE();

      return txn;
    }
  }

  if (error == SercomWireError::BUS_STATE_UNKNOWN ||
      error == SercomWireError::ARBITRATION_LOST  ||
      error == SercomWireError::BUS_ERROR) {
    _wire.retryCount = 0;
  }

  if(isMasterWIRE())
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ; // Wait for DATA to sync from last transaction

  // Undocumented HW limitation: DMA transfers must terminate with STOP and bus release.
  // After a DMA write, the host holds the bus ~7.33 us before the next transfer (Sr window).
  // Writing ADDR during that window leaves the bus in an undefined state and breaks
  // subsequent DMA/non-DMA transactions. To avoid this, we must wait for BUSSTATE
  // to return to IDLE after a STOP returning the hardware to a known state.
  // At the tested 48 MHz, this busy-wait is ~350 cycles corresponding to a 3.5 us delay
  // at 100 MHz. This wait must occur BEFORE the callback to ensure the bus is stable
  // before user code can enqueue the next transaction.
  if (isMasterWIRE() && txn && (txn->config & I2C_CFG_STOP)) {
    while (sercom->I2CM.STATUS.bit.BUSSTATE > 0x1) ;
  }

  // Callbacks are expected to run in non-ISR context (main loop/PendSV).
  if (txn && txn->onComplete)
    txn->onComplete(txn->user, static_cast<int>(error));

  // Allow multi-phase I2C transactions to chain without dequeuing.
  if (isMasterWIRE() && txn && txn->chainNext) {
    txn->chainNext = false; // reset for next iteration
    startTransmissionWIRE();
    return txn;
  }

  if(isMasterWIRE())
    _txnQueue.read(txn); // remove the completed transaction from the queue
  else {
    // Deliver deferred WIRE callback outside the SERCOM ISR (from PendSV).
    // This avoids running user code in the hardware interrupt context.
    if (_wireDeferredPending && _wireDeferredCb) {
      _wireDeferredPending = false;
      _wireDeferredCb(_wireDeferredUser, _wireDeferredLength);
    }
  }

  _wire.retryCount = 0;
  _wire.active = false;
  _wire.currentTxn = nullptr;

  bool isMaster = isMasterWIRE();

  if (_txnQueue.peek(next) && isMaster)
    startTransmissionWIRE();
  else if (isMaster)
    sercom->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_ERROR |
                                SERCOM_I2CM_INTENCLR_MB    |
                                SERCOM_I2CM_INTENCLR_SB;

  return txn;
}

// Hardware metadata structure for SERCOM peripherals - private to this file
#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
// SAMD51 has separate core and slow clocks, and extended interrupt array
struct SercomData {
  Sercom   *sercomPtr;
  uint8_t   id_core;
  uint8_t   id_slow;
  IRQn_Type irq[4];
  uint8_t   dmaTxTrigger;
  uint8_t   dmaRxTrigger;
  void     *dataReg;  // Pointer to DATA register
};

static const SercomData sercomData[] = {
  { SERCOM0, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW,
    SERCOM0_0_IRQn, SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_3_IRQn,
    SERCOM0_DMAC_ID_TX, SERCOM0_DMAC_ID_RX, (void*)&SERCOM0->I2CM.DATA.reg },
  { SERCOM1, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW,
    SERCOM1_0_IRQn, SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_3_IRQn,
    SERCOM1_DMAC_ID_TX, SERCOM1_DMAC_ID_RX, (void*)&SERCOM1->I2CM.DATA.reg },
  { SERCOM2, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW,
    SERCOM2_0_IRQn, SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_3_IRQn,
    SERCOM2_DMAC_ID_TX, SERCOM2_DMAC_ID_RX, (void*)&SERCOM2->I2CM.DATA.reg },
  { SERCOM3, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW,
    SERCOM3_0_IRQn, SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_3_IRQn,
    SERCOM3_DMAC_ID_TX, SERCOM3_DMAC_ID_RX, (void*)&SERCOM3->I2CM.DATA.reg },
  { SERCOM4, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW,
    SERCOM4_0_IRQn, SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_3_IRQn,
    SERCOM4_DMAC_ID_TX, SERCOM4_DMAC_ID_RX, (void*)&SERCOM4->I2CM.DATA.reg },
  { SERCOM5, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW,
    SERCOM5_0_IRQn, SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_3_IRQn,
    SERCOM5_DMAC_ID_TX, SERCOM5_DMAC_ID_RX, (void*)&SERCOM5->I2CM.DATA.reg },
#if defined(SERCOM6)
  { SERCOM6, SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW,
    SERCOM6_0_IRQn, SERCOM6_1_IRQn, SERCOM6_2_IRQn, SERCOM6_3_IRQn,
    SERCOM6_DMAC_ID_TX, SERCOM6_DMAC_ID_RX, (void*)&SERCOM6->I2CM.DATA.reg },
#endif
#if defined(SERCOM7)
  { SERCOM7, SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW,
    SERCOM7_0_IRQn, SERCOM7_1_IRQn, SERCOM7_2_IRQn, SERCOM7_3_IRQn,
    SERCOM7_DMAC_ID_TX, SERCOM7_DMAC_ID_RX, (void*)&SERCOM7->I2CM.DATA.reg },
#endif
};

#else // end if SAMD51 (prob SAMD21)
// SAMD21 has unified clock and single interrupt
struct SercomData {
  Sercom   *sercomPtr;
  uint8_t   clock;
  IRQn_Type irqn;
  uint8_t   dmaTxTrigger;
  uint8_t   dmaRxTrigger;
  void     *dataReg;  // Pointer to DATA register
};

static const SercomData sercomData[] = {
  { SERCOM0, GCM_SERCOM0_CORE, SERCOM0_IRQn, SERCOM0_DMAC_ID_TX, SERCOM0_DMAC_ID_RX, (void*)&SERCOM0->I2CM.DATA.reg },
  { SERCOM1, GCM_SERCOM1_CORE, SERCOM1_IRQn, SERCOM1_DMAC_ID_TX, SERCOM1_DMAC_ID_RX, (void*)&SERCOM1->I2CM.DATA.reg },
  { SERCOM2, GCM_SERCOM2_CORE, SERCOM2_IRQn, SERCOM2_DMAC_ID_TX, SERCOM2_DMAC_ID_RX, (void*)&SERCOM2->I2CM.DATA.reg },
  { SERCOM3, GCM_SERCOM3_CORE, SERCOM3_IRQn, SERCOM3_DMAC_ID_TX, SERCOM3_DMAC_ID_RX, (void*)&SERCOM3->I2CM.DATA.reg },
#if defined(SERCOM4)
  { SERCOM4, GCM_SERCOM4_CORE, SERCOM4_IRQn, SERCOM4_DMAC_ID_TX, SERCOM4_DMAC_ID_RX, (void*)&SERCOM4->I2CM.DATA.reg },
#endif
#if defined(SERCOM5)
  { SERCOM5, GCM_SERCOM5_CORE, SERCOM5_IRQn, SERCOM5_DMAC_ID_TX, SERCOM5_DMAC_ID_RX, (void*)&SERCOM5->I2CM.DATA.reg },
#endif
};

#endif // end !SAMD51

std::array<SERCOM::SercomState, SERCOM::kSercomCount> SERCOM::s_states = {};
std::array<SERCOM*, SERCOM::kSercomCount> SERCOM::s_instances = {};
volatile uint32_t SERCOM::s_pendingMask = 0;

bool SERCOM::claim(uint8_t sercomId, Role role)
{
  if (sercomId >= kSercomCount)
    return false;

  SercomState &state = s_states[sercomId];
  if (state.role != Role::None && state.role != role)
    return false;

  state.role = role;
  return true;
}

void SERCOM::release(uint8_t sercomId)
{
  if (sercomId >= kSercomCount)
    return;

  SercomState &state = s_states[sercomId];
  state.role = Role::None;
  state.service = nullptr;
#ifdef SERCOM_STRICT_PADS
  clearPads(sercomId);
#endif // SERCOM_STRICT_PADS
  s_pendingMask &= ~(1u << sercomId);
}

bool SERCOM::registerService(uint8_t sercomId, ServiceFn fn)
{
  if (sercomId >= kSercomCount)
    return false;

  s_states[sercomId].service = fn;
  return true;
}

#ifdef USE_ZERODMA
SERCOM::DmaStatus SERCOM::dmaInit(int8_t sercomId, uint8_t beatSize)
{
  if (_dmaConfigured)
    return DmaStatus::Ok;

  // Validate beat size: 0=byte, 1=halfword, 2=word
  if (beatSize > DMA_BEAT_SIZE_WORD)
    beatSize = DMA_BEAT_SIZE_BYTE;

  // Look up DMA triggers from sercomData table
#ifdef SERCOM0_DMAC_ID_TX
  if (sercomId >= 0 && sercomId < (int8_t)kSercomCount && sercomData[sercomId].dmaTxTrigger != 0)
  {
    _dmaTxTrigger = sercomData[sercomId].dmaTxTrigger;
    _dmaRxTrigger = sercomData[sercomId].dmaRxTrigger;
  }
  else
#endif
  {
    // Fallback: calculate triggers if table lookup unavailable
    _dmaTxTrigger = SERCOM0_DMAC_ID_TX + (sercomId * 2);
    _dmaRxTrigger = SERCOM0_DMAC_ID_RX + (sercomId * 2);
  }

  // DATA register is at the same offset (0x28) for all protocols (I2C, SPI, UART).
  // Access via any union member is transparent—just use I2CM as the canonical reference.
  void* dataReg = (void*)&sercom->I2CM.DATA.reg;

  if (!_dmaTx)
    _dmaTx = new Adafruit_ZeroDMA();
  if (!_dmaRx)
    _dmaRx = new Adafruit_ZeroDMA();
  if (!_dmaTx || !_dmaRx)
  {
    _dmaLastError = DmaStatus::AllocateFailed;
    dmaRelease();
    return _dmaLastError;
  }

  if (_dmaTx->allocate() != DMA_STATUS_OK)
  {
    _dmaLastError = DmaStatus::AllocateFailed;
    dmaRelease();
    return _dmaLastError;
  }
  if (_dmaRx->allocate() != DMA_STATUS_OK)
  {
    _dmaLastError = DmaStatus::AllocateFailed;
    dmaRelease();
    return _dmaLastError;
  }

  _dmaTx->setTrigger(_dmaTxTrigger);
  _dmaTx->setAction(DMA_TRIGGER_ACTON_BEAT);
  _dmaRx->setTrigger(_dmaRxTrigger);
  _dmaRx->setAction(DMA_TRIGGER_ACTON_BEAT);

  if (_dmaTxCb)
    _dmaTx->setCallback(_dmaTxCb);
  if (_dmaRxCb)
    _dmaRx->setCallback(_dmaRxCb);

  if (_dmaTxDesc == nullptr)
    _dmaTxDesc = _dmaTx->addDescriptor(&_dmaDummy, dataReg, 0, (dma_beat_size)beatSize, true, false);
  if (_dmaRxDesc == nullptr)
    _dmaRxDesc = _dmaRx->addDescriptor(dataReg, &_dmaDummy, 0, (dma_beat_size)beatSize, false, true);
  if (_dmaTxDesc == nullptr || _dmaRxDesc == nullptr)
  {
    _dmaLastError = DmaStatus::DescriptorFailed;
    dmaRelease();
    return _dmaLastError;
  }

  _dmaConfigured = true;
  _dmaLastError = DmaStatus::Ok;
  return _dmaLastError;
}

void SERCOM::dmaSetCallbacks(DmaCallback txCb, DmaCallback rxCb)
{
  _dmaTxCb = txCb;
  _dmaRxCb = rxCb;

  if (_dmaConfigured)
  {
    if (_dmaTxCb)
      _dmaTx->setCallback(_dmaTxCb);
    if (_dmaRxCb)
      _dmaRx->setCallback(_dmaRxCb);
  }
}

SERCOM::DmaStatus SERCOM::dmaStartTx(const void* src, volatile void* dstReg, size_t len)
{
  if (!_dmaConfigured || !_dmaTx) {
    _dmaLastError = DmaStatus::NotConfigured;
    return _dmaLastError;
  }
  if (src == nullptr || dstReg == nullptr) {
    _dmaLastError = DmaStatus::NullPtr;
    return _dmaLastError;
  }
  if (len == 0) {
    _dmaLastError = DmaStatus::ZeroLen;
    return _dmaLastError;
  }
  if (_dmaTxDesc == nullptr) {
    _dmaLastError = DmaStatus::DescriptorFailed;
    return _dmaLastError;
  }

  _dmaTx->changeDescriptor(_dmaTxDesc, const_cast<void*>(src),
                           const_cast<void*>(dstReg), len);

  if (_dmaTx->startJob() != DMA_STATUS_OK) {
    _dmaTx->abort();
    _dmaLastError = DmaStatus::StartFailed;
    return _dmaLastError;
  }

  _dmaTxActive = true;
  _dmaLastError = DmaStatus::Ok;
  return _dmaLastError;
}

SERCOM::DmaStatus SERCOM::dmaStartRx(void* dst, volatile void* srcReg, size_t len)
{
  if (!_dmaConfigured || !_dmaRx) {
    _dmaLastError = DmaStatus::NotConfigured;
    return _dmaLastError;
  }
  if (dst == nullptr || srcReg == nullptr) {
    _dmaLastError = DmaStatus::NullPtr;
    return _dmaLastError;
  }
  if (len == 0) {
    _dmaLastError = DmaStatus::ZeroLen;
    return _dmaLastError;
  }
  if (_dmaRxDesc == nullptr) {
    _dmaLastError = DmaStatus::DescriptorFailed;
    return _dmaLastError;
  }

  _dmaRx->changeDescriptor(_dmaRxDesc,
                           const_cast<void*>(srcReg),
                           dst, len);

  if (_dmaRx->startJob() != DMA_STATUS_OK) {
    _dmaRx->abort();
    _dmaLastError = DmaStatus::StartFailed;
    return _dmaLastError;
  }

  _dmaRxActive = true;
  _dmaLastError = DmaStatus::Ok;
  return _dmaLastError;
}

SERCOM::DmaStatus SERCOM::dmaStartDuplex(const void* txSrc, void* rxDst, volatile void* txReg, volatile void* rxReg, size_t len,
                                         const uint8_t* dummyTx)
{
  if (len == 0)
  {
    _dmaLastError = DmaStatus::ZeroLen;
    return _dmaLastError;
  }
  DmaStatus st = dmaStartRx(rxDst, rxReg, len);
  if (st != DmaStatus::Ok)
    return st;
  static const uint8_t kDummyByte = 0xFF;
  const void* txPtr = txSrc ? txSrc : (dummyTx ? dummyTx : &kDummyByte);
  st = dmaStartTx(txPtr, txReg, len);
  if (st != DmaStatus::Ok)
  {
    dmaAbortRx();
    return st;
  }
  return DmaStatus::Ok;
}

void SERCOM::dmaAbortTx()
{
  if (_dmaTx)
    _dmaTx->abort();
  _dmaTxActive = false;
}

void SERCOM::dmaAbortRx()
{
  if (_dmaRx)
    _dmaRx->abort();
  _dmaRxActive = false;
}

void SERCOM::dmaRelease()
{
  if (!_dmaConfigured)
  {
    dmaResetDescriptors();
    if (_dmaTx)
    {
      delete _dmaTx;
      _dmaTx = nullptr;
    }
    if (_dmaRx)
    {
      delete _dmaRx;
      _dmaRx = nullptr;
    }
    _dmaLastError = DmaStatus::Ok;
    return;
  }

  dmaAbortTx();
  dmaAbortRx();

  if (_dmaTx)
    _dmaTx->free();
  if (_dmaRx)
    _dmaRx->free();

  dmaResetDescriptors();

  _dmaConfigured = false;
  if (_dmaTx)
  {
    delete _dmaTx;
    _dmaTx = nullptr;
  }
  if (_dmaRx)
  {
    delete _dmaRx;
    _dmaRx = nullptr;
  }
  _dmaLastError = DmaStatus::Ok;
}

void SERCOM::dmaResetDescriptors()
{
  _dmaTxDesc = nullptr;
  _dmaRxDesc = nullptr;
}

bool SERCOM::dmaTxBusy() const
{
  return _dmaTxActive;
}

bool SERCOM::dmaRxBusy() const
{
  return _dmaRxActive;
}

SERCOM::DmaStatus SERCOM::dmaLastError() const
{
  return _dmaLastError;
}
#endif

#ifdef SERCOM_STRICT_PADS
bool SERCOM::registerPads(uint8_t sercomId, const PadFunc (&pads)[4], bool muxFunctionD)
{
  if (sercomId >= kSercomCount)
    return false;

  SercomState &state = s_states[sercomId];
  for (size_t i = 0; i < 4; ++i)
  {
    PadFunc desired = pads[i];
    if (desired == PadFunc::None)
      continue;
    PadFunc existing = state.pads[i];
    if (existing != PadFunc::None && existing != desired)
      return false;
  }
  if (state.padsConfigured && state.muxFunctionD != muxFunctionD)
    return false;

  bool any = false;
  for (size_t i = 0; i < 4; ++i)
  {
    PadFunc desired = pads[i];
    if (desired == PadFunc::None)
      continue;
    state.pads[i] = desired;
    any = true;
  }
  if (any)
  {
    state.padsConfigured = true;
    state.muxFunctionD = muxFunctionD;
  }
  return true;
}

void SERCOM::clearPads(uint8_t sercomId)
{
  if (sercomId >= kSercomCount)
    return;

  SercomState &state = s_states[sercomId];
  for (size_t i = 0; i < 4; ++i)
    state.pads[i] = PadFunc::None;
  state.padsConfigured = false;
  state.muxFunctionD = false;
}
#endif // SERCOM_STRICT_PADS

void SERCOM::setPending(uint8_t sercomId)
{
  if (sercomId >= kSercomCount)
    return;

  __disable_irq();
  s_pendingMask |= (1u << sercomId);
  __enable_irq();
  __DMB();
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void SERCOM::dispatchPending(void)
{
  uint32_t pending;

  __disable_irq();
  pending = s_pendingMask;
  s_pendingMask = 0;
  __enable_irq();

  for (size_t i = 0; i < kSercomCount; ++i)
  {
    if ((pending & (1u << i)) == 0)
      continue;

    ServiceFn fn = s_states[i].service;
    SERCOM* inst = s_instances[i];
    if (fn && inst)
      (inst->*fn)();
  }
}

extern "C" void PendSV_Handler(void)
{
  SERCOM::dispatchPending();
}

int8_t SERCOM::getSercomIndex(void) {
  for(uint8_t i=0; i<(sizeof(sercomData) / sizeof(sercomData[0])); i++) {
    if(sercom == sercomData[i].sercomPtr) return i;
  }
  return -1;
}

uint32_t SERCOM::getSercomFreqRef(void)
{
#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
  int8_t idx = getSercomIndex();
  uint8_t gen = 1; // default to GCLK1 (48 MHz) if we can't resolve

  if (idx >= 0)
  {
    uint8_t pch = sercomData[idx].id_core;
    gen = GCLK->PCHCTRL[pch].bit.GEN;
  }

  switch (gen)
  {
  case 0:
    freqRef = 100000000UL;
    break;
  case 1:
    freqRef = 48000000UL;
    break;
  case 2:
    freqRef = 100000000UL;
    break;
  case 3:
    freqRef = 32768UL;
    break;
  case 4:
    freqRef = 12000000UL;
    break;
  default:
    freqRef = 48000000UL;
    break;
  }
#else
  freqRef = SystemCoreClock;
#endif

  return freqRef;
}

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
// This is currently for overriding an SPI SERCOM's clock source only --
// NOT for UART or WIRE SERCOMs, where it will have unintended consequences.
// It does not check.
// SERCOM clock source override is available only on SAMD51 (not 21).
// A dummy function for SAMD21 (compiles to nothing) is present in SERCOM.h
// so user code doesn't require a lot of conditional situations.
void SERCOM::setClockSource(int8_t idx, SercomClockSource src, bool core) {

  if(src == SERCOM_CLOCK_SOURCE_NO_CHANGE) return;

  uint8_t clk_id = core ? sercomData[idx].id_core : sercomData[idx].id_slow;

  GCLK->PCHCTRL[clk_id].bit.CHEN = 0;     // Disable timer
  while(GCLK->PCHCTRL[clk_id].bit.CHEN);  // Wait for disable

  if(core) clockSource = src; // Save SercomClockSource value

  // From cores/arduino/startup.c:
  // GCLK0 = F_CPU (this is 120 MHz and exceeds SERCOM maximum)
  // GCLK1 = 48 MHz
  // GCLK2 = 100 MHz
  // GCLK3 = XOSC32K
  // GCLK4 = 12 MHz
  if(src == SERCOM_CLOCK_SOURCE_FCPU) {
    GCLK->PCHCTRL[clk_id].reg =
        GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); // Guard Sercom from exceeding 100 MHz maximum
    if (core)
      freqRef = 100000000; // Save clock frequency value
  }
  else if (src == SERCOM_CLOCK_SOURCE_48M)
  {
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    if(core) freqRef = 48000000;
  } else if(src == SERCOM_CLOCK_SOURCE_100M) {
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    if(core) freqRef = 100000000;
  } else if(src == SERCOM_CLOCK_SOURCE_32K) {
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    if(core) freqRef = 32768;
  } else if(src == SERCOM_CLOCK_SOURCE_12M) {
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    if(core) freqRef = 12000000;
  }

  while(!GCLK->PCHCTRL[clk_id].bit.CHEN); // Wait for clock enable
}
#endif

void SERCOM::initClockNVIC( void )
{
  int8_t idx = getSercomIndex();
  if(idx < 0) return; // We got a problem here

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)

  for(uint8_t i=0; i<4; i++) {
    NVIC_ClearPendingIRQ(sercomData[idx].irq[i]);
    NVIC_SetPriority(sercomData[idx].irq[i], SERCOM_NVIC_PRIORITY);
    NVIC_EnableIRQ(sercomData[idx].irq[i]);
  }

  setClockSource(idx, clockSource, true); // true  = core clock

#else // end if SAMD51 (prob SAMD21)

  uint8_t   clockId = sercomData[idx].clock;
  IRQn_Type IdNvic  = sercomData[idx].irqn;

  // Setting NVIC
  NVIC_ClearPendingIRQ(IdNvic);
  NVIC_SetPriority(IdNvic, SERCOM_NVIC_PRIORITY);
  NVIC_EnableIRQ(IdNvic);

  // Setting clock
  GCLK->CLKCTRL.reg =
    GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
    GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
    GCLK_CLKCTRL_CLKEN;

  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization

#endif // end !SAMD51

  getSercomFreqRef();
}
