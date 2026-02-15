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

/* =========================
 * ===== Sercom UART
 * =========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
  initClockNVIC();
  resetUART();

#ifdef USE_ZERODMA
#ifdef SERCOM0_DMAC_ID_TX
  int8_t id = getSercomIndex();
  if (id < 0)
    return;

  dmaSetCallbacks(SERCOM::dmaTxCallbackUART, SERCOM::dmaRxCallbackUART);

  if (_dmaConfigured)
    return;

  _dmaTxTrigger = SERCOM0_DMAC_ID_TX + (id * 2);
  _dmaRxTrigger = SERCOM0_DMAC_ID_RX + (id * 2);
  dmaInit(_dmaTxTrigger, _dmaRxTrigger);
#else
  (void)0;
#endif // SERCOM0_DMAC_ID_TX
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
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
}

void SERCOM::enableUART()
{
  //Setting  the enable bit to 1
  sercom->USART.CTRLA.bit.ENABLE = 0x1u;

  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(sercom->USART.SYNCBUSY.bit.ENABLE);
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
#else
  return false;
#endif
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
  if (_txnQueue.read(txn) && txn != nullptr)
  {
    _uart.active = false;
    _uart.currentTxn = nullptr;
    if (txn->onComplete)
      txn->onComplete(txn->user, static_cast<int>(error));
  }

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
  dmaSetCallbacks(SERCOM::dmaTxCallbackSPI, SERCOM::dmaRxCallbackSPI);
#endif

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
  if (!_txnQueue.store(txn))
    return false;
  if (!_spi.active) {
    startTransmissionSPI();
  }
  return true;
}

void SERCOM::serviceSPI(void)
{
  if (!_spi.active || _spi.currentTxn == nullptr)
    return;

  uint8_t flags = sercom->SPI.INTFLAG.reg;

  if (flags & SERCOM_SPI_INTFLAG_ERROR)
  {
    _spi.returnValue = SercomSpiError::BUF_OVERFLOW;
    sercom->SPI.INTFLAG.reg = SERCOM_SPI_INTFLAG_ERROR;
    deferStopSPI(_spi.returnValue);
    return;
  }

  SercomTxn* txn = _spi.currentTxn;

  if (flags & SERCOM_SPI_INTFLAG_RXC) {
    uint8_t rx = sercom->SPI.DATA.reg;
    if (txn->rxPtr && _spi.index > 0 && (_spi.index - 1) < _spi.length)
      txn->rxPtr[_spi.index - 1] = rx;
    if (_spi.index >= _spi.length) {
      sercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_DRE | SERCOM_SPI_INTENCLR_RXC | SERCOM_SPI_INTENCLR_ERROR;
      _spi.returnValue = SercomSpiError::SUCCESS;
      deferStopSPI(_spi.returnValue);
      return;
    }
  }

  if (flags & SERCOM_SPI_INTFLAG_DRE) {
    if (_spi.index < _spi.length) {
      uint8_t out = 0xFF;
      if (txn->txPtr)
        out = txn->txPtr[_spi.index];
      sercom->SPI.DATA.reg = out;
      _spi.index++;
      return;
    }
    sercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENCLR_DRE;
  }
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
  if (_txnQueue.read(txn) && txn != nullptr)
  {
    _spi.active = false;
    _spi.currentTxn = nullptr;
    if (txn->onComplete)
      txn->onComplete(txn->user, static_cast<int>(error));
  }

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
  //Setting the Software Reset bit to 1
  sercom->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
}

void SERCOM::enableSPI()
{
  //Setting the enable bit to 1
  sercom->SPI.CTRLA.bit.ENABLE = 1;

  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
}

void SERCOM::disableSPI()
{
  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  sercom->SPI.CTRLA.bit.ENABLE = 0;
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
  //I2CM OR I2CS, no matter SWRST is the same bit.

  //Setting the Software bit to 1
  sercom->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);

  _wire = WireConfig{};
}
void SERCOM::initWIRE(void)
{
  if (!_wire.inited) {
    uint8_t idx = getSercomIndex();
    initClockNVIC();
    registerService(idx, static_cast<ServiceFn>(&SERCOM::stopTransmissionWIRE));
    _wire.inited = true;
  }
#ifdef USE_ZERODMA
  initWireDma();
#endif
}

#ifdef USE_ZERODMA
void SERCOM::initWireDma(void)
{
#ifdef SERCOM0_DMAC_ID_TX
  if (_dmaConfigured)
    return;

  int8_t id = getSercomIndex();
  if (id < 0)
    return;

  _dmaTxTrigger = SERCOM0_DMAC_ID_TX + (id * 2);
  _dmaRxTrigger = SERCOM0_DMAC_ID_RX + (id * 2);
  dmaSetCallbacks(SERCOM::dmaTxCallbackWIRE, SERCOM::dmaRxCallbackWIRE);
  dmaInit(_dmaTxTrigger, _dmaRxTrigger);
#else
  (void)0;
#endif
}
#endif

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

void SERCOM::registerWireReceive(void (*cb)(void* user, int length), void* user)
{
  _wireDeferredCb = cb;
  _wireDeferredUser = user;
}

void SERCOM::deferWireReceive(int length)
{
  _wireDeferredLength = length;
  _wireDeferredPending = true;
  setPending((uint8_t)getSercomIndex());
}

void SERCOM::setMasterWIRE(void)
{
  disableWIRE();

  sercom->I2CM.CTRLB.reg = _wire.ctrlb |SERCOM_I2CM_CTRLB_QCEN;
  sercom->I2CM.BAUD.reg = _wire.baud;
  bool sclsm = (_wire.masterSpeed == 0x2);

  // Set master mode and clock settings
  sercom->I2CM.CTRLA.reg = _wire.ctrla |
                           SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION) |
                           SERCOM_I2CM_CTRLA_SPEED(_wire.masterSpeed) |
                           (sclsm ? SERCOM_I2CM_CTRLA_SCLSM : 0 );

  while (sercom->I2CM.SYNCBUSY.bit.ENABLE != 0) ;

  // Setting bus idle mode
  sercom->I2CM.STATUS.bit.BUSSTATE = 1 ;
  while (sercom->I2CM.SYNCBUSY.bit.SYSOP != 0) ;

  // Disable slave interrupts: address match, data ready, stop/restart.
  sercom->I2CS.INTENCLR.reg = SERCOM_I2CS_INTENSET_AMATCH |
                              SERCOM_I2CS_INTENSET_DRDY |
                              SERCOM_I2CS_INTENSET_PREC;
}

void SERCOM::setSlaveWIRE(void)
{
  disableWIRE();

  sercom->I2CS.ADDR.reg = _wire.addr;
  sercom->I2CS.CTRLB.reg = _wire.ctrlb;
  bool sclsm = (_wire.slaveSpeed == 0x2);

  // Set master mode and clock settings
  sercom->I2CS.CTRLA.reg = _wire.ctrla |
                           SERCOM_I2CS_CTRLA_MODE(I2C_SLAVE_OPERATION) |
                           SERCOM_I2CS_CTRLA_SPEED(_wire.slaveSpeed) |
                           (sclsm ? SERCOM_I2CS_CTRLA_SCLSM : 0 );

  while (sercom->I2CS.SYNCBUSY.bit.ENABLE != 0) ;

  // Setting bus idle mode
  sercom->I2CS.STATUS.bit.BUSSTATE = 1 ;
  while (sercom->I2CS.SYNCBUSY.bit.SYSOP != 0) ;

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
  uint8_t clockStretchMode; // See: 28.6.2.4.6 (SERCOM I2C Highspeed mode)

  if (baudrate <= topSpeeds[0]) {
    speedBit = 0; // Standard/Fast mode up to 400 khz
    clockStretchMode = 0;
  } else if (baudrate <= topSpeeds[1]) {
    speedBit = 1; // Fast mode+ up to 1 Mhz
    clockStretchMode = 0;
  } else {
    // High speed up to 3.4 Mhz
    speedBit = 2;
    clockStretchMode = 1;
  }

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


SercomTxn* SERCOM::startTransmissionWIRE(void)
{
  SercomTxn* txn = nullptr;
  if (!_txnQueue.peek(txn) || txn == nullptr)
    return nullptr;

  _wire.currentTxn = txn;
  _wire.txnIndex = 0;
  _wire.txnLength = txn->length;
  
  const bool read = (txn->config & I2C_CFG_READ) != 0;
  uint16_t addr = (txn->config & I2C_CFG_10BIT) ? I2C_ADDR(txn->address) : I2C_ADDR7(txn->address);
  addr = (uint16_t)((addr << 1) | (read ? 1u : 0u));

  // If another master owns the bus or the last bus owner has not properly
  // sent a stop, return failure early. This will prevent some misbehaved
  // devices from deadlocking here at the cost of the caller being responsible
  // for retrying the failed transmission. See SercomWireBusState for the
  // possible bus states.
  if(!isBusOwnerWIRE())
  {
    if (isArbLostWIRE() && !isBusIdleWIRE()) {
      stopTransmissionWIRE(SercomWireError::ARBITRATION_LOST);
      return nullptr;
    }
    if (isBusUnknownWIRE()) {
      stopTransmissionWIRE(SercomWireError::BUS_STATE_UNKNOWN);
      return nullptr;
    }
  }

  uint32_t addrReg = SERCOM_I2CM_ADDR_ADDR(addr) |
                     ((txn->config & I2C_CFG_10BIT) ? SERCOM_I2CM_ADDR_TENBITEN : 0) |
                     ((_wire.masterSpeed == 0x2) ? SERCOM_I2CM_ADDR_HS : 0);

#ifdef USE_ZERODMA
  setWireDma(txn->length > 0 && txn->length < 256 && (txn->config & I2C_CFG_STOP));

  if (isWireDma())
  {
    if (!_dmaConfigured)
      dmaInit(_dmaTxTrigger, _dmaRxTrigger);

    if (!_dmaConfigured || !_dmaTx || !_dmaRx) {
      stopTransmissionWIRE(SercomWireError::OTHER);
      return nullptr;
    }
    
    addrReg |= SERCOM_I2CM_ADDR_LENEN | SERCOM_I2CM_ADDR_LEN((uint8_t)txn->length);
  }
#endif
  
  // Send start and address (non-blocking; ISR handles MB/SB)
  _wire.active = true;
  sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_ERROR | SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB;
  sercom->I2CM.ADDR.reg = addrReg;

  return txn;
}

bool SERCOM::enqueueWIRE(SercomTxn* txn)
{
  if (txn == nullptr)
    return false;
  if (!_txnQueue.store(txn))
    return false;
  if (!_wire.active)
    return startTransmissionWIRE() != nullptr;
  return true;
}

void SERCOM::deferStopWIRE(SercomWireError error)
{
  _wire.returnValue = error;
  setPending((uint8_t)getSercomIndex());
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

  SercomTxn* txn = nullptr;
  _txnQueue.peek(txn);

  if (error == SercomWireError::BUS_STATE_UNKNOWN) {
    sercom->I2CM.STATUS.bit.BUSSTATE = 1;

    while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ;

    if (txn)
      startTransmissionWIRE();

    return txn;
  }

  if (error == SercomWireError::ARBITRATION_LOST || error == SercomWireError::BUS_ERROR) {
    sercom->I2CM.STATUS.bit.ARBLOST = 1; // Clear arbitration lost flag
    sercom->I2CM.INTFLAG.reg = SERCOM_I2CM_INTFLAG_ERROR;

    if (txn)
      startTransmissionWIRE();

    return txn;
  }

  // Callbacks are expected to run in non-ISR context (main loop/PendSV).
  if (_txnQueue.read(txn) && txn != nullptr) {
    _wire.active = false;
    _wire.currentTxn = nullptr;
    if (txn->onComplete)
      txn->onComplete(txn->user, static_cast<int>(error));
  }

  SercomTxn* next = nullptr;

  if (_txnQueue.peek(next) && next)
    startTransmissionWIRE();

  if (_wireDeferredPending && _wireDeferredCb) {
    _wireDeferredPending = false;
    _wireDeferredCb(_wireDeferredUser, _wireDeferredLength);
  }

  return txn;
}

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)

static const struct {
  Sercom   *sercomPtr;
  uint8_t   id_core;
  uint8_t   id_slow;
  IRQn_Type irq[4];
} sercomData[] = {
  { SERCOM0, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW,
    SERCOM0_0_IRQn, SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_3_IRQn },
  { SERCOM1, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW,
    SERCOM1_0_IRQn, SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_3_IRQn },
  { SERCOM2, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW,
    SERCOM2_0_IRQn, SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_3_IRQn },
  { SERCOM3, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW,
    SERCOM3_0_IRQn, SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_3_IRQn },
  { SERCOM4, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW,
    SERCOM4_0_IRQn, SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_3_IRQn },
  { SERCOM5, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW,
    SERCOM5_0_IRQn, SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_3_IRQn },
#if defined(SERCOM6)
  { SERCOM6, SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW,
    SERCOM6_0_IRQn, SERCOM6_1_IRQn, SERCOM6_2_IRQn, SERCOM6_3_IRQn },
#endif
#if defined(SERCOM7)
  { SERCOM7, SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW,
    SERCOM7_0_IRQn, SERCOM7_1_IRQn, SERCOM7_2_IRQn, SERCOM7_3_IRQn },
#endif
};

#else // end if SAMD51 (prob SAMD21)

static const struct {
  Sercom   *sercomPtr;
  uint8_t   clock;
  IRQn_Type irqn;
} sercomData[] = {
  SERCOM0, GCM_SERCOM0_CORE, SERCOM0_IRQn,
  SERCOM1, GCM_SERCOM1_CORE, SERCOM1_IRQn,
  SERCOM2, GCM_SERCOM2_CORE, SERCOM2_IRQn,
  SERCOM3, GCM_SERCOM3_CORE, SERCOM3_IRQn,
#if defined(SERCOM4)
  SERCOM4, GCM_SERCOM4_CORE, SERCOM4_IRQn,
#endif
#if defined(SERCOM5)
  SERCOM5, GCM_SERCOM5_CORE, SERCOM5_IRQn,
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
SERCOM::DmaStatus SERCOM::dmaInit(uint8_t txTrigger, uint8_t rxTrigger)
{
  if (_dmaConfigured)
    return DmaStatus::Ok;

  _dmaTxTrigger = txTrigger;
  _dmaRxTrigger = rxTrigger;

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
    _dmaTxDesc = _dmaTx->addDescriptor(&_dmaDummy, (void*)&sercom->I2CM.DATA.reg, 1, DMA_BEAT_SIZE_BYTE, true, false);
  if (_dmaRxDesc == nullptr)
    _dmaRxDesc = _dmaRx->addDescriptor((void*)&sercom->I2CM.DATA.reg, &_dmaDummy, 1, DMA_BEAT_SIZE_BYTE, false, true);
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

SERCOM::DmaStatus SERCOM::dmaStartDuplex(const void* txSrc, void* rxDst, void* txReg, void* rxReg, size_t len,
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
  if (sercomId >= kSercomCount || sercomId < 0)
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
