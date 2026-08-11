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
#include "SERCOM_WireBusErrorPolicy.h"
#include "Arduino.h"
#include "PendSV.h"
#include "variant.h"

#ifdef USE_ZERODMA
#include <Adafruit_ZeroDMA.h>
#endif // USE_ZERODMA

#ifndef WIRE_RISE_TIME_NANOSECONDS
// Default rise time in nanoseconds, based on 4.7K ohm pull up resistors
// you can override this value in your variant if needed
#define WIRE_RISE_TIME_NANOSECONDS 125
#endif // !WIRE_RISE_TIME_NANOSECONDS

#if defined(__SAME53__) || defined(__SAME54__)
SERCOM::SERCOM(sercom_registers_t *s)
#else
SERCOM::SERCOM(Sercom *s)
#endif // __SAME53__ / __SAME54__
{
  sercom = s;
  int8_t idx = getSercomIndex();
  if (idx >= 0 && idx < (int8_t)kSercomCount)
    s_instances[idx] = this;

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) ||       \
    defined(__SAME54__)
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
#endif // SERCOM_SPI_FREQ_REF == F_CPU
#endif // __SAMD51__ / __SAME51__ || __SAME53__ / __SAME54__
}

void SERCOM::resetSERCOM()
{
  // UART, SPI, I2CS, and I2CM use the same SWRST and DBGCTRL bits, so this works for all modes
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
#else
  sercom->USART.CTRLA.bit.SWRST = 1;
#endif // __SAME53__ / __SAME54__

  waitSyncBusySwrst();

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_DBGCTRL &= ~SERCOM_USART_INT_DBGCTRL_DBGSTOP_Msk;
#else
  // DBGCTRL is not affected by SWRST, so explicitly clear it here to ensure debug behavior is
  // consistent after reset
  sercom->USART.DBGCTRL.bit.DBGSTOP = 0;
#endif // __SAME53__ / __SAME54__
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

  uint32_t baudTimes8 = 0;
  if (mode == UART_INT_CLOCK) {
    const uint16_t sampleRateValue = (sampleRate == SAMPLE_RATE_x16) ? 16 : 8;

    // Asynchronous fractional mode (Table 24-2 in datasheet)
    //   BAUD = fref / (sampleRateValue * fbaud)
    // (multiply by 8, to calculate fractional piece)
    baudTimes8 = (freqRef * 8) / (sampleRateValue * baudrate);
  }

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE(mode) |
                                    SERCOM_USART_INT_CTRLA_SAMPR(sampleRate);
  sercom->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk |
                                      SERCOM_USART_INT_INTENSET_ERROR_Msk;

  if (mode == UART_INT_CLOCK) {
    sercom->USART_INT.SERCOM_BAUD =
        SERCOM_USART_INT_BAUD_FRAC_FP(baudTimes8 % 8) |
        SERCOM_USART_INT_BAUD_FRAC_BAUD(baudTimes8 / 8);
  }
#else
  // Setting the CTRLA register
  sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |
                            SERCOM_USART_CTRLA_SAMPR(sampleRate);

  // Setting the Interrupt register
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  // Received complete
                               SERCOM_USART_INTENSET_ERROR; // All others errors

  if (mode == UART_INT_CLOCK) {
    sercom->USART.BAUD.FRAC.FP = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
  }
#endif // __SAME53__ / __SAME54__
}

void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder,
                       SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_CTRLA |=
      SERCOM_USART_INT_CTRLA_FORM(parityMode == SERCOM_NO_PARITY ? 0 : 1) |
      SERCOM_USART_INT_CTRLA_DORD(dataOrder);
  sercom->USART_INT.SERCOM_CTRLB |=
      SERCOM_USART_INT_CTRLB_CHSIZE(charSize)   |
      SERCOM_USART_INT_CTRLB_SBMODE(nbStopBits) |
      SERCOM_USART_INT_CTRLB_PMODE(parityMode == SERCOM_NO_PARITY ? 0 : parityMode);
#else
  // Setting the CTRLA register
  sercom->USART.CTRLA.reg |=
      SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1)) |
      (dataOrder << SERCOM_USART_CTRLA_DORD_Pos);

  // Setting the CTRLB register
  sercom->USART.CTRLB.reg |=
      SERCOM_USART_CTRLB_CHSIZE(charSize)           |
      (nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos) |
      ((parityMode == SERCOM_NO_PARITY ? 0 : parityMode) << SERCOM_USART_CTRLB_PMODE_Pos);
#endif // __SAME53__ / __SAME54__
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_TXPO(txPad) |
                                    SERCOM_USART_INT_CTRLA_RXPO(rxPad);
  sercom->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN_Msk |
                                    SERCOM_USART_INT_CTRLB_RXEN_Msk;
#else
  // Setting the CTRLA register
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |
                             SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::resetUART()
{
  _txnQueue.clear();
  _uart.currentTxn = nullptr;
  _uart.index = 0;
  _uart.length = 0;
  _uart.active = false;
  _uart.returnValue = SercomUartError::SUCCESS;

#ifdef USE_ZERODMA
  _uart.dmaNeedTx = false;
  _uart.dmaNeedRx = false;
  _uart.dmaTxDone = false;
  _uart.dmaRxDone = false;
  dmaAbortTx();
  dmaAbortRx();
#endif // USE_ZERODMA

  resetSERCOM();
}

void SERCOM::flushUART()
{
  // Skip checking transmission completion if data register is empty
  if (isDataRegisterEmptyUART())
    return;

  // Wait for transmission to complete
#if defined(__SAME53__) || defined(__SAME54__)
  while ((sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk) == 0) ;
#else
  while (!sercom->USART.INTFLAG.bit.TXC) ;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::clearStatusUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_STATUS = SERCOM_USART_INT_STATUS_RESETVALUE;
#else
  // Reset (with 0) the STATUS register
  sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::availableDataUART()
{
  // RXC: Receive Complete
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.RXC;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isUARTError()
{
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_ERROR_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.ERROR;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::acknowledgeUARTError()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_ERROR_Msk;
#else
  sercom->USART.INTFLAG.bit.ERROR = 1;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::enableReceiveCompleteInterruptUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk;
#else
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::disableReceiveCompleteInterruptUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_RXC_Msk;
#else
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isBufferOverflowErrorUART()
{
  // BUFOVF: Buffer Overflow
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_BUFOVF_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.BUFOVF;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isFrameErrorUART()
{
  // FERR: Frame Error
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_FERR_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.FERR;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::clearFrameErrorUART()
{
  // Clear FERR bit writing 1 status bit
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_STATUS = SERCOM_USART_INT_STATUS_FERR_Msk;
#else
  sercom->USART.STATUS.bit.FERR = 1;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isParityErrorUART()
{
  // PERR: Parity Error
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_PERR_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.PERR;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isDataRegisterEmptyUART()
{
  // DRE: Data Register Empty
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.DRE;
#endif // __SAME53__ / __SAME54__
}

uint8_t SERCOM::readDataUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  return (uint8_t)sercom->USART_INT.SERCOM_DATA;
#else
  return sercom->USART.DATA.bit.DATA;
#endif // __SAME53__ / __SAME54__
}

int SERCOM::writeDataUART(uint8_t data)
{
  // Wait for data register to be empty
  while (!isDataRegisterEmptyUART());

  // Put data into DATA register
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_DATA = data;
#else
  sercom->USART.DATA.reg = (uint16_t)data;
#endif // __SAME53__ / __SAME54__
  return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk;
#else
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
#endif // __SAME53__ / __SAME54__
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk;
#else
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
#endif // __SAME53__ / __SAME54__
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
  _uart.returnValue = SercomUartError::SUCCESS;

#ifdef USE_ZERODMA
  _uart.useDma = _dmaConfigured;
#else
  _uart.useDma = false;
#endif // USE_ZERODMA

  if (!_uart.useDma)
  {
    _uart.active = false;
    _uart.currentTxn = nullptr;
    _uart.returnValue = SercomUartError::UNKNOWN_ERROR;
    return false;
  }

#ifdef USE_ZERODMA
#if defined(__SAME53__) || defined(__SAME54__)
  void* dataReg = (void*)&sercom->USART_INT.SERCOM_DATA;
#else
  void* dataReg = (void*)&sercom->USART.DATA.reg;
#endif // __SAME53__ / __SAME54__
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
    _uart.active = false;
    _uart.currentTxn = nullptr;
    _uart.returnValue = SercomUartError::UNKNOWN_ERROR;
    return false;
  }
  return true;
#else
  return false;
#endif // USE_ZERODMA
}

bool SERCOM::enqueueUART(SercomTxn* txn)
{
  if (txn == nullptr)
    return false;
#ifdef USE_ZERODMA
  if (!_dmaConfigured)
    return false;
#endif // USE_ZERODMA
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

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_CTRLA = SERCOM_SPIM_CTRLA_MODE_SPI_MASTER |
                               SERCOM_SPIM_CTRLA_DOPO(mosi) |
                               SERCOM_SPIM_CTRLA_DIPO(miso) |
                               SERCOM_SPIM_CTRLA_DORD(dataOrder);
  sercom->SPIM.SERCOM_CTRLB = SERCOM_SPIM_CTRLB_CHSIZE(charSize) |
                               SERCOM_SPIM_CTRLB_RXEN_Msk;
#else
  // Setting the CTRLA register
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3) |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          (dataOrder << SERCOM_SPI_CTRLA_DORD_Pos);
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN; // Active the SPI receiver.
#endif // __SAME53__ / __SAME54__
  waitSyncBusyCtrlB();
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

  if (_spi.useDma) {
#if defined(__SAME53__) || defined(__SAME54__)
    void* dataReg = (void*)&sercom->SPIM.SERCOM_DATA;
#else
    void* dataReg = (void*)&sercom->SPI.DATA.reg;
#endif // __SAME53__ / __SAME54__
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
  }
#else
  _spi.useDma = false;
#endif // USE_ZERODMA

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_INTENSET = SERCOM_SPIM_INTENSET_DRE_Msk |
                                 SERCOM_SPIM_INTENSET_RXC_Msk |
                                 SERCOM_SPIM_INTENSET_ERROR_Msk;
#else
  sercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_DRE |
                             SERCOM_SPI_INTENSET_RXC |
                             SERCOM_SPI_INTENSET_ERROR;
#endif // __SAME53__ / __SAME54__
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

  if (!_spi.active)
    startTransmissionSPI();

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
  // Extract data from clockMode
  int cpha, cpol;

  if ((clockMode & (0x1ul)) == 0)
    cpha = 0;
  else
    cpha = 1;

  if ((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  // Setting the CTRLA register
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPHA(cpha) |
                               SERCOM_SPIM_CTRLA_CPOL(cpol);
  sercom->SPIM.SERCOM_BAUD = SERCOM_SPIM_BAUD_BAUD(calculateBaudrateSynchronous(baudrate));
#else
  sercom->SPI.CTRLA.reg |= (cpha << SERCOM_SPI_CTRLA_CPHA_Pos) |
                           (cpol << SERCOM_SPI_CTRLA_CPOL_Pos);

  // Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
#endif // __SAME53__ / __SAME54__
}

void SERCOM::resetSPI()
{
  resetSERCOM();
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
  // Register enable-protected
  disableSPI();

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_CTRLA =
      (sercom->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_DORD_Msk) |
      SERCOM_SPIM_CTRLA_DORD(dataOrder);
#else
  sercom->SPI.CTRLA.bit.DORD = dataOrder;
#endif // __SAME53__ / __SAME54__

  enableSPI();
}

SercomDataOrder SERCOM::getDataOrderSPI()
{
#if defined(__SAME53__) || defined(__SAME54__)
  return (sercom->SPIM.SERCOM_CTRLA & SERCOM_SPIM_CTRLA_DORD_Msk) ? LSB_FIRST : MSB_FIRST;
#else
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
#endif // __SAME53__ / __SAME54__
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
  disableSPI(); // Register is enable-protected
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_BAUD = SERCOM_SPIM_BAUD_BAUD(calculateBaudrateSynchronous(freqRef / divider));
#else
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(freqRef / divider);
#endif // __SAME53__ / __SAME54__
  enableSPI();
}

void SERCOM::setClockModeSPI(SercomSpiClockMode clockMode)
{
  int cpha, cpol;
  if ((clockMode & (0x1ul)) == 0)
    cpha = 0;
  else
    cpha = 1;

  if ((clockMode & (0x2ul)) == 0)
    cpol = 0;
  else
    cpol = 1;

  // Register enable-protected
  disableSPI();

#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_CTRLA =
      (sercom->SPIM.SERCOM_CTRLA & ~(SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) |
      SERCOM_SPIM_CTRLA_CPOL(cpol) | SERCOM_SPIM_CTRLA_CPHA(cpha);
#else
  sercom->SPI.CTRLA.bit.CPOL = cpol;
  sercom->SPI.CTRLA.bit.CPHA = cpha;
#endif // __SAME53__ / __SAME54__

  enableSPI();
}

uint8_t SERCOM::transferDataSPI(uint8_t data)
{
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->SPIM.SERCOM_DATA = data;
  while ((sercom->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) == 0);
  return (uint8_t)sercom->SPIM.SERCOM_DATA;
#else
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while (sercom->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception

  return sercom->SPI.DATA.bit.DATA; // Reading data
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isBufferOverflowErrorSPI()
{
#if defined(__SAME53__) || defined(__SAME54__)
  return sercom->SPIM.SERCOM_STATUS & SERCOM_SPIM_STATUS_BUFOVF_Msk;
#else
  return sercom->SPI.STATUS.bit.BUFOVF;
#endif // __SAME53__ / __SAME54__
}

bool SERCOM::isDataRegisterEmptySPI()
{
  // DRE: Data Register Empty
#if defined(__SAME53__) || defined(__SAME54__)
  return sercom->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk;
#else
  return sercom->SPI.INTFLAG.bit.DRE;
#endif // __SAME53__ / __SAME54__
}

//bool SERCOM::isTransmitCompleteSPI()
//{
//  // TXC: Transmit complete
//  return sercom->SPI.INTFLAG.bit.TXC;
//}
//
//bool SERCOM::isReceiveCompleteSPI()
//{
//  // RXC: Receive complete
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
  _wire.slaveTransactionActive = false;
  _wire.currentTxn = nullptr;
  _wire.slaveTxn = nullptr;
  _wire.txnIndex = 0;
  _wire.txnLength = 0;
  _wire.returnValue = SercomWireError::SUCCESS;
  _wire.retryCount = 0;
  _wire.releasePending = false;
  _wire.abortPending = false;
  _wire.busErrorRecoveryPending = false;
  _wire.busErrorRecoveryStartedUs = 0;
  _wire.busErrorRecoveryDeadlineActive = false;
  _wire.busErrorRecoveryDeadlineStartedUs = 0;
  _wire.busErrorRecoveryArbitrationLost = false;
  _wire.busErrorRecoveryCommandReady = false;
#ifdef USE_ZERODMA
  _wireRxSuspendPending = false;
#endif

  // Clear deferred callbacks (from slave/receive operations)
  _wireCallbackUser = nullptr;
  _wireReceiveCb = nullptr;
  _wireRequestCb = nullptr;
  _wireReceivePending = false;
  _wireRequestPending = false;
}

void SERCOM::initWIRE(void)
{
  if (_wire.inited)  // If already initialized, return
    return;

  uint8_t idx = getSercomIndex();
  initClockNVIC();
  registerService(idx, static_cast<ServiceFn>(&SERCOM::stopTransmissionWIRE));

#ifdef USE_ZERODMA
  dmaSetCallbacks(SERCOM::dmaTxCallbackWIRE, SERCOM::dmaRxCallbackWIRE,
                  SERCOM::dmaErrorCallbackWIRE);
  if (idx >= 0)
    dmaInit(idx);
  if (_dmaRx)
    _dmaRx->setCallback(SERCOM::dmaRxSuspendCallbackWIRE,
                        DMA_CALLBACK_CHANNEL_SUSPEND);
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
  uint32_t generalCall = 0;
  uint32_t tenBit = 0;
#if defined(__SAME53__) || defined(__SAME54__)
  generalCall = enableGeneralCall ? SERCOM_I2CS_ADDR_GENCEN(1) : 0u;
  tenBit = enable10Bit ? SERCOM_I2CS_ADDR_TENBITEN(1) : 0u;
#else
  generalCall = enableGeneralCall ? SERCOM_I2CS_ADDR_GENCEN : 0u;
  tenBit = enable10Bit ? SERCOM_I2CS_ADDR_TENBITEN : 0u;
#endif // __SAME53__ / __SAME54__

  _wire.slaveSpeed = speed;
  _wire.addr = SERCOM_I2CS_ADDR_ADDR(ucAddress & mask) |
               SERCOM_I2CS_ADDR_ADDRMASK(0x00ul)      |
               tenBit                                  |
               generalCall;
  _wire.slaveConfigured = true;
  setSlaveWIRE();
}

void SERCOM::registerReceiveWIRE(WireReceiveCallback callback, void *user) {
  _wireReceiveCb = callback;
  _wireCallbackUser = user;
}

void SERCOM::registerRequestWIRE(WireRequestCallback callback, void *user) {
  _wireRequestCb = callback;
  _wireCallbackUser = user;
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
  initWIRE();

  setBaudrateWIRE(baudrate);
  _wire.slaveConfigured = false;
  setMasterWIRE();
}

void SERCOM::deferReceiveWIRE(void) {
  _wireReceivePending = true;
  _wire.returnValue = SercomWireError::SUCCESS;
#ifdef USE_ZERODMA
  // The RX suspend callback publishes the completed byte count and owns the
  // retirement dispatch. Do not let PendSV abort the channel first.
  if (_dmaRxActive)
    return;
#endif // USE_ZERODMA
  setPending((uint8_t)getSercomIndex());
}

void SERCOM::deferRequestWIRE(void) {
  _wireRequestPending = true;
  _wire.returnValue = SercomWireError::SUCCESS;
#ifdef USE_ZERODMA
  // A repeated START may request a slave response while the preceding RX DMA
  // descriptor is still being suspended. Its suspend callback owns the next
  // PendSV dispatch so the receive length is published before onRequest can
  // replace the active transaction.
  if (_wireRxSuspendPending || _dmaRxActive)
    return;
#endif // USE_ZERODMA
  setPending((uint8_t)getSercomIndex());
}

void SERCOM::deferReceiveCompleteWIRE(void) {
#ifdef USE_ZERODMA
  // BTCNT for the active channel lives in the DMAC's internal descriptor.
  // Suspend first so hardware commits that counter to WRBADDR; the suspend
  // callback snapshots it, aborts the channel, and defers normal retirement.
  if (_dmaRxActive) {
    if (!_wireRxSuspendPending && _dmaRx) {
      _wireRxSuspendPending = true;
      _dmaRx->suspend();
    }
    return;
  }
#endif // USE_ZERODMA
  deferStopWIRE(SercomWireError::SUCCESS);
}

#ifdef USE_ZERODMA
void SERCOM::dmaRxSuspendCallbackWIRE(Adafruit_ZeroDMA *dma) {
  SERCOM *inst = findDmaOwner(dma, false);
  if (!inst || !inst->_wireRxSuspendPending)
    return;

  inst->_wireRxSuspendPending = false;
  size_t transferred = 0;
  uint16_t remaining = 0;
  const uint8_t channel = dma->getChannel();
  if (channel < DMAC_CH_NUM) {
#if defined(__SAME53__) || defined(__SAME54__)
    const DmacDescriptor *writeback =
        reinterpret_cast<const DmacDescriptor *>(DMAC_REGS->DMAC_WRBADDR);
    remaining = writeback[channel].DMAC_BTCNT;
#else
    const DmacDescriptor *writeback =
        reinterpret_cast<const DmacDescriptor *>(DMAC->WRBADDR.reg);
    remaining = writeback[channel].BTCNT.reg;
#endif // __SAME53__ / __SAME54__
    if (inst->_wire.dmaBlockLength >= remaining)
      transferred = inst->_wire.dmaBlockLength - remaining;
  }
  inst->_wire.txnIndex += transferred;
  inst->dmaAbortRx();
  inst->deferStopWIRE(SercomWireError::SUCCESS);
}
#endif // USE_ZERODMA

SercomTxn *SERCOM::retireSlaveTransactionWIRE(bool reserveForFollowup) {
  SercomTxn *txn = _wire.currentTxn;
  _wire.retryCount = 0;
  _wire.active = reserveForFollowup;
  _wire.slaveTransactionActive = false;
  _wire.currentTxn = nullptr;
  return txn;
}

void SERCOM::setMasterWIRE(void)
{
  // Errata: do not enable QCEN when SCLSM=1 (bus error). Hs-mode requires SCLSM=1,
  // so master Hs-mode must be DMA-only and STOP-only (no repeated starts).
  disableWIRE();
  bool sclsm = (_wire.masterSpeed == 0x2);
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->I2CM.SERCOM_CTRLA = _wire.ctrla                                  |
                              SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION) |
                              SERCOM_I2CM_CTRLA_SPEED(_wire.masterSpeed)   |
                              (sclsm ? SERCOM_I2CM_CTRLA_SCLSM(1) : 0);
  sercom->I2CM.SERCOM_CTRLB = _wire.ctrlb;
  sercom->I2CM.SERCOM_BAUD = _wire.baud;
  enableWIRE();
  sercom->I2CS.SERCOM_INTENCLR = SERCOM_I2CS_INTENCLR_ERROR_Msk  |
                                 SERCOM_I2CS_INTENCLR_AMATCH_Msk |
                                 SERCOM_I2CS_INTENCLR_DRDY_Msk   |
                                 SERCOM_I2CS_INTENCLR_PREC_Msk;
#else
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
#endif // __SAME53__ / __SAME54__
}

void SERCOM::setSlaveWIRE(void)
{
  disableWIRE();
  bool sclsm = (_wire.slaveSpeed == 0x2);
#if defined(__SAME53__) || defined(__SAME54__)
  uint32_t slaveCtrlb = _wire.ctrlb;
  if (slaveCtrlb & SERCOM_I2CS_CTRLB_SMEN_Msk)
    slaveCtrlb &= ~SERCOM_I2CS_CTRLB_AACKEN_Msk;
  sercom->I2CS.SERCOM_CTRLA = _wire.ctrla                                 |
                              SERCOM_I2CS_CTRLA_MODE(I2C_SLAVE_OPERATION) |
                              SERCOM_I2CS_CTRLA_SPEED(_wire.slaveSpeed)   |
                              (sclsm ? SERCOM_I2CS_CTRLA_SCLSM(1) : 0);
  sercom->I2CS.SERCOM_CTRLB = slaveCtrlb;
  sercom->I2CS.SERCOM_ADDR = _wire.addr;
  enableWIRE();
  sercom->I2CS.SERCOM_INTENSET =
      SERCOM_I2CS_INTENSET_ERROR_Msk | SERCOM_I2CS_INTENSET_AMATCH_Msk;
#else
  uint32_t slaveCtrlb = _wire.ctrlb;
  if (slaveCtrlb & SERCOM_I2CS_CTRLB_SMEN)
    slaveCtrlb &= ~SERCOM_I2CS_CTRLB_AACKEN;
  sercom->I2CS.CTRLA.reg = SERCOM_I2CS_CTRLA_MODE(I2C_SLAVE_OPERATION) |
                           SERCOM_I2CS_CTRLA_SPEED(_wire.slaveSpeed)   |
                           (sclsm ? SERCOM_I2CS_CTRLA_SCLSM : 0 );
  sercom->I2CS.CTRLB.reg = slaveCtrlb;
  sercom->I2CS.ADDR.reg = _wire.addr;
  enableWIRE();
  // ERROR and AMATCH remain armed while the slave is idle. DRDY and PREC are
  // owned by an active transaction and are enabled in startTransmissionWIRE().
  sercom->I2CS.INTENSET.reg =
      SERCOM_I2CS_INTENSET_ERROR | SERCOM_I2CS_INTENSET_AMATCH;
#endif // __SAME53__ / __SAME54__
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
  // BUSY: The master waits until the bus is IDLE.
  // IDLE: A START is generated, the address is sent, and on ACK the master
  // holds SCL low with CLKHOLD set and MB asserted. OWNER: A repeated START is
  // generated; if the prior transaction was a read, the ACK/NACK for the read
  // is sent before the repeated START. The repeated START ADDR write must occur
  // while MB or SB is set. Writing ADDR also clears BUSERR, ARBLOST, MB, and
  // SB.

  if (isSlaveWIRE()) {
    SercomTxn *txn = _wire.currentTxn;
    if (txn == nullptr || txn->length == 0)
      return nullptr;
    const bool slaveTransmit = (txn->config & I2C_CFG_READ) != 0;
    const auto clearAmatch = [this]() {
#if defined(__SAME53__) || defined(__SAME54__)
      clearINTFLAG(SERCOM_I2CS_INTFLAG_AMATCH_Msk);
#else
      clearINTFLAG(SERCOM_I2CS_INTFLAG_AMATCH);
#endif // __SAME53__ / __SAME54__
    };

#ifdef USE_ZERODMA
    // Mirror the master engine selection: eligible transactions use DMA;
    // larger transactions remain on the interrupt-driven DATA path.
    if (isDmaWIRE()) {
      if (!_dmaConfigured)
        dmaInit(getSercomIndex());
      if (!_dmaConfigured || !_dmaTx || !_dmaRx)
        return nullptr;

      // AMATCH holds SCL low while software prepares the transfer. Arm the
      // DATA-triggered DMA channel before releasing AMATCH so the first DRDY
      // request cannot arrive before DMAC is ready.
      const bool started = slaveTransmit ? sendDataWIRE() : readDataWIRE();
      if (!started)
        return nullptr;
#if defined(__SAME53__) || defined(__SAME54__)
      enableInterrupts(SERCOM_I2CS_INTENSET_DRDY_Msk |
                       SERCOM_I2CS_INTENSET_PREC_Msk);
#else
      enableInterrupts(SERCOM_I2CS_INTENSET_DRDY | SERCOM_I2CS_INTENSET_PREC);
#endif // __SAME53__ / __SAME54__
      clearAmatch();
    } else {
#if defined(__SAME53__) || defined(__SAME54__)
      enableInterrupts(SERCOM_I2CS_INTENSET_DRDY_Msk |
                       SERCOM_I2CS_INTENSET_PREC_Msk);
#else
      enableInterrupts(SERCOM_I2CS_INTENSET_DRDY | SERCOM_I2CS_INTENSET_PREC);
#endif // __SAME53__ / __SAME54__
      clearAmatch();
    }
#else
    // The no-DMA build advances this transaction from slave DRDY interrupts.
    setDmaWIRE(false);
#if defined(__SAME53__) || defined(__SAME54__)
    enableInterrupts(SERCOM_I2CS_INTENSET_DRDY_Msk |
                     SERCOM_I2CS_INTENSET_PREC_Msk);
#else
    enableInterrupts(SERCOM_I2CS_INTENSET_DRDY | SERCOM_I2CS_INTENSET_PREC);
#endif // __SAME53__ / __SAME54__
    clearAmatch();
#endif // USE_ZERODMA
#if defined(__SAME53__) || defined(__SAME54__)
    enableInterrupts(SERCOM_I2CS_INTENSET_AMATCH_Msk);
#else
    enableInterrupts(SERCOM_I2CS_INTENSET_AMATCH);
#endif // __SAME53__ / __SAME54__
    return txn;
  }

  if (isBusUnknownWIRE()) {
    stopTransmissionWIRE(SercomWireError::BUS_STATE_UNKNOWN);
    return nullptr;
  }

  SercomTxn* txn = nullptr;

  if (!_txnQueue.peek(txn))
    return nullptr;

  if (txn != _wire.currentTxn) {
    _wire.retryCount = 0;
    _wire.busErrorRecoveryDeadlineActive = false;
  }

  _wire.currentTxn = txn;
  _wire.txnIndex = 0;
  _wire.txnLength = txn->length;

  setDmaWIRE(false);  // Reset DMA mode - let code below decide if DMA is used
  const bool read = (txn->config & I2C_CFG_READ) != 0;
  uint16_t addr = (txn->config & I2C_CFG_10BIT) ? I2C_ADDR(txn->address) : I2C_ADDR7(txn->address);
  addr = (uint16_t)((addr << 1) | (read ? 1u : 0u));

  if (!isBusOwnerWIRE()) {
    if (isBusUnknownWIRE()) {
      stopTransmissionWIRE(SercomWireError::BUS_STATE_UNKNOWN);
      return nullptr;
    }
  }

  bool hsMode = (_wire.masterSpeed == 0x2);
#if defined(__SAME53__) || defined(__SAME54__)
  bool sclsm = (sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SCLSM_Msk) != 0;
  uint32_t addrReg = SERCOM_I2CM_ADDR_ADDR(addr)                                         |
                     ((txn->config & I2C_CFG_10BIT) ? SERCOM_I2CM_ADDR_TENBITEN_Msk : 0) |
                     (hsMode ? SERCOM_I2CM_ADDR_HS(1) : 0);
#else
  bool sclsm = sercom->I2CM.CTRLA.bit.SCLSM;
  uint32_t addrReg = SERCOM_I2CM_ADDR_ADDR(addr)                                     |
                     ((txn->config & I2C_CFG_10BIT) ? SERCOM_I2CM_ADDR_TENBITEN : 0) |
                     (hsMode ? SERCOM_I2CM_ADDR_HS : 0);
#endif // __SAME53__ / __SAME54__

  if (hsMode || sclsm) {
#ifndef USE_ZERODMA
    stopTransmissionWIRE(SercomWireError::OTHER);
    return nullptr;
#endif // !USE_ZERODMA
    if (txn->length > 255) {
      stopTransmissionWIRE(SercomWireError::DATA_TOO_LONG);
      return nullptr;
    }

    if (
#if defined(__SAME53__) || defined(__SAME54__)
        sercom->I2CM.SERCOM_CTRLB & SERCOM_I2CM_CTRLB_QCEN_Msk
#else
        sercom->I2CM.CTRLB.bit.QCEN
#endif // __SAME53__ / __SAME54__
    ) {
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

#if defined(__SAME53__) || defined(__SAME54__)
    addrReg |= SERCOM_I2CM_ADDR_LENEN_Msk | SERCOM_I2CM_ADDR_LEN((uint8_t)txn->length);
#else
    addrReg |= SERCOM_I2CM_ADDR_LENEN | SERCOM_I2CM_ADDR_LEN((uint8_t)txn->length);
#endif // __SAME53__ / __SAME54__
  }
#endif // USE_ZERODMA

  // Send address (non-blocking; ISR handles ERROR/MB/SB)
  _wire.active = true;
#if defined(__SAME53__) || defined(__SAME54__)
  sercom->I2CM.SERCOM_INTENSET = SERCOM_I2CM_INTENSET_ERROR_Msk |
                                 SERCOM_I2CM_INTENSET_SB_Msk |
                                 SERCOM_I2CM_INTENSET_MB_Msk;
  sercom->I2CM.SERCOM_ADDR = addrReg;
#else
  sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_ERROR | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_MB;
  sercom->I2CM.ADDR.reg = addrReg; // ADDR is write synchronized so just wait for the MB/SB to know when synced
#endif // __SAME53__ / __SAME54__
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

  if (isSlaveWIRE() && _wire.currentTxn == _wire.slaveTxn &&
      !_wire.slaveTransactionActive) {
    _wire.active = false;
    _wire.currentTxn = nullptr;
    setMasterWIRE();
    return startTransmissionWIRE() != nullptr;
  }

  if (!_wire.active) {
    // dI2C uses a single SERCOM in both slave and master roles. A queued
    // master operation is the role-transition point; callers must not
    // tear down and re-begin Wire around every transaction.
    if (!isMasterWIRE())
      setMasterWIRE();

    return startTransmissionWIRE() != nullptr;
  }

  return true;
}

bool SERCOM::startNextQueuedWIRE(void) {
  if (_wire.active || _wire.currentTxn != nullptr)
    return false;

  SercomTxn *next = nullptr;
  if (!_txnQueue.peek(next))
    return false;

  if (!isMasterWIRE())
    setMasterWIRE();
  startTransmissionWIRE();
  return true;
}

bool SERCOM::abortWIRE(SercomWireError error) {
  if (!_wire.active || _wire.currentTxn == nullptr)
    return false;

  _wire.abortPending = true;
  _wire.abortError = error;
  setPending((uint8_t)getSercomIndex());
  return true;
}

bool SERCOM::serviceAbortWIRE(void) {
  if (!_wire.abortPending)
    return false;

#ifdef USE_ZERODMA
  dmaAbortTx();
  dmaAbortRx();
  // Reject a completion callback already pending for the aborted phase.
  setDmaWIRE(false);
#endif // USE_ZERODMA

  bool commandReady = false;
  if (isMasterWIRE()) {
#if defined(__SAME53__) || defined(__SAME54__)
    const uint32_t flags = sercom->I2CM.SERCOM_INTFLAG;
    commandReady =
        (flags & (SERCOM_I2CM_INTFLAG_MB_Msk | SERCOM_I2CM_INTFLAG_SB_Msk)) !=
        0u;
#else
    const uint8_t flags = sercom->I2CM.INTFLAG.reg;
    commandReady =
        (flags & (SERCOM_I2CM_INTFLAG_MB | SERCOM_I2CM_INTFLAG_SB)) != 0u;
#endif // __SAME53__ / __SAME54__
  }

  const bool busOwner = isBusOwnerWIRE();
  if (busOwner && commandReady)
    prepareCommandBitsWIRE(WIRE_MASTER_ACT_STOP);
  else if (busOwner) {
    // A timed-out owner with no MB/SB flag has no legal command point. Reapply
    // the cached master configuration to release the wedged transfer before
    // publishing completion. This remains inside the SERCOM event chain.
    setMasterWIRE();
    _wire.releasePending = false;
  }

  const SercomWireError error = _wire.abortError;
  _wire.abortPending = false;

  if (isMasterWIRE()) {

#if defined(__SAME53__) || defined(__SAME54__)
    sercom->I2CM.SERCOM_INTENCLR = SERCOM_I2CM_INTENCLR_ERROR_Msk |
                                   SERCOM_I2CM_INTENCLR_MB_Msk |
                                   SERCOM_I2CM_INTENCLR_SB_Msk;
#else
    sercom->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_ERROR |
                                SERCOM_I2CM_INTENCLR_MB |
                                SERCOM_I2CM_INTENCLR_SB;
#endif // __SAME53__ / __SAME54__
  }

  deferStopWIRE(error);
  return true;
}

void SERCOM::deferBusErrorRecoveryWIRE(bool arbitrationLost,
                                       bool commandReady) {
#ifdef USE_ZERODMA
  dmaAbortTx();
  dmaAbortRx();
  setDmaWIRE(false);
#endif
  _wire.busErrorRecoveryPending = true;
  _wire.busErrorRecoveryStartedUs = micros();
  if (!_wire.busErrorRecoveryDeadlineActive) {
    _wire.busErrorRecoveryDeadlineActive = true;
    _wire.busErrorRecoveryDeadlineStartedUs = _wire.busErrorRecoveryStartedUs;
  }
  _wire.busErrorRecoveryArbitrationLost = arbitrationLost;
  _wire.busErrorRecoveryCommandReady = commandReady;
  setPending((uint8_t)getSercomIndex());
}

SercomTxn *SERCOM::serviceBusErrorRecoveryWIRE(void) {
  if (!_wire.busErrorRecoveryPending)
    return _wire.currentTxn;

  if (_wire.abortPending) {
    _wire.busErrorRecoveryPending = false;
    return serviceAbortWIRE() ? _wire.currentTxn : nullptr;
  }

  SercomTxn *txn = _wire.currentTxn;
  if (!txn) {
    _wire.busErrorRecoveryPending = false;
    return nullptr;
  }

  if (simio::wire::busErrorDeadlineExpired(
          _wire.busErrorRecoveryDeadlineStartedUs, micros(),
          _wire.busErrorRecoveryDeadlineActive)) {
    _wire.busErrorRecoveryPending = false;
    _wire.busErrorRecoveryDeadlineActive = false;
    return stopTransmissionWIRE(SercomWireError::BUS_ERROR);
  }

  simio::wire::BusState busState = simio::wire::BusState::Unknown;
  if (isBusIdleWIRE())
    busState = simio::wire::BusState::Idle;
  else if (isBusOwnerWIRE())
    busState = simio::wire::BusState::Owner;
  else if (isBusBusyWIRE())
    busState = simio::wire::BusState::Busy;

  const auto action = simio::wire::decideBusErrorAction(
      {true, busState, _wire.busErrorRecoveryArbitrationLost,
       _wire.busErrorRecoveryCommandReady, false, true, true});
  if (action == simio::wire::BusErrorAction::RestartQueueHead) {
    _wire.busErrorRecoveryPending = false;
    return startTransmissionWIRE();
  }

  if (action == simio::wire::BusErrorAction::EstablishIdle) {
#if defined(__SAME53__) || defined(__SAME54__)
    sercom->I2CM.SERCOM_STATUS =
        (sercom->I2CM.SERCOM_STATUS & ~SERCOM_I2CM_STATUS_BUSSTATE_Msk) |
        SERCOM_I2CM_STATUS_BUSSTATE(WIRE_IDLE_STATE);
#else
    sercom->I2CM.STATUS.bit.BUSSTATE = WIRE_IDLE_STATE;
#endif // __SAME53__ / __SAME54__
    waitSyncBusySysOp();
    _wire.busErrorRecoveryPending = false;
    return startTransmissionWIRE();
  }

  constexpr uint32_t kBusErrorRecoveryTimeoutUs = 100u;
  const bool timedOut =
      static_cast<uint32_t>(micros() - _wire.busErrorRecoveryStartedUs) >=
      kBusErrorRecoveryTimeoutUs;
  if (simio::wire::decideBusErrorWaitAction(busState, timedOut) ==
      simio::wire::BusErrorAction::WaitForBusState) {
    setPending((uint8_t)getSercomIndex());
    return txn;
  }

  resetSERCOM();
  setMasterWIRE();
  _wire.busErrorRecoveryPending = false;
  return startTransmissionWIRE();
}

SercomTxn* SERCOM::stopTransmissionWIRE( void )
{
  return stopTransmissionWIRE( _wire.returnValue );
}

SercomTxn* SERCOM::stopTransmissionWIRE( SercomWireError error )
{
  constexpr uint32_t kWireStopSettleTimeoutUs = 100u;
  if (_wire.busErrorRecoveryPending)
    return serviceBusErrorRecoveryWIRE();
  // Policy: only auto-retry recoverable bus-state errors here. All other
  // errors are surfaced to the transaction callback for protocol handling.
  // Retry/backoff policy is intentionally deferred; a future change may add
  // a retry budget or tick-based delay if needed.
  SercomTxn *txn = _wire.currentTxn;

  SercomWireCompletionReport &report = _wire.lastCompletionReport;
  report = {};
  report.error = error;
  report.address = txn ? txn->address : 0u;
  report.requestedLength = txn ? static_cast<uint16_t>(txn->length) : 0u;
  report.transferredLength = static_cast<uint16_t>(_wire.txnIndex);
  report.role =
      isMasterWIRE() ? SercomWireRole::Master : SercomWireRole::Slave;
  report.read = txn && (txn->config & I2C_CFG_READ);

  const auto refreshCompletionReport = [this, &report]() {
    report.transferredLength = static_cast<uint16_t>(_wire.txnIndex);

    if (report.role == SercomWireRole::Slave) {
      report.bus = report.error == SercomWireError::SUCCESS
                       ? SercomWireBusDisposition::Released
                       : SercomWireBusDisposition::Unknown;
    } else {
      const uint16_t status = getSTATUS();
#if defined(__SAME53__) || defined(__SAME54__)
      const uint8_t busState =
          static_cast<uint8_t>((status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
                               SERCOM_I2CM_STATUS_BUSSTATE_Pos);
      const bool clockHeld =
          (status & SERCOM_I2CM_STATUS_CLKHOLD_Msk) != 0u;
#else
      const uint8_t busState =
          static_cast<uint8_t>((status & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
                               SERCOM_I2CM_STATUS_BUSSTATE_Pos);
      const bool clockHeld = (status & SERCOM_I2CM_STATUS_CLKHOLD) != 0u;
#endif // __SAME53__ / __SAME54__

      if (isBusOwnerWIRE())
        report.bus = SercomWireBusDisposition::LocalOwner;
      else if (clockHeld)
        report.bus = SercomWireBusDisposition::Unknown;
      else if (busState == WIRE_IDLE_STATE)
        report.bus = SercomWireBusDisposition::Released;
      else if (busState == WIRE_BUSY_STATE)
        report.bus = SercomWireBusDisposition::OtherOwner;
      else
        report.bus = SercomWireBusDisposition::Unknown;
    }

    if (report.error == SercomWireError::SUCCESS) {
      report.delivery = SercomWireDelivery::Complete;
    } else if (report.error == SercomWireError::NACK_ON_ADDRESS ||
               report.error == SercomWireError::QUEUE_FULL ||
               report.error == SercomWireError::BUS_CONFLICT ||
               report.error == SercomWireError::BUS_STATE_UNKNOWN) {
      report.delivery = report.transferredLength == 0u
                            ? SercomWireDelivery::None
                            : SercomWireDelivery::Ambiguous;
    } else if (report.error == SercomWireError::NACK_ON_DATA) {
      report.delivery = report.transferredLength == 0u
                            ? SercomWireDelivery::None
                            : SercomWireDelivery::Partial;
    } else if (report.read && report.transferredLength != 0u) {
      report.delivery = SercomWireDelivery::Partial;
    } else {
      report.delivery = SercomWireDelivery::Ambiguous;
    }
  };

  // Explicit timeout cancellation is a newer entrance into the shared
  // PendSV retirement service. Keep it outside the tested normal master
  // completion sequence below.
  if (_wire.abortPending) {
    serviceAbortWIRE();
    return txn;
  }

  bool isMaster = isMasterWIRE();

  if (!isMaster) {
    const bool stopDetected = isStopDetectedWIRE();
#if defined(__SAME53__) || defined(__SAME54__)
    disableInterrupts(SERCOM_I2CS_INTENCLR_DRDY_Msk |
                      SERCOM_I2CS_INTENCLR_PREC_Msk);
#else
    disableInterrupts(SERCOM_I2CS_INTENCLR_DRDY | SERCOM_I2CS_INTENCLR_PREC);
#endif // __SAME53__ / __SAME54__
    const bool receivePending = _wireReceivePending;
    const bool requestPending = _wireRequestPending;
    const bool followupPending = receivePending || requestPending;
    SercomWireError completionError = error;
    const bool slaveTransmit =
        txn && (txn->config & I2C_CFG_READ) != 0;
    if (slaveTransmit &&
        (error == SercomWireError::SUCCESS ||
         error == SercomWireError::NACK_ON_DATA)) {
      if (!stopDetected)
        prepareSlaveCommandBitsWIRE(WIRE_SLAVE_ACT_COMPLETE);
      // A master NACK is the expected physical end of a slave response. Keep
      // NACK_ON_DATA as the stop/release reason, but publish successful request
      // completion after the slave has released the bus.
      if (error == SercomWireError::NACK_ON_DATA)
        completionError = SercomWireError::SUCCESS;
    }

    if (stopDetected) {
#if defined(__SAME53__) || defined(__SAME54__)
      clearINTFLAG(SERCOM_I2CS_INTFLAG_PREC_Msk);
#else
      clearINTFLAG(SERCOM_I2CS_INTFLAG_PREC);
#endif // __SAME53__ / __SAME54__
    }
    _wire.releasePending = false;

    if (txn) {
#ifdef USE_ZERODMA
      dmaAbortTx();
      dmaAbortRx();
#endif // USE_ZERODMA
      retireSlaveTransactionWIRE(followupPending);
      refreshCompletionReport();
      if (txn->onComplete)
        txn->onComplete(txn->user, static_cast<int>(completionError));
    }

    if (receivePending) {
      _wireReceivePending = false;
      if (_wireReceiveCb)
        _wireReceiveCb(_wireCallbackUser);
    }

    if (requestPending) {
      _wireRequestPending = false;
      if (_wireRequestCb)
        _wireRequestCb(_wireCallbackUser);
    }

    if (_wire.currentTxn == nullptr && _wire.slaveTxn != nullptr)
      setTxnWIRE(_wire.slaveTxn);

    startNextQueuedWIRE();
    return _wire.currentTxn;
  }

  // Everything below is master-only: retry, bus-state recovery, STOP wait,
  // queue retirement, transaction chaining, and return to slave mode.
  constexpr uint8_t kMaxWireRetries = 3;

  if (error == SercomWireError::BUS_STATE_UNKNOWN) {
    if (_wire.retryCount < kMaxWireRetries) {
      ++_wire.retryCount;

#if defined(__SAME53__) || defined(__SAME54__)
      sercom->I2CM.SERCOM_STATUS =
          (sercom->I2CM.SERCOM_STATUS & ~SERCOM_I2CM_STATUS_BUSSTATE_Msk) |
          SERCOM_I2CM_STATUS_BUSSTATE(WIRE_IDLE_STATE);
#else
      sercom->I2CM.STATUS.bit.BUSSTATE = 1;
#endif // __SAME53__ / __SAME54__
      waitSyncBusySysOp();
      startTransmissionWIRE();

      return txn;
    }
  }

  if (error == SercomWireError::BUS_STATE_UNKNOWN)
    _wire.retryCount = 0;

  waitSyncBusySysOp();

  SercomWireError completionError = error;

  // Undocumented HW limitation: DMA transfers must terminate with STOP and bus release.
  // After a DMA write, the host holds the bus ~7.33 us before the next transfer (Sr window).
  // Writing ADDR during that window leaves the bus in an undefined state and breaks
  // subsequent DMA/non-DMA transactions. To avoid this, we must wait for BUSSTATE
  // to return to IDLE after a STOP returning the hardware to a known state.
  // At the tested 48 MHz, this busy-wait is ~350 cycles corresponding to a 3.5 us delay
  // at 100 MHz. This wait must occur BEFORE the callback to ensure the bus is stable
  // before user code can enqueue the next transaction.
  if (txn && (txn->config & I2C_CFG_STOP) &&
      error == SercomWireError::SUCCESS) {
    const uint32_t stopSettleStart = micros();
    const auto busIsIdle = [this]() {
#if defined(__SAME53__) || defined(__SAME54__)
      return ((sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >>
              SERCOM_I2CM_STATUS_BUSSTATE_Pos) <= WIRE_IDLE_STATE;
#else
      return sercom->I2CM.STATUS.bit.BUSSTATE <= WIRE_IDLE_STATE;
#endif // __SAME53__ / __SAME54__
    };

    while (!busIsIdle() && static_cast<uint32_t>(micros() - stopSettleStart) <
                               kWireStopSettleTimeoutUs)
      ;

    if (!busIsIdle()) {
      completionError = SercomWireError::BUS_RELEASE_TIMEOUT;
      // The device datasheet identifies CTRLA.SWRST as recovery for an I2C
      // protocol hang. Preserve the cached Wire configuration and transaction
      // queue, reset only the peripheral, then restore a known master state.
      resetSERCOM();
      setMasterWIRE();
    }
  }
  refreshCompletionReport();
  // Preserve the tested async-DMA master retirement order: publish the
  // callback before inspecting chainNext, dequeuing, or restoring slave mode.
  if (txn && txn->onComplete)
    txn->onComplete(txn->user, static_cast<int>(completionError));

  // Allow multi-phase I2C transactions to chain without dequeuing.
  if (txn && completionError == SercomWireError::SUCCESS && txn->chainNext) {
    txn->chainNext = false; // reset for next iteration
    startTransmissionWIRE();
    return txn;
  }
  if (txn)
    txn->chainNext = false;

  SercomTxn* headTxn = nullptr;
  if (_txnQueue.peek(headTxn) && headTxn == txn)
    _txnQueue.read(txn); // remove completed master queue head

  _wire.retryCount = 0;
  _wire.active = false;
  _wire.currentTxn = nullptr;
  _wire.busErrorRecoveryPending = false;
  _wire.busErrorRecoveryDeadlineActive = false;

  SercomTxn *next = nullptr;
  if (_txnQueue.peek(next))
    startTransmissionWIRE();
  else {
#if defined(__SAME53__) || defined(__SAME54__)
    sercom->I2CM.SERCOM_INTENCLR = SERCOM_I2CM_INTENCLR_ERROR_Msk |
                                   SERCOM_I2CM_INTENCLR_MB_Msk    |
                                   SERCOM_I2CM_INTENCLR_SB_Msk;
#else
    sercom->I2CM.INTENCLR.reg = SERCOM_I2CM_INTENCLR_ERROR |
                                SERCOM_I2CM_INTENCLR_MB    |
                                SERCOM_I2CM_INTENCLR_SB;
#endif // __SAME53__ / __SAME54__

    if (_wire.slaveConfigured) {
      setSlaveWIRE();
      setTxnWIRE(_wire.slaveTxn);
    }
  }

  return txn;
}

// Hardware metadata structure for SERCOM peripherals - private to this file
struct SercomData {
#if defined(__SAME53__) || defined(__SAME54__)
  sercom_registers_t *sercomPtr;
#else
  Sercom   *sercomPtr;
#endif // __SAME53__ / __SAME54__
#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) ||       \
    defined(__SAME54__)
  uint8_t   id_core;
  uint8_t   id_slow;
  IRQn_Type irq[4];
#else
  uint8_t   clock;
  IRQn_Type irqn;
#endif // __SAMD51__ / __SAME51__ || __SAME53__ / __SAME54__
  uint8_t   dmaTxTrigger;
  uint8_t   dmaRxTrigger;
  void     *dataReg;  // Pointer to DATA register
};

static const SercomData sercomData[] = {
#if defined(__SAMD51__) || defined(__SAME51__)
    {SERCOM0, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW, SERCOM0_0_IRQn,
     SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_3_IRQn, SERCOM0_DMAC_ID_TX,
     SERCOM0_DMAC_ID_RX, (void *)&SERCOM0->I2CM.DATA.reg},
    {SERCOM1, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW, SERCOM1_0_IRQn,
     SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_3_IRQn, SERCOM1_DMAC_ID_TX,
     SERCOM1_DMAC_ID_RX, (void *)&SERCOM1->I2CM.DATA.reg},
    {SERCOM2, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW, SERCOM2_0_IRQn,
     SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_3_IRQn, SERCOM2_DMAC_ID_TX,
     SERCOM2_DMAC_ID_RX, (void *)&SERCOM2->I2CM.DATA.reg},
    {SERCOM3, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW, SERCOM3_0_IRQn,
     SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_3_IRQn, SERCOM3_DMAC_ID_TX,
     SERCOM3_DMAC_ID_RX, (void *)&SERCOM3->I2CM.DATA.reg},
    {SERCOM4, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW, SERCOM4_0_IRQn,
     SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_3_IRQn, SERCOM4_DMAC_ID_TX,
     SERCOM4_DMAC_ID_RX, (void *)&SERCOM4->I2CM.DATA.reg},
    {SERCOM5, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW, SERCOM5_0_IRQn,
     SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_3_IRQn, SERCOM5_DMAC_ID_TX,
     SERCOM5_DMAC_ID_RX, (void *)&SERCOM5->I2CM.DATA.reg},
#if defined(SERCOM6)
    {SERCOM6, SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW, SERCOM6_0_IRQn,
     SERCOM6_1_IRQn, SERCOM6_2_IRQn, SERCOM6_3_IRQn, SERCOM6_DMAC_ID_TX,
     SERCOM6_DMAC_ID_RX, (void *)&SERCOM6->I2CM.DATA.reg},
#endif // SERCOM6
#if defined(SERCOM7)
    {SERCOM7, SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW, SERCOM7_0_IRQn,
     SERCOM7_1_IRQn, SERCOM7_2_IRQn, SERCOM7_3_IRQn, SERCOM7_DMAC_ID_TX,
     SERCOM7_DMAC_ID_RX, (void *)&SERCOM7->I2CM.DATA.reg},
#endif // SERCOM7
#elif defined(__SAME53__) || defined(__SAME54__)
    {SERCOM0_REGS, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW, SERCOM0_0_IRQn,
     SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_OTHER_IRQn, SERCOM0_DMAC_ID_TX,
     SERCOM0_DMAC_ID_RX, (void *)&SERCOM0_REGS->I2CM.SERCOM_DATA},
    {SERCOM1_REGS, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW, SERCOM1_0_IRQn,
     SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_OTHER_IRQn, SERCOM1_DMAC_ID_TX,
     SERCOM1_DMAC_ID_RX, (void *)&SERCOM1_REGS->I2CM.SERCOM_DATA},
    {SERCOM2_REGS, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW, SERCOM2_0_IRQn,
     SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_OTHER_IRQn, SERCOM2_DMAC_ID_TX,
     SERCOM2_DMAC_ID_RX, (void *)&SERCOM2_REGS->I2CM.SERCOM_DATA},
    {SERCOM3_REGS, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW, SERCOM3_0_IRQn,
     SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_OTHER_IRQn, SERCOM3_DMAC_ID_TX,
     SERCOM3_DMAC_ID_RX, (void *)&SERCOM3_REGS->I2CM.SERCOM_DATA},
    {SERCOM4_REGS, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW, SERCOM4_0_IRQn,
     SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_OTHER_IRQn, SERCOM4_DMAC_ID_TX,
     SERCOM4_DMAC_ID_RX, (void *)&SERCOM4_REGS->I2CM.SERCOM_DATA},
    {SERCOM5_REGS, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW, SERCOM5_0_IRQn,
     SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_OTHER_IRQn, SERCOM5_DMAC_ID_TX,
     SERCOM5_DMAC_ID_RX, (void *)&SERCOM5_REGS->I2CM.SERCOM_DATA},
#if defined(SERCOM6_REGS)
    {SERCOM6_REGS, SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW, SERCOM6_0_IRQn,
     SERCOM6_1_IRQn, SERCOM6_2_IRQn, SERCOM6_OTHER_IRQn, SERCOM6_DMAC_ID_TX,
     SERCOM6_DMAC_ID_RX, (void *)&SERCOM6_REGS->I2CM.SERCOM_DATA},
#endif // SERCOM6_REGS
#if defined(SERCOM7_REGS)
    {SERCOM7_REGS, SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW, SERCOM7_0_IRQn,
     SERCOM7_1_IRQn, SERCOM7_2_IRQn, SERCOM7_OTHER_IRQn, SERCOM7_DMAC_ID_TX,
     SERCOM7_DMAC_ID_RX, (void *)&SERCOM7_REGS->I2CM.SERCOM_DATA},
#endif // SERCOM7_REGS
#else  // SAMD21
    // SAMD21 has unified clock and single interrupt
    {SERCOM0, GCM_SERCOM0_CORE, SERCOM0_IRQn, SERCOM0_DMAC_ID_TX,
     SERCOM0_DMAC_ID_RX, (void *)&SERCOM0->I2CM.DATA.reg},
    {SERCOM1, GCM_SERCOM1_CORE, SERCOM1_IRQn, SERCOM1_DMAC_ID_TX,
     SERCOM1_DMAC_ID_RX, (void *)&SERCOM1->I2CM.DATA.reg},
    {SERCOM2, GCM_SERCOM2_CORE, SERCOM2_IRQn, SERCOM2_DMAC_ID_TX,
     SERCOM2_DMAC_ID_RX, (void *)&SERCOM2->I2CM.DATA.reg},
    {SERCOM3, GCM_SERCOM3_CORE, SERCOM3_IRQn, SERCOM3_DMAC_ID_TX,
     SERCOM3_DMAC_ID_RX, (void *)&SERCOM3->I2CM.DATA.reg},
#if defined(SERCOM4)
    {SERCOM4, GCM_SERCOM4_CORE, SERCOM4_IRQn, SERCOM4_DMAC_ID_TX,
     SERCOM4_DMAC_ID_RX, (void *)&SERCOM4->I2CM.DATA.reg},
#endif // SERCOM4
#if defined(SERCOM5)
    {SERCOM5, GCM_SERCOM5_CORE, SERCOM5_IRQn, SERCOM5_DMAC_ID_TX,
     SERCOM5_DMAC_ID_RX, (void *)&SERCOM5->I2CM.DATA.reg},
#endif // SERCOM5
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__ / SAMD21
};

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
  PendSV::instance().clearService(PendSVChannels::sercom(sercomId));
#ifdef SERCOM_STRICT_PADS
  clearPads(sercomId);
#endif // SERCOM_STRICT_PADS
}

bool SERCOM::registerService(uint8_t sercomId, ServiceFn fn)
{
  if (sercomId >= kSercomCount)
    return false;

  s_states[sercomId].service = fn;
  if (!PendSV::instance().registerService(PendSVChannels::sercom(sercomId),
                                          &SERCOM::dispatchService, nullptr))
  {
    s_states[sercomId].service = nullptr;
    return false;
  }

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
#endif // SERCOM0_DMAC_ID_TX
  {
    // Fallback: calculate triggers if table lookup unavailable
    _dmaTxTrigger = SERCOM0_DMAC_ID_TX + (sercomId * 2);
    _dmaRxTrigger = SERCOM0_DMAC_ID_RX + (sercomId * 2);
  }

  // DATA register is at the same offset (0x28) for all protocols (I2C, SPI, UART).
  // Access via any union member is transparent—just use I2CM as the canonical reference.
#if defined(__SAME53__) || defined(__SAME54__)
  void* dataReg = (void*)&sercom->I2CM.SERCOM_DATA;
#else
  void* dataReg = (void*)&sercom->I2CM.DATA.reg;
#endif // __SAME53__ / __SAME54__

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
  if (_dmaErrorCb) {
    _dmaTx->setCallback(_dmaErrorCb, DMA_CALLBACK_TRANSFER_ERROR);
    _dmaRx->setCallback(_dmaErrorCb, DMA_CALLBACK_TRANSFER_ERROR);
  }

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

void SERCOM::dmaSetCallbacks(DmaCallback txCb, DmaCallback rxCb,
                             DmaCallback errorCb) {
  _dmaTxCb = txCb;
  _dmaRxCb = rxCb;
  _dmaErrorCb = errorCb;

  if (_dmaConfigured)
  {
    if (_dmaTxCb)
      _dmaTx->setCallback(_dmaTxCb);
    if (_dmaRxCb)
      _dmaRx->setCallback(_dmaRxCb);
    _dmaTx->setCallback(_dmaErrorCb, DMA_CALLBACK_TRANSFER_ERROR);
    _dmaRx->setCallback(_dmaErrorCb, DMA_CALLBACK_TRANSFER_ERROR);
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
#endif // USE_ZERODMA

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

  PendSV::instance().setPending(PendSVChannels::sercom(sercomId));
}

void SERCOM::dispatchService(uint8_t sercomId, void *context)
{
  (void)context;

  sercomId = PendSVChannels::sercomIndex(sercomId);

  if (sercomId >= kSercomCount)
    return;

  ServiceFn fn = s_states[sercomId].service;
  SERCOM* inst = s_instances[sercomId];
  if (fn && inst) {
    (inst->*fn)();
  }
}

void SERCOM::dispatchPending(void)
{
  for (uint8_t i = 0; i < kSercomCount; ++i)
    dispatchService(i, nullptr);
}

int8_t SERCOM::getSercomIndex(void) {
  for(uint8_t i=0; i<(sizeof(sercomData) / sizeof(sercomData[0])); i++) {
    if(sercom == sercomData[i].sercomPtr) return i;
  }
  return -1;
}

uint32_t SERCOM::getSercomFreqRef(void)
{
#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) ||       \
    defined(__SAME54__)
  int8_t idx = getSercomIndex();
  uint8_t gen = 1; // default to GCLK1 (48 MHz) if we can't resolve

  if (idx >= 0)
  {
    uint8_t pch = sercomData[idx].id_core;
#if defined(__SAME53__) || defined(__SAME54__)
    gen = (GCLK_REGS->GCLK_PCHCTRL[pch] & GCLK_PCHCTRL_GEN_Msk) >>
          GCLK_PCHCTRL_GEN_Pos;
#else
    gen = GCLK->PCHCTRL[pch].bit.GEN;
#endif // __SAME53__ / __SAME54__
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
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__

  return freqRef;
}

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) ||       \
    defined(__SAME54__)
// This is currently for overriding an SPI SERCOM's clock source only --
// NOT for UART or WIRE SERCOMs, where it will have unintended consequences.
// It does not check.
// SERCOM clock source override is available only on SAMD51 (not 21).
// A dummy function for SAMD21 (compiles to nothing) is present in SERCOM.h
// so user code doesn't require a lot of conditional situations.
void SERCOM::setClockSource(int8_t idx, SercomClockSource src, bool core) {

  if(src == SERCOM_CLOCK_SOURCE_NO_CHANGE) return;

  uint8_t clk_id = core ? sercomData[idx].id_core : sercomData[idx].id_slow;

#if defined(__SAME53__) || defined(__SAME54__)
  GCLK_REGS->GCLK_PCHCTRL[clk_id] &= ~GCLK_PCHCTRL_CHEN_Msk;
  while (GCLK_REGS->GCLK_PCHCTRL[clk_id] & GCLK_PCHCTRL_CHEN_Msk);
#else
  GCLK->PCHCTRL[clk_id].bit.CHEN = 0;     // Disable timer
  while(GCLK->PCHCTRL[clk_id].bit.CHEN);  // Wait for disable
#endif // __SAME53__ / __SAME54__

  if(core) clockSource = src; // Save SercomClockSource value

  // From cores/arduino/startup.c:
  // GCLK0 = F_CPU (this is 120 MHz and exceeds SERCOM maximum)
  // GCLK1 = 48 MHz
  // GCLK2 = 100 MHz
  // GCLK3 = XOSC32K
  // GCLK4 = 12 MHz
  if(src == SERCOM_CLOCK_SOURCE_FCPU) {
#if defined(__SAME53__) || defined(__SAME54__)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK2 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
        GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); // Guard Sercom from exceeding 100 MHz maximum
#endif // __SAME53__ / __SAME54__
    if (core)
      freqRef = 100000000; // Save clock frequency value
  }
  else if (src == SERCOM_CLOCK_SOURCE_48M)
  {
#if defined(__SAME53__) || defined(__SAME54__)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK1 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif // __SAME53__ / __SAME54__
    if(core) freqRef = 48000000;
  } else if(src == SERCOM_CLOCK_SOURCE_100M) {
#if defined(__SAME53__) || defined(__SAME54__)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK2 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif // __SAME53__ / __SAME54__
    if(core) freqRef = 100000000;
  } else if(src == SERCOM_CLOCK_SOURCE_32K) {
#if defined(__SAME53__) || defined(__SAME54__)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK3 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif // __SAME53__ / __SAME54__
    if(core) freqRef = 32768;
  } else if(src == SERCOM_CLOCK_SOURCE_12M) {
#if defined(__SAME53__) || defined(__SAME54__)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK4 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif // __SAME53__ / __SAME54__
    if(core) freqRef = 12000000;
  }

#if defined(__SAME53__) || defined(__SAME54__)
  while ((GCLK_REGS->GCLK_PCHCTRL[clk_id] & GCLK_PCHCTRL_CHEN_Msk) == 0);
#else
  while(!GCLK->PCHCTRL[clk_id].bit.CHEN); // Wait for clock enable
#endif // __SAME53__ / __SAME54__
}
#endif // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__

void SERCOM::initClockNVIC( void )
{
  int8_t idx = getSercomIndex();
  if(idx < 0) return; // We got a problem here

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) ||       \
    defined(__SAME54__)

  for(uint8_t i=0; i<4; i++) {
    NVIC_ClearPendingIRQ(sercomData[idx].irq[i]);
    NVIC_SetPriority(sercomData[idx].irq[i], SERCOM_NVIC_PRIORITY);
    NVIC_EnableIRQ(sercomData[idx].irq[i]);
  }

  setClockSource(idx, clockSource, true);              // true  = core clock
  setClockSource(idx, SERCOM_CLOCK_SOURCE_32K, false); // false = slow clock

#else // __SAMD51__ / __SAME51__ / __SAME53__ / __SAME54__

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

#endif // __SAMD51__ / __SAME51__ || __SAME53__ / __SAME54__

  getSercomFreqRef();
}
