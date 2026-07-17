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

#ifndef WIRE_RISE_TIME_NANOSECONDS
// Default rise time in nanoseconds, based on 4.7K ohm pull up resistors
// you can override this value in your variant if needed
#define WIRE_RISE_TIME_NANOSECONDS 125
#endif

#if defined(ARDUINO_SAMD51_E51)
SERCOM::SERCOM(Sercom* s)
#elif defined(ARDUINO_SAME53_E54)
SERCOM::SERCOM(sercom_registers_t* s)
#else
SERCOM::SERCOM(Sercom* s)
#endif
{
  sercom = s;

#if defined(ARDUINO_SAMD51_E51) || defined(ARDUINO_SAME53_E54)
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
#endif // SAME5x
}

/* =========================
 * ===== Sercom UART
 * =========================
*/
void SERCOM::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{
  initClockNVIC();
  resetUART();

#if defined(ARDUINO_SAMD51_E51)
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |
                            SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
                               SERCOM_USART_INTENSET_ERROR; //All others errors
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_MODE(mode) |
                                      SERCOM_USART_INT_CTRLA_SAMPR(sampleRate);
  sercom->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_RXC_Msk |
                                         SERCOM_USART_INT_INTENSET_ERROR_Msk;
#else
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |
                            SERCOM_USART_CTRLA_SAMPR(sampleRate);

  //Setting the Interrupt register
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
                               SERCOM_USART_INTENSET_ERROR; //All others errors
#endif

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

#if defined(ARDUINO_SAMD51_E51)
    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
#elif defined(ARDUINO_SAME53_E54)
    sercom->USART_INT.SERCOM_BAUD =
        SERCOM_USART_INT_BAUD_FRAC_FP(baudTimes8 % 8) |
        SERCOM_USART_INT_BAUD_FRAC_BAUD(baudTimes8 / 8);
#else
    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
#endif
  }
}
void SERCOM::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
#if defined(ARDUINO_SAMD51_E51)
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |=
    SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
    dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(charSize) |
    nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
    (parityMode == SERCOM_NO_PARITY ? 0 : parityMode) <<
      SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_CTRLA |=
      SERCOM_USART_INT_CTRLA_FORM(parityMode == SERCOM_NO_PARITY ? 0 : 1) |
      SERCOM_USART_INT_CTRLA_DORD(dataOrder);
  sercom->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_CHSIZE(charSize) |
                                       SERCOM_USART_INT_CTRLB_SBMODE(nbStopBits) |
                                       SERCOM_USART_INT_CTRLB_PMODE(
                                           parityMode == SERCOM_NO_PARITY ? 0 : parityMode);
#else
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |=
    SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1) ) |
    dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

  //Setting the CTRLB register
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(charSize) |
    nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |
    (parityMode == SERCOM_NO_PARITY ? 0 : parityMode) <<
      SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
#endif
}

void SERCOM::initPads(SercomUartTXPad txPad, SercomRXPad rxPad)
{
#if defined(ARDUINO_SAMD51_E51)
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |
                             SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_TXPO(txPad) |
                                       SERCOM_USART_INT_CTRLA_RXPO(rxPad);
  sercom->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_TXEN_Msk |
                                       SERCOM_USART_INT_CTRLB_RXEN_Msk;
#else
  //Setting the CTRLA register
  sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |
                             SERCOM_USART_CTRLA_RXPO(rxPad);

  // Enable Transceiver and Receiver
  sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
#endif
}

void SERCOM::resetUART()
{
#if defined(ARDUINO_SAMD51_E51)
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
  while ((sercom->USART_INT.SERCOM_CTRLA & SERCOM_USART_INT_CTRLA_SWRST_Msk) ||
         (sercom->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_SWRST_Msk)) {
  }
#else
  // Start the Software Reset
  sercom->USART.CTRLA.bit.SWRST = 1 ;

  while ( sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }
#endif
}

void SERCOM::enableUART()
{
#if defined(ARDUINO_SAMD51_E51)
  //Setting  the enable bit to 1
  sercom->USART.CTRLA.bit.ENABLE = 0x1u;

  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(sercom->USART.SYNCBUSY.bit.ENABLE);
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
  while (sercom->USART_INT.SERCOM_SYNCBUSY & SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk) {
  }
#else
  //Setting  the enable bit to 1
  sercom->USART.CTRLA.bit.ENABLE = 0x1u;

  //Wait for then enable bit from SYNCBUSY is equal to 0;
  while(sercom->USART.SYNCBUSY.bit.ENABLE);
#endif
}

void SERCOM::flushUART()
{
  // Skip checking transmission completion if data register is empty
  if(isDataRegisterEmptyUART())
    return;

  // Wait for transmission to complete
#if defined(ARDUINO_SAMD51_E51)
  while(!sercom->USART.INTFLAG.bit.TXC);
#elif defined(ARDUINO_SAME53_E54)
  while ((sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk) == 0) {
  }
#else
  while(!sercom->USART.INTFLAG.bit.TXC);
#endif
}

void SERCOM::clearStatusUART()
{
#if defined(ARDUINO_SAMD51_E51)
  //Reset (with 0) the STATUS register
  sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_STATUS = SERCOM_USART_INT_STATUS_RESETVALUE;
#else
  //Reset (with 0) the STATUS register
  sercom->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
#endif
}

bool SERCOM::availableDataUART()
{
  //RXC : Receive Complete
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.INTFLAG.bit.RXC;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.RXC;
#endif
}

bool SERCOM::isUARTError()
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.INTFLAG.bit.ERROR;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_ERROR_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.ERROR;
#endif
}

void SERCOM::acknowledgeUARTError()
{
#if defined(ARDUINO_SAMD51_E51)
  sercom->USART.INTFLAG.bit.ERROR = 1;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_INTFLAG = SERCOM_USART_INT_INTFLAG_ERROR_Msk;
#else
  sercom->USART.INTFLAG.bit.ERROR = 1;
#endif
}

bool SERCOM::isBufferOverflowErrorUART()
{
  //BUFOVF : Buffer Overflow
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.STATUS.bit.BUFOVF;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_BUFOVF_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.BUFOVF;
#endif
}

bool SERCOM::isFrameErrorUART()
{
  //FERR : Frame Error
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.STATUS.bit.FERR;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_FERR_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.FERR;
#endif
}

void SERCOM::clearFrameErrorUART()
{
  // clear FERR bit writing 1 status bit
#if defined(ARDUINO_SAMD51_E51)
  sercom->USART.STATUS.bit.FERR = 1;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_STATUS = SERCOM_USART_INT_STATUS_FERR_Msk;
#else
  sercom->USART.STATUS.bit.FERR = 1;
#endif
}

bool SERCOM::isParityErrorUART()
{
  //PERR : Parity Error
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.STATUS.bit.PERR;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_STATUS & SERCOM_USART_INT_STATUS_PERR_Msk) != 0;
#else
  return sercom->USART.STATUS.bit.PERR;
#endif
}

bool SERCOM::isDataRegisterEmptyUART()
{
  //DRE : Data Register Empty
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.INTFLAG.bit.DRE;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk) != 0;
#else
  return sercom->USART.INTFLAG.bit.DRE;
#endif
}

uint8_t SERCOM::readDataUART()
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->USART.DATA.bit.DATA;
#elif defined(ARDUINO_SAME53_E54)
  return (uint8_t)sercom->USART_INT.SERCOM_DATA;
#else
  return sercom->USART.DATA.bit.DATA;
#endif
}

int SERCOM::writeDataUART(uint8_t data)
{
  // Wait for data register to be empty
  while(!isDataRegisterEmptyUART());

  //Put data into DATA register
#if defined(ARDUINO_SAMD51_E51)
  sercom->USART.DATA.reg = (uint16_t)data;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_DATA = data;
#else
  sercom->USART.DATA.reg = (uint16_t)data;
#endif
  return 1;
}

void SERCOM::enableDataRegisterEmptyInterruptUART()
{
#if defined(ARDUINO_SAMD51_E51)
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_INTENSET = SERCOM_USART_INT_INTENSET_DRE_Msk;
#else
  sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
#endif
}

void SERCOM::disableDataRegisterEmptyInterruptUART()
{
#if defined(ARDUINO_SAMD51_E51)
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
#elif defined(ARDUINO_SAME53_E54)
  sercom->USART_INT.SERCOM_INTENCLR = SERCOM_USART_INT_INTENCLR_DRE_Msk;
#else
  sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
#endif
}

/* =========================
 * ===== Sercom SPI
 * =========================
*/
void SERCOM::initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder)
{
  resetSPI();
  initClockNVIC();

#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3) | // master mode
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA = SERCOM_SPIM_CTRLA_MODE_SPI_MASTER |
                               SERCOM_SPIM_CTRLA_DOPO(mosi) |
                               SERCOM_SPIM_CTRLA_DIPO(miso) |
                               SERCOM_SPIM_CTRLA_DORD(dataOrder);
#else
  //Setting the CTRLA register
  sercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
                          SERCOM_SPI_CTRLA_DOPO(mosi) |
                          SERCOM_SPI_CTRLA_DIPO(miso) |
                          dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;
#endif

  //Setting the CTRLB register
#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

  while( sercom->SPI.SYNCBUSY.bit.CTRLB == 1 );
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLB = SERCOM_SPIM_CTRLB_CHSIZE(charSize) |
                               SERCOM_SPIM_CTRLB_RXEN_Msk;
  while (sercom->SPIM.SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
#else
  sercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |
                          SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

  while( sercom->SPI.SYNCBUSY.bit.CTRLB == 1 );
#endif
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
#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.CTRLA.reg |= ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                           ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_CPHA(cpha) |
                                SERCOM_SPIM_CTRLA_CPOL(cpol);
  sercom->SPIM.SERCOM_BAUD = SERCOM_SPIM_BAUD_BAUD(calculateBaudrateSynchronous(baudrate));
#else
  sercom->SPI.CTRLA.reg |= ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |
                           ( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );

  //Synchronous arithmetic
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(baudrate);
#endif
}

void SERCOM::resetSPI()
{
#if defined(ARDUINO_SAMD51_E51)
  //Setting the Software Reset bit to 1
  sercom->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_SWRST_Msk;
  while ((sercom->SPIM.SERCOM_CTRLA & SERCOM_SPIM_CTRLA_SWRST_Msk) ||
         (sercom->SPIM.SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_SWRST_Msk));
#else
  //Setting the Software Reset bit to 1
  sercom->SPI.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
#endif
}

void SERCOM::enableSPI()
{
#if defined(ARDUINO_SAMD51_E51)
  //Setting the enable bit to 1
  sercom->SPI.CTRLA.bit.ENABLE = 1;

  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA |= SERCOM_SPIM_CTRLA_ENABLE_Msk;
  while (sercom->SPIM.SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
#else
  //Setting the enable bit to 1
  sercom->SPI.CTRLA.bit.ENABLE = 1;

  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }
#endif
}

void SERCOM::disableSPI()
{
#if defined(ARDUINO_SAMD51_E51)
  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  sercom->SPI.CTRLA.bit.ENABLE = 0;
#elif defined(ARDUINO_SAME53_E54)
  while (sercom->SPIM.SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);
  sercom->SPIM.SERCOM_CTRLA &= ~SERCOM_SPIM_CTRLA_ENABLE_Msk;
#else
  while(sercom->SPI.SYNCBUSY.bit.ENABLE)
  {
    //Waiting then enable bit from SYNCBUSY is equal to 0;
  }

  //Setting the enable bit to 0
  sercom->SPI.CTRLA.bit.ENABLE = 0;
#endif
}

void SERCOM::setDataOrderSPI(SercomDataOrder dataOrder)
{
  //Register enable-protected
  disableSPI();

#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.CTRLA.bit.DORD = dataOrder;
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA =
      (sercom->SPIM.SERCOM_CTRLA & ~SERCOM_SPIM_CTRLA_DORD_Msk) |
      SERCOM_SPIM_CTRLA_DORD(dataOrder);
#else
  sercom->SPI.CTRLA.bit.DORD = dataOrder;
#endif

  enableSPI();
}

SercomDataOrder SERCOM::getDataOrderSPI()
{
#if defined(ARDUINO_SAMD51_E51)
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->SPIM.SERCOM_CTRLA & SERCOM_SPIM_CTRLA_DORD_Msk) ? LSB_FIRST : MSB_FIRST;
#else
  return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
#endif
}

void SERCOM::setBaudrateSPI(uint8_t divider)
{
  disableSPI(); // Register is enable-protected
#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(freqRef / divider);
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_BAUD = SERCOM_SPIM_BAUD_BAUD(calculateBaudrateSynchronous(freqRef / divider));
#else
  sercom->SPI.BAUD.reg = calculateBaudrateSynchronous(freqRef / divider);
#endif
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

#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.CTRLA.bit.CPOL = cpol;
  sercom->SPI.CTRLA.bit.CPHA = cpha;
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_CTRLA =
      (sercom->SPIM.SERCOM_CTRLA & ~(SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) |
      SERCOM_SPIM_CTRLA_CPOL(cpol) | SERCOM_SPIM_CTRLA_CPHA(cpha);
#else
  sercom->SPI.CTRLA.bit.CPOL = cpol;
  sercom->SPI.CTRLA.bit.CPHA = cpha;
#endif

  enableSPI();
}

uint8_t SERCOM::transferDataSPI(uint8_t data)
{
#if defined(ARDUINO_SAMD51_E51)
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while(sercom->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception

  return sercom->SPI.DATA.bit.DATA;  // Reading data
#elif defined(ARDUINO_SAME53_E54)
  sercom->SPIM.SERCOM_DATA = data;
  while ((sercom->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_RXC_Msk) == 0);
  return (uint8_t)sercom->SPIM.SERCOM_DATA;
#else
  sercom->SPI.DATA.bit.DATA = data; // Writing data into Data register

  while(sercom->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception

  return sercom->SPI.DATA.bit.DATA;  // Reading data
#endif
}

bool SERCOM::isBufferOverflowErrorSPI()
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->SPI.STATUS.bit.BUFOVF;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->SPIM.SERCOM_STATUS & SERCOM_SPIM_STATUS_BUFOVF_Msk;
#else
  return sercom->SPI.STATUS.bit.BUFOVF;
#endif
}

bool SERCOM::isDataRegisterEmptySPI()
{
  //DRE : Data Register Empty
#if defined(ARDUINO_SAMD51_E51)
  return sercom->SPI.INTFLAG.bit.DRE;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk;
#else
  return sercom->SPI.INTFLAG.bit.DRE;
#endif
}

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

#if defined(ARDUINO_SAMD51_E51)
  //Setting the Software bit to 1
  sercom->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_SWRST_Msk;
  while ((sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SWRST_Msk) ||
         (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SWRST_Msk));
#else
  //Setting the Software bit to 1
  sercom->I2CM.CTRLA.bit.SWRST = 1;

  //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
  while(sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);
#endif
}

void SERCOM::enableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

#if defined(ARDUINO_SAMD51_E51)
  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }

  // Setting bus idle mode
  sercom->I2CM.STATUS.bit.BUSSTATE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY coming back to 0
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE_Msk;
  while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);

  sercom->I2CM.SERCOM_STATUS =
      (sercom->I2CM.SERCOM_STATUS & ~SERCOM_I2CM_STATUS_BUSSTATE_Msk) |
      SERCOM_I2CM_STATUS_BUSSTATE_IDLE;
  while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
#else
  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }

  // Setting bus idle mode
  sercom->I2CM.STATUS.bit.BUSSTATE = 1 ;

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY coming back to 0
  }
#endif
}

void SERCOM::disableWIRE()
{
  // I2C Master and Slave modes share the ENABLE bit function.

#if defined(ARDUINO_SAMD51_E51)
  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk;
  while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk);
#else
  // Enable the I2C master mode
  sercom->I2CM.CTRLA.bit.ENABLE = 0 ;

  while ( sercom->I2CM.SYNCBUSY.bit.ENABLE != 0 )
  {
    // Waiting the enable bit from SYNCBUSY is equal to 0;
  }
#endif
}

void SERCOM::initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall )
{
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;
  resetWIRE() ;

  // Set slave mode
#if defined(ARDUINO_SAMD51_E51)
  sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;

  sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( ucAddress & 0x7Ful ) | // 0x7F, select only 7 bits
                          SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul );          // 0x00, only match exact address
  if (enableGeneralCall) {
    sercom->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_GENCEN;                   // enable general call (address 0x00)
  }

  // Set the interrupt register
  sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
                              SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                              SERCOM_I2CS_INTENSET_DRDY ;   // Data Ready

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CS.SERCOM_CTRLA = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
  sercom->I2CS.SERCOM_ADDR = SERCOM_I2CS_ADDR_ADDR(ucAddress & 0x7Ful) |
                             SERCOM_I2CS_ADDR_ADDRMASK(0x00ul);
  if (enableGeneralCall) {
    sercom->I2CS.SERCOM_ADDR |= SERCOM_I2CS_ADDR_GENCEN_Msk;
  }
  sercom->I2CS.SERCOM_INTENSET = SERCOM_I2CS_INTENSET_PREC_Msk |
                                  SERCOM_I2CS_INTENSET_AMATCH_Msk |
                                  SERCOM_I2CS_INTENSET_DRDY_Msk;
  while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
#else
  sercom->I2CS.CTRLA.bit.MODE = I2C_SLAVE_OPERATION;

  sercom->I2CS.ADDR.reg = SERCOM_I2CS_ADDR_ADDR( ucAddress & 0x7Ful ) | // 0x7F, select only 7 bits
                          SERCOM_I2CS_ADDR_ADDRMASK( 0x00ul );          // 0x00, only match exact address
  if (enableGeneralCall) {
    sercom->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_GENCEN;                   // enable general call (address 0x00)
  }

  // Set the interrupt register
  sercom->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC |   // Stop
                              SERCOM_I2CS_INTENSET_AMATCH | // Address Match
                              SERCOM_I2CS_INTENSET_DRDY ;   // Data Ready

  while ( sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 )
  {
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
  }
#endif
}

void SERCOM::initMasterWIRE( uint32_t baudrate )
{
  // Initialize the peripheral clock and interruption
  initClockNVIC() ;

  resetWIRE() ;

  // Set master mode and enable SCL Clock Stretch mode (stretch after ACK bit)
#if defined(ARDUINO_SAMD51_E51)
  sercom->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_CTRLA = SERCOM_I2CM_CTRLA_MODE_I2C_MASTER;
#else
  sercom->I2CM.CTRLA.reg =  SERCOM_I2CM_CTRLA_MODE( I2C_MASTER_OPERATION )/* |
                            SERCOM_I2CM_CTRLA_SCLSM*/ ;
#endif

  // Enable Smart mode and Quick Command
  //sercom->I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*| SERCOM_I2CM_CTRLB_QCEN*/ ;


  // Enable all interrupts
  // sercom->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

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

#if defined(ARDUINO_SAMD51_E51)
  sercom->I2CM.CTRLA.bit.SPEED = speedBit;
  sercom->I2CM.CTRLA.bit.SCLSM = clockStretchMode;
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_CTRLA =
      (sercom->I2CM.SERCOM_CTRLA &
       ~(SERCOM_I2CM_CTRLA_SPEED_Msk | SERCOM_I2CM_CTRLA_SCLSM_Msk)) |
      SERCOM_I2CM_CTRLA_SPEED(speedBit) |
      SERCOM_I2CM_CTRLA_SCLSM(clockStretchMode);
#else
  sercom->I2CM.CTRLA.bit.SPEED = speedBit;
  sercom->I2CM.CTRLA.bit.SCLSM = clockStretchMode;
#endif

  uint32_t minBaudrate = freqRef / 512; // BAUD = 255: SAMD51(@100MHz) ~195kHz, SAMD21 ~94kHz
  uint32_t maxBaudrate = topSpeeds[speedBit];
  baudrate = max(minBaudrate, min(baudrate, maxBaudrate));

  if (speedBit == 0x2) {
#if defined(ARDUINO_SAMD51_E51)
    sercom->I2CM.BAUD.bit.HSBAUD = freqRef / (2 * baudrate) - 1;
#elif defined(ARDUINO_SAME53_E54)
    sercom->I2CM.SERCOM_BAUD =
        (sercom->I2CM.SERCOM_BAUD & ~SERCOM_I2CM_BAUD_HSBAUD_Msk) |
        SERCOM_I2CM_BAUD_HSBAUD(freqRef / (2 * baudrate) - 1);
#else
    sercom->I2CM.BAUD.bit.HSBAUD = freqRef / (2 * baudrate) - 1;
#endif
  } else {
#if defined(ARDUINO_SAMD51_E51)
    sercom->I2CM.BAUD.bit.BAUD = freqRef / (2 * baudrate) - 5 -
      (freqRef/1000000ul * WIRE_RISE_TIME_NANOSECONDS) / 2000;
#elif defined(ARDUINO_SAME53_E54)
    sercom->I2CM.SERCOM_BAUD =
        (sercom->I2CM.SERCOM_BAUD & ~SERCOM_I2CM_BAUD_BAUD_Msk) |
        SERCOM_I2CM_BAUD_BAUD(freqRef / (2 * baudrate) - 5 -
          (freqRef/1000000ul * WIRE_RISE_TIME_NANOSECONDS) / 2000);
#else
    sercom->I2CM.BAUD.bit.BAUD = freqRef / (2 * baudrate) - 5 -
      (freqRef/1000000ul * WIRE_RISE_TIME_NANOSECONDS) / 2000;
#endif
  }
}

void SERCOM::prepareNackBitWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  if(isMasterWIRE()) {
    // Send a NACK
    sercom->I2CM.CTRLB.bit.ACKACT = 1;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 1;
  }
#elif defined(ARDUINO_SAME53_E54)
  if (isMasterWIRE()) {
    sercom->I2CM.SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_ACKACT_Msk;
  } else {
    sercom->I2CS.SERCOM_CTRLB |= SERCOM_I2CS_CTRLB_ACKACT_Msk;
  }
#else
  if(isMasterWIRE()) {
    // Send a NACK
    sercom->I2CM.CTRLB.bit.ACKACT = 1;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 1;
  }
#endif
}

void SERCOM::prepareAckBitWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  if(isMasterWIRE()) {
    // Send an ACK
    sercom->I2CM.CTRLB.bit.ACKACT = 0;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 0;
  }
#elif defined(ARDUINO_SAME53_E54)
  if (isMasterWIRE()) {
    sercom->I2CM.SERCOM_CTRLB &= ~SERCOM_I2CM_CTRLB_ACKACT_Msk;
  } else {
    sercom->I2CS.SERCOM_CTRLB &= ~SERCOM_I2CS_CTRLB_ACKACT_Msk;
  }
#else
  if(isMasterWIRE()) {
    // Send an ACK
    sercom->I2CM.CTRLB.bit.ACKACT = 0;
  } else {
    sercom->I2CS.CTRLB.bit.ACKACT = 0;
  }
#endif
}

void SERCOM::prepareCommandBitsWire(uint8_t cmd)
{
#if defined(ARDUINO_SAMD51_E51)
  if(isMasterWIRE()) {
    sercom->I2CM.CTRLB.bit.CMD = cmd;

    while(sercom->I2CM.SYNCBUSY.bit.SYSOP)
    {
      // Waiting for synchronization
    }
  } else {
    sercom->I2CS.CTRLB.bit.CMD = cmd;
  }
#elif defined(ARDUINO_SAME53_E54)
  if (isMasterWIRE()) {
    sercom->I2CM.SERCOM_CTRLB =
        (sercom->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
        SERCOM_I2CM_CTRLB_CMD(cmd);
    while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk);
  } else {
    sercom->I2CS.SERCOM_CTRLB =
        (sercom->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_CMD_Msk) |
        SERCOM_I2CS_CTRLB_CMD(cmd);
  }
#else
  if(isMasterWIRE()) {
    sercom->I2CM.CTRLB.bit.CMD = cmd;

    while(sercom->I2CM.SYNCBUSY.bit.SYSOP)
    {
      // Waiting for synchronization
    }
  } else {
    sercom->I2CS.CTRLB.bit.CMD = cmd;
  }
#endif
}

bool SERCOM::startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag)
{
  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;

  // If another master owns the bus or the last bus owner has not properly
  // sent a stop, return failure early. This will prevent some misbehaved
  // devices from deadlocking here at the cost of the caller being responsible
  // for retrying the failed transmission. See SercomWireBusState for the
  // possible bus states.
  if(!isBusOwnerWIRE())
  {
    if( isBusBusyWIRE() || (isArbLostWIRE() && !isBusIdleWIRE()) || isBusUnknownWIRE() )
    {
      return false;
    }
  }

#if defined(ARDUINO_SAMD51_E51)
  // Send start and address
  sercom->I2CM.INTFLAG.bit.ERROR = 1;
  sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_ADDR(address) |
                          ((sercom->I2CM.CTRLA.bit.SPEED == 0x2) ? SERCOM_I2CM_ADDR_HS : 0);

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !sercom->I2CM.INTFLAG.bit.MB ) {
      // Wait transmission complete

      // If certain errors occur, the MB bit may never be set (RFTM: SAMD21 sec:28.10.6; SAMD51 sec:36.10.7).
      // The data transfer errors that can occur (including BUSERR) are all
      // rolled up into INTFLAG.bit.ERROR from STATUS.reg
      if (sercom->I2CM.INTFLAG.bit.ERROR) {
        return false;
      }
    }
  }
  else  // Read mode
  {
    while( !sercom->I2CM.INTFLAG.bit.SB ) {
      // Wait transmission complete

      // If the slave NACKS the address, the MB bit will be set.
      // A variety of errors in the STATUS register can set the ERROR bit in the INTFLAG register
      // In that case, send a stop condition and return false.
      if (sercom->I2CM.INTFLAG.bit.MB || sercom->I2CM.INTFLAG.bit.ERROR) {
        sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        return false;
      }
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
  }

  //ACK received (0: ACK, 1: NACK)
  if(sercom->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_ERROR_Msk;
  const bool highSpeed =
      (sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SPEED_Msk) ==
      SERCOM_I2CM_CTRLA_SPEED_HIGH_SPEED_MODE;
  sercom->I2CM.SERCOM_ADDR = SERCOM_I2CM_ADDR_ADDR(address) |
                              (highSpeed ? SERCOM_I2CM_ADDR_HS_Msk : 0);

  if (flag == WIRE_WRITE_FLAG) {
    while ((sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) == 0) {
      if (sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_ERROR_Msk) {
        return false;
      }
    }
  } else {
    while ((sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_SB_Msk) == 0) {
      if (sercom->I2CM.SERCOM_INTFLAG &
          (SERCOM_I2CM_INTFLAG_MB_Msk | SERCOM_I2CM_INTFLAG_ERROR_Msk)) {
        sercom->I2CM.SERCOM_CTRLB =
            (sercom->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
            SERCOM_I2CM_CTRLB_CMD(3);
        return false;
      }
    }
  }
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) == 0;
#else
  // Send start and address
  sercom->I2CM.INTFLAG.bit.ERROR = 1;
  sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_ADDR(address) | 
                          ((sercom->I2CM.CTRLA.bit.SPEED == 0x2) ? SERCOM_I2CM_ADDR_HS : 0);

  // Address Transmitted
  if ( flag == WIRE_WRITE_FLAG ) // Write mode
  {
    while( !sercom->I2CM.INTFLAG.bit.MB ) {
      // Wait transmission complete

      // If certain errors occur, the MB bit may never be set (RFTM: SAMD21 sec:28.10.6; SAMD51 sec:36.10.7).
      // The data transfer errors that can occur (including BUSERR) are all
      // rolled up into INTFLAG.bit.ERROR from STATUS.reg
      if (sercom->I2CM.INTFLAG.bit.ERROR) {
        return false;
      }
    }
  }
  else  // Read mode
  {
    while( !sercom->I2CM.INTFLAG.bit.SB ) {
      // Wait transmission complete

      // If the slave NACKS the address, the MB bit will be set.
      // A variety of errors in the STATUS register can set the ERROR bit in the INTFLAG register
      // In that case, send a stop condition and return false.
      if (sercom->I2CM.INTFLAG.bit.MB || sercom->I2CM.INTFLAG.bit.ERROR) {
        sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        return false;
      }
    }

    // Clean the 'Slave on Bus' flag, for further usage.
    //sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
  }

  //ACK received (0: ACK, 1: NACK)
  if(sercom->I2CM.STATUS.bit.RXNACK)
  {
    return false;
  }
  else
  {
    return true;
  }
#endif
}

bool SERCOM::sendDataMasterWIRE(uint8_t data)
{
#if defined(ARDUINO_SAMD51_E51)
  //Send data
  sercom->I2CM.INTFLAG.bit.ERROR = 1;
  sercom->I2CM.DATA.bit.DATA = data;

  //Wait transmission successful
  while(!sercom->I2CM.INTFLAG.bit.MB) {
    // If a data transfer error occurs, the MB bit may never be set.
    // Check the error bit and bail if it's set.
    // The data transfer errors that can occur (including BUSERR) are all
    // rolled up into INTFLAG.bit.ERROR from STATUS.reg
    if (sercom->I2CM.INTFLAG.bit.ERROR) {
      return false;
    }
  }

  //Problems on line? nack received?
  if(sercom->I2CM.STATUS.bit.RXNACK)
    return false;
  else
    return true;
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CM.SERCOM_INTFLAG = SERCOM_I2CM_INTFLAG_ERROR_Msk;
  sercom->I2CM.SERCOM_DATA = data;

  while ((sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_MB_Msk) == 0) {
    if (sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_ERROR_Msk) {
      return false;
    }
  }
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) == 0;
#else
  //Send data
  sercom->I2CM.INTFLAG.bit.ERROR = 1;
  sercom->I2CM.DATA.bit.DATA = data;

  //Wait transmission successful
  while(!sercom->I2CM.INTFLAG.bit.MB) {
    // If a data transfer error occurs, the MB bit may never be set.
    // Check the error bit and bail if it's set.
    // The data transfer errors that can occur (including BUSERR) are all
    // rolled up into INTFLAG.bit.ERROR from STATUS.reg
    if (sercom->I2CM.INTFLAG.bit.ERROR) {
      return false;
    }
  }

  //Problems on line? nack received?
  if(sercom->I2CM.STATUS.bit.RXNACK)
    return false;
  else
    return true;
#endif
}

bool SERCOM::sendDataSlaveWIRE(uint8_t data)
{
#if defined(ARDUINO_SAMD51_E51)
  //Send data
  sercom->I2CS.DATA.bit.DATA = data;

  //Problems on line? nack received?
  if(!sercom->I2CS.INTFLAG.bit.DRDY || sercom->I2CS.STATUS.bit.RXNACK)
    return false;
  else
    return true;
#elif defined(ARDUINO_SAME53_E54)
  sercom->I2CS.SERCOM_DATA = data;
  return (sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY_Msk) &&
         ((sercom->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_RXNACK_Msk) == 0);
#else
  //Send data
  sercom->I2CS.DATA.bit.DATA = data;

  //Problems on line? nack received?
  if(!sercom->I2CS.INTFLAG.bit.DRDY || sercom->I2CS.STATUS.bit.RXNACK)
    return false;
  else
    return true;
#endif
}

bool SERCOM::isMasterWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CS.SERCOM_CTRLA & SERCOM_I2CS_CTRLA_MODE_Msk) ==
         SERCOM_I2CS_CTRLA_MODE_I2C_MASTER;
#else
  return sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION;
#endif
}

bool SERCOM::isSlaveWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CS.SERCOM_CTRLA & SERCOM_I2CS_CTRLA_MODE_Msk) ==
         SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
#else
  return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
#endif
}

bool SERCOM::isBusIdleWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) ==
         SERCOM_I2CM_STATUS_BUSSTATE_IDLE;
#else
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE;
#endif
}

bool SERCOM::isBusOwnerWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) ==
         SERCOM_I2CM_STATUS_BUSSTATE_OWNER;
#else
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE;
#endif
}

bool SERCOM::isBusUnknownWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_UNKNOWN_STATE;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) ==
         SERCOM_I2CM_STATUS_BUSSTATE_UNKNOWN;
#else
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_UNKNOWN_STATE;
#endif
}

bool SERCOM::isArbLostWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.ARBLOST == 1;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_ARBLOST_Msk;
#else
  return sercom->I2CM.STATUS.bit.ARBLOST == 1;
#endif
}

bool SERCOM::isBusBusyWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_BUSY_STATE;
#elif defined(ARDUINO_SAME53_E54)
  return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) ==
         SERCOM_I2CM_STATUS_BUSSTATE_BUSY;
#else
  return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_BUSY_STATE;
#endif
}

bool SERCOM::isDataReadyWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.INTFLAG.bit.DRDY;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY_Msk;
#else
  return sercom->I2CS.INTFLAG.bit.DRDY;
#endif
}

bool SERCOM::isStopDetectedWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.INTFLAG.bit.PREC;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_PREC_Msk;
#else
  return sercom->I2CS.INTFLAG.bit.PREC;
#endif
}

bool SERCOM::isRestartDetectedWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.STATUS.bit.SR;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_SR_Msk;
#else
  return sercom->I2CS.STATUS.bit.SR;
#endif
}

bool SERCOM::isAddressMatch( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.INTFLAG.bit.AMATCH;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH_Msk;
#else
  return sercom->I2CS.INTFLAG.bit.AMATCH;
#endif
}

bool SERCOM::isMasterReadOperationWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CS.STATUS.bit.DIR;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR_Msk;
#else
  return sercom->I2CS.STATUS.bit.DIR;
#endif
}

bool SERCOM::isRXNackReceivedWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  return sercom->I2CM.STATUS.bit.RXNACK;
#elif defined(ARDUINO_SAME53_E54)
  return sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk;
#else
  return sercom->I2CM.STATUS.bit.RXNACK;
#endif
}

int SERCOM::availableWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  if(isMasterWIRE())
    return sercom->I2CM.INTFLAG.bit.SB;
  else
    return sercom->I2CS.INTFLAG.bit.DRDY;
#elif defined(ARDUINO_SAME53_E54)
  if (isMasterWIRE()) {
    return (sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_SB_Msk) != 0;
  }
  return (sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY_Msk) != 0;
#else
  if(isMasterWIRE())
    return sercom->I2CM.INTFLAG.bit.SB;
  else
    return sercom->I2CS.INTFLAG.bit.DRDY;
#endif
}

uint8_t SERCOM::readDataWIRE( void )
{
#if defined(ARDUINO_SAMD51_E51)
  if(isMasterWIRE())
  {
    while (sercom->I2CM.INTFLAG.bit.SB == 0) {
      // Waiting complete receive
      // A variety of errors in the STATUS register can set the ERROR bit in the INTFLAG register
      // In that case, send a stop condition and return false.
      // readDataWIRE should really be able to indicate an error (which would never be used
      // because the readDataWIRE callers (in Wire.cpp) should have checked availableWIRE() first and timed it
      // out if the data never showed up
      if (sercom->I2CM.INTFLAG.bit.MB || sercom->I2CM.INTFLAG.bit.ERROR) {
        sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        return 0xFF;
      }
    }

    return sercom->I2CM.DATA.bit.DATA ;
  }
  else
  {
    return sercom->I2CS.DATA.reg ;
  }
#elif defined(ARDUINO_SAME53_E54)
  if (isMasterWIRE()) {
    while ((sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_SB_Msk) == 0) {
      if (sercom->I2CM.SERCOM_INTFLAG &
          (SERCOM_I2CM_INTFLAG_MB_Msk | SERCOM_I2CM_INTFLAG_ERROR_Msk)) {
        sercom->I2CM.SERCOM_CTRLB =
            (sercom->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
            SERCOM_I2CM_CTRLB_CMD(3);
        return 0xFF;
      }
    }
    return (uint8_t)sercom->I2CM.SERCOM_DATA;
  }
  return (uint8_t)sercom->I2CS.SERCOM_DATA;
#else
  if(isMasterWIRE())
  {
    while (sercom->I2CM.INTFLAG.bit.SB == 0) {
      // Waiting complete receive
      // A variety of errors in the STATUS register can set the ERROR bit in the INTFLAG register
      // In that case, send a stop condition and return false.
      // readDataWIRE should really be able to indicate an error (which would never be used
      // because the readDataWIRE callers (in Wire.cpp) should have checked availableWIRE() first and timed it
      // out if the data never showed up
      if (sercom->I2CM.INTFLAG.bit.MB || sercom->I2CM.INTFLAG.bit.ERROR) {
        sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        return 0xFF;
      }
    }

    return sercom->I2CM.DATA.bit.DATA ;
  }
  else
  {
    return sercom->I2CS.DATA.reg ;
  }
#endif
}

#if defined(ARDUINO_SAMD51_E51)

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

#elif defined(ARDUINO_SAME53_E54)

static const struct {
  sercom_registers_t *sercomPtr;
  uint8_t id_core;
  uint8_t id_slow;
  IRQn_Type irq[4];
} sercomData[] = {
  { SERCOM0_REGS, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW,
    SERCOM0_0_IRQn, SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_OTHER_IRQn },
  { SERCOM1_REGS, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW,
    SERCOM1_0_IRQn, SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_OTHER_IRQn },
  { SERCOM2_REGS, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW,
    SERCOM2_0_IRQn, SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_OTHER_IRQn },
  { SERCOM3_REGS, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW,
    SERCOM3_0_IRQn, SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_OTHER_IRQn },
  { SERCOM4_REGS, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW,
    SERCOM4_0_IRQn, SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_OTHER_IRQn },
  { SERCOM5_REGS, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW,
    SERCOM5_0_IRQn, SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_OTHER_IRQn },
  { SERCOM6_REGS, SERCOM6_GCLK_ID_CORE, SERCOM6_GCLK_ID_SLOW,
    SERCOM6_0_IRQn, SERCOM6_1_IRQn, SERCOM6_2_IRQn, SERCOM6_OTHER_IRQn },
  { SERCOM7_REGS, SERCOM7_GCLK_ID_CORE, SERCOM7_GCLK_ID_SLOW,
    SERCOM7_0_IRQn, SERCOM7_1_IRQn, SERCOM7_2_IRQn, SERCOM7_OTHER_IRQn },
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

int8_t SERCOM::getSercomIndex(void) {
  for(uint8_t i=0; i<(sizeof(sercomData) / sizeof(sercomData[0])); i++) {
    if(sercom == sercomData[i].sercomPtr) return i;
  }
  return -1;
}

uint32_t SERCOM::getSercomFreqRef(void)
{
#if defined(ARDUINO_SAMD51_E51) || defined(ARDUINO_SAME53_E54)
  int8_t idx = getSercomIndex();
  uint8_t gen = 1; // default to GCLK1 (48 MHz) if we can't resolve

  if (idx >= 0)
  {
    uint8_t pch = sercomData[idx].id_core;
#if defined(ARDUINO_SAMD51_E51)
    gen = GCLK->PCHCTRL[pch].bit.GEN;
#elif defined(ARDUINO_SAME53_E54)
    gen = (GCLK_REGS->GCLK_PCHCTRL[pch] & GCLK_PCHCTRL_GEN_Msk) >>
          GCLK_PCHCTRL_GEN_Pos;
#else
    gen = GCLK->PCHCTRL[pch].bit.GEN;
#endif
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

#if defined(ARDUINO_SAMD51_E51) || defined(ARDUINO_SAME53_E54)
// This is currently for overriding an SPI SERCOM's clock source only --
// NOT for UART or WIRE SERCOMs, where it will have unintended consequences.
// It does not check.
// SERCOM clock source override is available only on SAMD51 (not 21).
// A dummy function for SAMD21 (compiles to nothing) is present in SERCOM.h
// so user code doesn't require a lot of conditional situations.
void SERCOM::setClockSource(int8_t idx, SercomClockSource src, bool core) {

  if(src == SERCOM_CLOCK_SOURCE_NO_CHANGE) return;

  uint8_t clk_id = core ? sercomData[idx].id_core : sercomData[idx].id_slow;

#if defined(ARDUINO_SAMD51_E51)
  GCLK->PCHCTRL[clk_id].bit.CHEN = 0;     // Disable timer
  while(GCLK->PCHCTRL[clk_id].bit.CHEN);  // Wait for disable
#elif defined(ARDUINO_SAME53_E54)
  GCLK_REGS->GCLK_PCHCTRL[clk_id] &= ~GCLK_PCHCTRL_CHEN_Msk;
  while (GCLK_REGS->GCLK_PCHCTRL[clk_id] & GCLK_PCHCTRL_CHEN_Msk);
#else
  GCLK->PCHCTRL[clk_id].bit.CHEN = 0;     // Disable timer
  while(GCLK->PCHCTRL[clk_id].bit.CHEN);  // Wait for disable
#endif

  if(core) clockSource = src; // Save SercomClockSource value

  // From cores/arduino/startup.c:
  // GCLK0 = F_CPU (this is 120 MHz and exceeds SERCOM maximum)
  // GCLK1 = 48 MHz
  // GCLK2 = 100 MHz
  // GCLK3 = XOSC32K
  // GCLK4 = 12 MHz
  if(src == SERCOM_CLOCK_SOURCE_FCPU) {
#if defined(ARDUINO_SAMD51_E51)
    GCLK->PCHCTRL[clk_id].reg =
        GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); // Guard Sercom from exceeding 100 MHz maximum
#elif defined(ARDUINO_SAME53_E54)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK2 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
        GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); // Guard Sercom from exceeding 100 MHz maximum
#endif
    if (core)
      freqRef = 100000000; // Save clock frequency value
  }
  else if (src == SERCOM_CLOCK_SOURCE_48M)
  {
#if defined(ARDUINO_SAMD51_E51)
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#elif defined(ARDUINO_SAME53_E54)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK1 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif
    if(core) freqRef = 48000000;
  } else if(src == SERCOM_CLOCK_SOURCE_100M) {
#if defined(ARDUINO_SAMD51_E51)
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#elif defined(ARDUINO_SAME53_E54)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK2 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif
    if(core) freqRef = 100000000;
  } else if(src == SERCOM_CLOCK_SOURCE_32K) {
#if defined(ARDUINO_SAMD51_E51)
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#elif defined(ARDUINO_SAME53_E54)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK3 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif
    if(core) freqRef = 32768;
  } else if(src == SERCOM_CLOCK_SOURCE_12M) {
#if defined(ARDUINO_SAMD51_E51)
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#elif defined(ARDUINO_SAME53_E54)
    GCLK_REGS->GCLK_PCHCTRL[clk_id] = GCLK_PCHCTRL_GEN_GCLK4 |
                                       GCLK_PCHCTRL_CHEN_Msk;
#else
    GCLK->PCHCTRL[clk_id].reg =
      GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
#endif
    if(core) freqRef = 12000000;
  }

#if defined(ARDUINO_SAMD51_E51)
  while(!GCLK->PCHCTRL[clk_id].bit.CHEN); // Wait for clock enable
#elif defined(ARDUINO_SAME53_E54)
  while ((GCLK_REGS->GCLK_PCHCTRL[clk_id] & GCLK_PCHCTRL_CHEN_Msk) == 0);
#else
  while(!GCLK->PCHCTRL[clk_id].bit.CHEN); // Wait for clock enable
#endif
}
#endif

void SERCOM::initClockNVIC( void )
{
  int8_t idx = getSercomIndex();
  if(idx < 0) return; // We got a problem here

#if defined(ARDUINO_SAMD51_E51) || defined(ARDUINO_SAME53_E54)

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
