/*
  Copyright (c) 2016 Arduino LLC. 2017 Adafruit  All right reserved.

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

#pragma once

#include <Arduino.h>

class I2SDevice_SAMD51 {
public:

  I2SDevice_SAMD51(I2s& _i2s) :
    i2s(_i2s)
  {
    // Empty
  }

  inline void reset() {
    while(i2s.SYNCBUSY.bit.SWRST);
    i2s.CTRLA.bit.SWRST = 1;
    //while(i2s.SYNCBUSY.bit.SWRST || i2s.SYNCBUSY.bit.ENABLE); //wait for sync
  }

  inline void disable() {
    while(i2s.SYNCBUSY.bit.ENABLE);
    i2s.CTRLA.bit.ENABLE = 0;
  }

  inline void enable() {
    while(i2s.SYNCBUSY.bit.ENABLE);
    i2s.CTRLA.bit.ENABLE = 1;
  }

  inline int glckId(int index) {
    return (index == 0) ? I2S_GCLK_ID_0 : I2S_GCLK_ID_1;
  }
// --- New for SAMD51, taken from Adafrui Zero I2S Library ----
  inline void setMasterClockOutputDivisionFactor(int index, uint8_t mckoutdiv) {
  // printRegisters();
  //  Serial.println ("index: " + String(index));
  //  Serial.println ("mckoutdiv: " + String(mckoutdiv));
    while(i2s.SYNCBUSY.reg); // wait for sync
    i2s.CLKCTRL[index].reg |= I2S_CLKCTRL_MCKOUTDIV(mckoutdiv); //(I2S_CLKCTRL_MCKOUTDIV_Msk & ((mckoutdiv) << I2S_CLKCTRL_MCKOUTDIV_Pos));
  // Serial.println ("mckoutdiv: " + String(i2s.CLKCTRL[index].bit.MCKOUTDIV));
  }
  
  inline void setMasterClockDivisionFactor(int index, uint8_t mckdiv) {
    while(i2s.SYNCBUSY.reg); // wait for sync
    i2s.CLKCTRL[index].reg |= I2S_CLKCTRL_MCKDIV(mckdiv);
  }
  
  inline void setMasterClockEnable(int index) {
    while(i2s.SYNCBUSY.reg); // wait for sync
    i2s.CLKCTRL[index].reg |= I2S_CLKCTRL_MCKEN;
  }
  
  inline void setFrameSyncPulse(int index, uint8_t fswidth) { // possible values are: I2S_CLKCTRL_FSWIDTH_SLOT_Val, I2S_CLKCTRL_FSWIDTH_HALF_Val, I2S_CLKCTRL_FSWIDTH_BIT_Val, I2S_CLKCTRL_FSWIDTH_BURST_Val
    while(i2s.SYNCBUSY.reg);  // wait for sync
    i2s.CLKCTRL[index].reg |= I2S_CLKCTRL_FSWIDTH(fswidth); 
  }
  
  // added bitsPerSample parameter
  inline void setTxMode(int index, int bitsPerSample) {

    // Previously used here:  
    //i2s.RXCTRL.bit.SERMODE = 0x01;
    //i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS... <- I think Atmel's datasheet has an errata, it has no sense to set this serializer as RX
    //i2s.TXCTRL.reg |= 0x01;
    
    uint8_t wordSize = 0;
    switch (bitsPerSample)
    {
      case 8:
        wordSize = I2S_TXCTRL_DATASIZE_8_Val;
        break;
      case 16:
        wordSize = I2S_TXCTRL_DATASIZE_16_Val;
        break;
      case 24:
        wordSize = I2S_TXCTRL_DATASIZE_24_Val;
        break;
      case 32:
        wordSize = I2S_TXCTRL_DATASIZE_32_Val;
        break;
    }

    i2s.TXCTRL.reg &= I2S_TXCTRL_RESETVALUE;
    i2s.TXCTRL.reg = I2S_TXCTRL_DMA_SINGLE |
            I2S_TXCTRL_MONO_STEREO |
            I2S_TXCTRL_BITREV_MSBIT |
            I2S_TXCTRL_EXTEND_ZERO |
            I2S_TXCTRL_WORDADJ_RIGHT |
            I2S_TXCTRL_DATASIZE(wordSize) |
            I2S_TXCTRL_TXSAME_ZERO |
            I2S_TXCTRL_TXDEFAULT_ZERO;

   // printRegisters();
   // while(true);
 }

  // added bitsPerSample parameter
  inline void setRxMode(int index, int bitsPerSample) {
  
    //i2s.RXCTRL.bit.SERMODE = 0x00;
    //i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS...
    //i2s.TXCTRL.reg |= 0x00;
    
    uint8_t wordSize = 0;
    switch (bitsPerSample)
    {
      case 8:
        wordSize = I2S_RXCTRL_DATASIZE_8_Val;
        break;
      case 16:
        wordSize = I2S_RXCTRL_DATASIZE_16_Val;
        break;
      case 24:
        wordSize = I2S_RXCTRL_DATASIZE_24_Val;
        break;
      case 32:
        wordSize = I2S_RXCTRL_DATASIZE_32_Val;
        break;
    }

    //while(i2s.SYNCBUSY.reg);  // wait for sync
    i2s.RXCTRL.reg &= I2S_RXCTRL_RESETVALUE;
    i2s.RXCTRL.reg = I2S_RXCTRL_DMA_SINGLE | //I2S_RXCTRL_DMA_MULTIPLE |   <- IT HANGS 
        I2S_RXCTRL_MONO_STEREO |
        I2S_RXCTRL_BITREV_MSBIT |
        I2S_RXCTRL_EXTEND_ZERO |  // ZERO fills with ONEs and ONE fills with ZEROs, CMSIS errata?
        I2S_RXCTRL_WORDADJ_RIGHT | 
        I2S_RXCTRL_DATASIZE(wordSize) |
        I2S_RXCTRL_SLOTADJ_RIGHT | 
        I2S_RXCTRL_CLKSEL_CLK0 |
        I2S_RXCTRL_SERMODE_RX;
    
    // printRegisters();
    // while(true);
}
// end new-----

  inline void setSerialClockSelectMasterClockDiv(int index) {
    i2s.CLKCTRL[index].bit.SCKSEL = I2S_CLKCTRL_SCKSEL_MCKDIV_Val;
  }

  inline void setSerialClockSelectPin(int index) {
    i2s.CLKCTRL[index].bit.SCKSEL = I2S_CLKCTRL_SCKSEL_SCKPIN_Val;
  }

  inline void setFrameSyncSelectSerialClockDiv(int index) {
    i2s.CLKCTRL[index].bit.FSSEL = I2S_CLKCTRL_FSSEL_SCKDIV_Val;
  }

  inline void setFrameSyncSelectPin(int index) {
    i2s.CLKCTRL[index].bit.FSSEL = I2S_CLKCTRL_FSSEL_FSPIN_Val;
  }

  inline void set0BitDelay(int index) {
    i2s.CLKCTRL[index].bit.BITDELAY = I2S_CLKCTRL_BITDELAY_LJ_Val;
  }

  inline void set1BitDelay(int index) {
    i2s.CLKCTRL[index].bit.BITDELAY = I2S_CLKCTRL_BITDELAY_I2S_Val;
  }

  inline void setNumberOfSlots(int index, int nbslots) {
    i2s.CLKCTRL[index].bit.NBSLOTS = nbslots;
  }

  inline void setSlotSize(int index, int size) {
    switch (size) {
     case 32:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_32_Val;
       break;

     case 24:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_24_Val;
       break;

     case 16:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_16_Val;
       break;

     case 8:
       i2s.CLKCTRL[index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_8_Val;
       break;
    }
  }

  inline void setDataSize(int index, int size) {
    switch (size) {
     case 32:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_32_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_32_Val;
       break;

     case 24:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_24_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_24_Val;
       break;

     case 16:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_16_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_16_Val;
       break;

     case 8:
       i2s.RXCTRL.bit.DATASIZE = I2S_RXCTRL_DATASIZE_8_Val;
       i2s.TXCTRL.bit.DATASIZE = I2S_TXCTRL_DATASIZE_8_Val;
       break;
    }
  }

  inline void setSlotAdjustedRight(int index) {
    i2s.RXCTRL.bit.SLOTADJ = I2S_RXCTRL_SLOTADJ_RIGHT_Val;
    i2s.TXCTRL.bit.SLOTADJ = I2S_TXCTRL_SLOTADJ_RIGHT_Val;
  }

  inline void setSlotAdjustedLeft(int index) {
    i2s.RXCTRL.bit.SLOTADJ = I2S_RXCTRL_SLOTADJ_LEFT_Val;
    i2s.TXCTRL.bit.SLOTADJ = I2S_TXCTRL_SLOTADJ_LEFT_Val;
  }

  inline void setClockUnit(int index) {
    i2s.RXCTRL.bit.CLKSEL = (index == 0) ? I2S_RXCTRL_CLKSEL_CLK0_Val : I2S_RXCTRL_CLKSEL_CLK1_Val;
  }
/*
  inline void setTxMode(int index) {
    i2s.RXCTRL.bit.SERMODE = 0x01;
    i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS...
    i2s.TXCTRL.reg |= 0x01;
  }

  inline void setRxMode(int index) {
    i2s.RXCTRL.bit.SERMODE = 0x00;
    i2s.TXCTRL.reg &= ~(0x03); //TODO: why is this not in CMSIS...
    i2s.TXCTRL.reg |= 0x00;
  }
*/
  inline void enableClockUnit(int index) {
    if (index == 0) {
      while(i2s.SYNCBUSY.bit.CKEN0);
      i2s.CTRLA.bit.CKEN0 = 1;
    } else {
      while(i2s.SYNCBUSY.bit.CKEN1);
      i2s.CTRLA.bit.CKEN1 = 1;
    }
  }

  inline void disableClockUnit(int index) {
    if (index == 0) {
      while(i2s.SYNCBUSY.bit.CKEN0);
      i2s.CTRLA.bit.CKEN0 = 0;
    } else {
      while(i2s.SYNCBUSY.bit.CKEN1);
      i2s.CTRLA.bit.CKEN1 = 0;
    }
  }

  // this should be explitted or filtered with "bool txrx"
  inline void enableSerializer(int index) {
    i2s.CTRLA.bit.RXEN = 1;
    i2s.CTRLA.bit.TXEN = 1;
  }

  // this should be explitted or filtered with "bool txrx"
  inline void disableSerializer(int index) {
    i2s.CTRLA.bit.RXEN = 0;
    i2s.CTRLA.bit.TXEN = 0; 
  }

  // this is also new, dmaTriggerSource also depends on serializer furthermore clock source index
  // rx = 0, tx = 1
  inline int dmaTriggerSource(int index, bool txrx = 0) {
    if (txrx) {
      return (index == 0) ? I2S_DMAC_ID_TX_0 : I2S_DMAC_ID_TX_1;
    } else {
      return (index == 0) ? I2S_DMAC_ID_RX_0 : I2S_DMAC_ID_RX_1;
    }

    // D21
    /*
    if (i2s.SERCTRL[index].bit.SERMODE == I2S_SERCTRL_SERMODE_TX_Val) {
      return (index == 0) ? I2S_DMAC_ID_TX_0 : I2S_DMAC_ID_TX_1;
    } else {
      return (index == 0) ? I2S_DMAC_ID_RX_0 : I2S_DMAC_ID_RX_1;
    }
    */
  }

  inline int txReady(int index) {
    return (index == 0) ? i2s.INTFLAG.bit.TXRDY0 :i2s.INTFLAG.bit.TXRDY1;
  }

  inline void writeData(int index, int32_t value) {
    while (i2s.SYNCBUSY.bit.TXDATA);

    i2s.TXDATA.bit.DATA = value;
  }

  inline void clearTxReady(int index) {
    if (index == 0) {
     i2s.INTFLAG.bit.TXRDY0 = 1;
    } else {
     i2s.INTFLAG.bit.TXRDY1 = 1;
    }
  }

  inline int rxReady(int index) {
    return (index == 0) ? i2s.INTFLAG.bit.RXRDY0 :i2s.INTFLAG.bit.RXRDY1;
  }
// for blocking read ---
  inline void read(int32_t *left, int32_t *right) {
   // printRegisters ();
    while( (!i2s.INTFLAG.bit.RXRDY0) || i2s.SYNCBUSY.bit.RXDATA );
    *left = i2s.RXDATA.reg;

    while( (!i2s.INTFLAG.bit.RXRDY0) || i2s.SYNCBUSY.bit.RXDATA );
    *right = i2s.RXDATA.reg;
/*
    if (i2s.INTFLAG.bit.RXOR0) {
      Serial.println("overrun");
      i2s.INTENCLR.bit.RXOR0 = 0b1;
    }
*/
  }
// ---
  inline int32_t readData(int index) {

    //printRegisters ();
    //while( (!i2s.INTFLAG.bit.RXRDY0) || i2s.SYNCBUSY.bit.RXDATA );
    //return i2s.RXDATA.reg;

    while(i2s.SYNCBUSY.bit.RXDATA);
      return i2s.RXDATA.bit.DATA;
  }

  inline void clearRxReady(int index) {
    if (index == 0) {
     i2s.INTFLAG.bit.RXRDY0 = 1;
    } else {
     i2s.INTFLAG.bit.RXRDY1 = 1;
    }
  }

  inline void* data(int index) {
    return (void*)&i2s.RXDATA.reg;
  }


  // BOOOM, This is my arduino's embeded debugger xD
  inline void printRegisters()
  {
    // MCLK_APBAMASK
    Serial.println ("SAMD51_MCLK Main Clock (APBA Mask)");
    Serial.println ("(32-bit) MCLK->APBAMASK: 0x"   + String(MCLK->APBAMASK.reg,                HEX) + "\t, 0b" + String(MCLK->APBAMASK.reg,                BIN));
    Serial.println ("\t [31:16] Reserved");
    Serial.println ("\t [15]    TC1_:           0x" + String(MCLK->APBAMASK.bit.TC1_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.TC1_,           BIN) + "\t TC1 APB Clock Enable");
    Serial.println ("\t [14]    TC0_:           0x" + String(MCLK->APBAMASK.bit.TC0_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.TC0_,           BIN) + "\t TC0 APB Clock Enable");
    Serial.println ("\t [13]    SERCOM1_:       0x" + String(MCLK->APBAMASK.bit.SERCOM1_,       HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.SERCOM1_,       BIN) + "\t SERCOM1 APB Clock Enable");
    Serial.println ("\t [12]    SERCOM0_:       0x" + String(MCLK->APBAMASK.bit.SERCOM0_,       HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.SERCOM0_,       BIN) + "\t SERCOM0 APB Clock Enable");
    Serial.println ("\t [11]    FREQM_:         0x" + String(MCLK->APBAMASK.bit.FREQM_,         HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.FREQM_,         BIN) + "\t FREQM APB Clock Enable");
    Serial.println ("\t [10]    EIC_:           0x" + String(MCLK->APBAMASK.bit.EIC_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.EIC_,           BIN) + "\t EIC APB Clock Enable");
    Serial.println ("\t [9]     RTC_:           0x" + String(MCLK->APBAMASK.bit.RTC_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.RTC_,           BIN) + "\t RTC APB Clock Enable");
    Serial.println ("\t [8]     WDT_:           0x" + String(MCLK->APBAMASK.bit.WDT_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.WDT_,           BIN) + "\t WDT APB Clock Enable");
    Serial.println ("\t [7]     GCLK_:          0x" + String(MCLK->APBAMASK.bit.GCLK_,          HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.GCLK_,          BIN) + "\t GCLK APB Clock Enable");
    Serial.println ("\t [6]     SUPC_:          0x" + String(MCLK->APBAMASK.bit.SUPC_,          HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.SUPC_,          BIN) + "\t SUPC APB Clock Enable");
    Serial.println ("\t [5]     OSC32KCTRL_:    0x" + String(MCLK->APBAMASK.bit.OSC32KCTRL_,    HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.OSC32KCTRL_,    BIN) + "\t OSC32KCTRL APB Clock Enable");
    Serial.println ("\t [4]     OSCCTRL_:       0x" + String(MCLK->APBAMASK.bit.OSCCTRL_,       HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.OSCCTRL_,       BIN) + "\t OSCCTRL APB Clock Enable");
    Serial.println ("\t [3]     RSTC_:          0x" + String(MCLK->APBAMASK.bit.RSTC_,          HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.RSTC_,          BIN) + "\t RSTC APB Clock Enable");
    Serial.println ("\t [2]     MCLK_:          0x" + String(MCLK->APBAMASK.bit.MCLK_,          HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.MCLK_,          BIN) + "\t MCLK APB Clock Enable");
    Serial.println ("\t [1]     PM_:            0x" + String(MCLK->APBAMASK.bit.PM_,            HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.PM_,            BIN) + "\t PM APB Clock Enable");
    Serial.println ("\t [0]     PAC_:           0x" + String(MCLK->APBAMASK.bit.PAC_,           HEX) + "\t, 0b" + String(MCLK->APBAMASK.bit.PAC_,           BIN) + "\t PAC APB Clock Enable");
    Serial.println ();

    // GCLK_CTRLA
    Serial.println ("SAMD51_GCLK Generic Clock Generator (Control)");
    Serial.println ("(8-bit) GCLK->CTRLA: 0x" + String(GCLK->CTRLA.reg,     HEX) + "\t, 0b" + String(GCLK->CTRLA.reg,       BIN));
    Serial.println ("\t [7:1] Reserved");
    Serial.println ("\t [0]   SWRST: 0x" + String(GCLK->CTRLA.bit.SWRST,    HEX) + "\t, 0b" + String(GCLK->CTRLA.bit.SWRST, BIN) + "\t Software Reset ");
    Serial.println ();

    // GCLK_PCHCTRL[I2S_GCLK_ID_0]
    Serial.println ("SAMD51_GCLK Generic Clock Generator (Peripheral Clock Control)");
    Serial.println ("(32-bit) GCLK->PCHCTRL[I2S_GCLK_ID_0]: 0x"   + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].reg,  HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].reg,         BIN));
    Serial.println ("\t [31:8] Reserved");
    Serial.println ("\t [7]    WRTLOCK: 0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.WRTLOCK, HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.WRTLOCK, BIN) + "\t Write Lock");
    Serial.println ("\t [6]    CHEN:    0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.CHEN,    HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.CHEN,    BIN) + "\t Channel Enable");
    Serial.println ("\t [5:4]  Reserved");   
    Serial.println ("\t [3:0]  GEN:     0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.GEN,     HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_0].bit.GEN,     BIN) + "\t Generic Clock Generator");
    Serial.println ();

    // GCLK_PCHCTRL[I2S_GCLK_ID_1]
    Serial.println ("SAMD51_GCLK Generic Clock Generator (Peripheral Clock Control)");
    Serial.println ("(32-bit) GCLK->PCHCTRL[I2S_GCLK_ID_1]: 0x"   + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].reg,  HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].reg,         BIN));
    Serial.println ("\t [31:8] Reserved");
    Serial.println ("\t [7]    WRTLOCK: 0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.WRTLOCK, HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.WRTLOCK, BIN) + "\t Write Lock");
    Serial.println ("\t [6]    CHEN:    0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.CHEN,    HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.CHEN,    BIN) + "\t Channel Enable");
    Serial.println ("\t [5:4]  Reserved");   
    Serial.println ("\t [3:0]  GEN:     0x" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.GEN,     HEX) + "\t, 0b" + String(GCLK->PCHCTRL[I2S_GCLK_ID_1].bit.GEN,     BIN) + "\t Generic Clock Generator");
    Serial.println ();

    // I2S_CTRLA
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Control A)");
    Serial.println ("(8-bit) I2S->CTRLA: 0x"     + String(I2S->CTRLA.reg,   HEX) + "\t, 0b" + String(I2S->CTRLA.reg,        BIN));
    Serial.println ("\t [7:6]   Reserved");
    Serial.println ("\t [5]     RXEN:   0x" + String(I2S->CTRLA.bit.RXEN,   HEX) + "\t, 0b" + String(I2S->CTRLA.bit.RXEN,   BIN) + "\t Rx Serializer Enable ");
    Serial.println ("\t [4]     TXEN:   0x" + String(I2S->CTRLA.bit.TXEN,   HEX) + "\t, 0b" + String(I2S->CTRLA.bit.TXEN,   BIN) + "\t Tx Serializer Enable");
    Serial.println ("\t [3]     CKEN1:  0x" + String(I2S->CTRLA.bit.CKEN1,  HEX) + "\t, 0b" + String(I2S->CTRLA.bit.CKEN1,  BIN) + "\t Clock Unit 1 Enable");
    Serial.println ("\t [2]     CKEN0:  0x" + String(I2S->CTRLA.bit.CKEN0,  HEX) + "\t, 0b" + String(I2S->CTRLA.bit.CKEN0,  BIN) + "\t Clock Unit 0 Enable");
    Serial.println ("\t [1]     ENABLE: 0x" + String(I2S->CTRLA.bit.ENABLE, HEX) + "\t, 0b" + String(I2S->CTRLA.bit.ENABLE, BIN) + "\t Enable");
    Serial.println ("\t [0]     SWRST:  0x" + String(I2S->CTRLA.bit.SWRST,  HEX) + "\t, 0b" + String(I2S->CTRLA.bit.SWRST,  BIN) + "\t Software Reset");
    Serial.println ();

    // I2S_CLKCTRL0
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Clock Unit 0 Control)");
    Serial.println ("(32-bit) I2S->CLKCTRL[0]: 0x"  + String(I2S->CLKCTRL[0].reg,      HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].reg,           BIN));
    Serial.println ("\t [31:30] Reserved");
    Serial.println ("\t [29:24] MCKOUTDIV: 0x" + String(I2S->CLKCTRL[0].bit.MCKOUTDIV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.MCKOUTDIV, BIN) + "\t Master Clock Output Division Factor");
    Serial.println ("\t [23:22] Reserved");
    Serial.println ("\t [21:16] MCKDIV:    0x" + String(I2S->CLKCTRL[0].bit.MCKDIV,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.MCKDIV,    BIN) + "\t Master Clock Division Factor");
    Serial.println ("\t [15]    MCKOUTINV: 0x" + String(I2S->CLKCTRL[0].bit.MCKOUTINV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.MCKOUTINV, BIN) + "\t Master Clock Output Invert");
    Serial.println ("\t [14]    MCKEN:     0x" + String(I2S->CLKCTRL[0].bit.MCKEN,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.MCKEN,     BIN) + "\t Master Clock Enable");
    Serial.println ("\t [13]    MCKSEL:    0x" + String(I2S->CLKCTRL[0].bit.MCKSEL,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.MCKSEL,    BIN) + "\t Master Clock Select ");
    Serial.println ("\t [12]    SCKOUTINV: 0x" + String(I2S->CLKCTRL[0].bit.SCKOUTINV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.SCKOUTINV, BIN) + "\t Serial Clock Output Invert");
    Serial.println ("\t [11]    SCKSEL:    0x" + String(I2S->CLKCTRL[0].bit.SCKSEL,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.SCKSEL,    BIN) + "\t Serial Clock Select");
    Serial.println ("\t [10]    FSOUTINV:  0x" + String(I2S->CLKCTRL[0].bit.FSOUTINV,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.FSOUTINV,  BIN) + "\t Frame Sync Output Invert");
    Serial.println ("\t [9]     FSINV:     0x" + String(I2S->CLKCTRL[0].bit.FSINV,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.FSINV,     BIN) + "\t Frame Sync Invert");
    Serial.println ("\t [8]     FSSEL:     0x" + String(I2S->CLKCTRL[0].bit.FSSEL,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.FSSEL,     BIN) + "\t Frame Sync Select");
    Serial.println ("\t [7]     BITDELAY:  0x" + String(I2S->CLKCTRL[0].bit.BITDELAY,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.BITDELAY,  BIN) + "\t Data Delay from Frame Sync");
    Serial.println ("\t [6:5]   FSWIDTH:   0x" + String(I2S->CLKCTRL[0].bit.FSWIDTH,   HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.FSWIDTH,   BIN) + "\t Frame Sync Width");
    Serial.println ("\t [4:2]   NBSLOTS:   0x" + String(I2S->CLKCTRL[0].bit.NBSLOTS,   HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.NBSLOTS,   BIN) + "\t Number of Slots in Frame");
    Serial.println ("\t [1:0]   SLOTSIZE:  0x" + String(I2S->CLKCTRL[0].bit.SLOTSIZE,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[0].bit.SLOTSIZE,  BIN) + "\t Slot Size");
    Serial.println ();

    // I2S_CLKCTRL1
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Clock Unit 1 Control)");
    Serial.println ("(32-bit) I2S->CLKCTRL[1]: 0x" + String(I2S->CLKCTRL[1].reg,       HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].reg,           BIN));
    Serial.println ("\t [31:30] Reserved");
    Serial.println ("\t [29:24] MCKOUTDIV: 0x" + String(I2S->CLKCTRL[1].bit.MCKOUTDIV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.MCKOUTDIV, BIN) + "\t Master Clock Output Division Factor");
    Serial.println ("\t [23:22] Reserved");
    Serial.println ("\t [21:16] MCKDIV:    0x" + String(I2S->CLKCTRL[1].bit.MCKDIV,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.MCKDIV,    BIN) + "\t Master Clock Division Factor");
    Serial.println ("\t [15]    MCKOUTINV: 0x" + String(I2S->CLKCTRL[1].bit.MCKOUTINV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.MCKOUTINV, BIN) + "\t Master Clock Output Invert");
    Serial.println ("\t [14]    MCKEN:     0x" + String(I2S->CLKCTRL[1].bit.MCKEN,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.MCKEN,     BIN) + "\t Master Clock Enable");
    Serial.println ("\t [13]    MCKSEL:    0x" + String(I2S->CLKCTRL[1].bit.MCKSEL,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.MCKSEL,    BIN) + "\t Master Clock Select ");
    Serial.println ("\t [12]    SCKOUTINV: 0x" + String(I2S->CLKCTRL[1].bit.SCKOUTINV, HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.SCKOUTINV, BIN) + "\t Serial Clock Output Invert");
    Serial.println ("\t [11]    SCKSEL:    0x" + String(I2S->CLKCTRL[1].bit.SCKSEL,    HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.SCKSEL,    BIN) + "\t Serial Clock Select");
    Serial.println ("\t [10]    FSOUTINV:  0x" + String(I2S->CLKCTRL[1].bit.FSOUTINV,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.FSOUTINV,  BIN) + "\t Frame Sync Output Invert");
    Serial.println ("\t [9]     FSINV:     0x" + String(I2S->CLKCTRL[1].bit.FSINV,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.FSINV,     BIN) + "\t Frame Sync Invert");
    Serial.println ("\t [8]     FSSEL:     0x" + String(I2S->CLKCTRL[1].bit.FSSEL,     HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.FSSEL,     BIN) + "\t Frame Sync Select");
    Serial.println ("\t [7]     BITDELAY:  0x" + String(I2S->CLKCTRL[1].bit.BITDELAY,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.BITDELAY,  BIN) + "\t Data Delay from Frame Sync");
    Serial.println ("\t [6:5]   FSWIDTH:   0x" + String(I2S->CLKCTRL[1].bit.FSWIDTH,   HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.FSWIDTH,   BIN) + "\t Frame Sync Width");
    Serial.println ("\t [4:2]   NBSLOTS:   0x" + String(I2S->CLKCTRL[1].bit.NBSLOTS,   HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.NBSLOTS,   BIN) + "\t Number of Slots in Frame");
    Serial.println ("\t [1:0]   SLOTSIZE:  0x" + String(I2S->CLKCTRL[1].bit.SLOTSIZE,  HEX) + "\t, 0b" + String(I2S->CLKCTRL[1].bit.SLOTSIZE,  BIN) + "\t Slot Size");
    Serial.println ();

    // I2S_INTENCLR
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Interrupt Enable Clear)");
    Serial.println ("(16-bit) I2S->INTENCLR: 0x" + String(I2S->INTENCLR.reg,   HEX) + "\t, 0b" + String(I2S->INTENCLR.reg,        BIN));
    Serial.println ("\t [15:14] Reserved");
    Serial.println ("\t [13]    TXUR1:  0x" + String(I2S->INTENCLR.bit.TXUR1,  HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.TXUR1,  BIN) + "\t Transmit Underrun 1 Interrupt");
    Serial.println ("\t [12]    TXUR0:  0x" + String(I2S->INTENCLR.bit.TXUR0,  HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.TXUR0,  BIN) + "\t Transmit Underrun 0 Interrupt");
    Serial.println ("\t [11:10] Reserved");
    Serial.println ("\t [9]     TXRDY1: 0x" + String(I2S->INTENCLR.bit.TXRDY1, HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.TXRDY1, BIN) + "\t Transmit Ready 1 Interrupt Enable");
    Serial.println ("\t [8]     TXRDY0: 0x" + String(I2S->INTENCLR.bit.TXRDY0, HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.TXRDY0, BIN) + "\t Transmit Ready 0 Interrupt Enable");
    Serial.println ("\t [7:6]   Reserved");
    Serial.println ("\t [5]     RXOR1:  0x" + String(I2S->INTENCLR.bit.RXOR1,  HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.RXOR1,  BIN) + "\t Receive Overrun 1 Interrupt Enable");
    Serial.println ("\t [4]     RXOR0:  0x" + String(I2S->INTENCLR.bit.RXOR0,  HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.RXOR0,  BIN) + "\t Receive Overrun 0 Interrupt Enable");
    Serial.println ("\t [3:2]   Reserved");
    Serial.println ("\t [1]     RXRDY1: 0x" + String(I2S->INTENCLR.bit.RXRDY1, HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.RXRDY1, BIN) + "\t Receive Ready 1 Interrupt Enable");
    Serial.println ("\t [0]     RXRDY0: 0x" + String(I2S->INTENCLR.bit.RXRDY0, HEX) + "\t, 0b" + String(I2S->INTENCLR.bit.RXRDY0, BIN) + "\t Receive Ready 0 Interrupt Enable");
    Serial.println ();

    // I2S_INTENSET
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Interrupt Enable Set)");
    Serial.println ("(16-bit) I2S->INTENSET: 0x" + String(I2S->INTENSET.reg,   HEX) + "\t, 0b" + String(I2S->INTENSET.reg,        BIN));
    Serial.println ("\t [15:14] Reserved");
    Serial.println ("\t [13]    TXUR1:  0x" + String(I2S->INTENSET.bit.TXUR1,  HEX) + "\t, 0b" + String(I2S->INTENSET.bit.TXUR1,  BIN) + "\t Transmit Underrun 1 Interrupt Enable");
    Serial.println ("\t [12]    TXUR0:  0x" + String(I2S->INTENSET.bit.TXUR0,  HEX) + "\t, 0b" + String(I2S->INTENSET.bit.TXUR0,  BIN) + "\t Transmit Underrun 0 Interrupt Enable");
    Serial.println ("\t [11:10] Reserved");
    Serial.println ("\t [9]     TXRDY1: 0x" + String(I2S->INTENSET.bit.TXRDY1, HEX) + "\t, 0b" + String(I2S->INTENSET.bit.TXRDY1, BIN) + "\t Transmit Ready 1 Interrupt Enable");
    Serial.println ("\t [8]     TXRDY0: 0x" + String(I2S->INTENSET.bit.TXRDY0, HEX) + "\t, 0b" + String(I2S->INTENSET.bit.TXRDY0, BIN) + "\t Transmit Ready 0 Interrupt Enable");
    Serial.println ("\t [7:6]   Reserved");
    Serial.println ("\t [5]     RXOR1:  0x" + String(I2S->INTENSET.bit.RXOR1,  HEX) + "\t, 0b" + String(I2S->INTENSET.bit.RXOR1,  BIN) + "\t Receive Overrun 1 Interrupt Enable");
    Serial.println ("\t [4]     RXOR0:  0x" + String(I2S->INTENSET.bit.RXOR0,  HEX) + "\t, 0b" + String(I2S->INTENSET.bit.RXOR0,  BIN) + "\t Receive Overrun 0 Interrupt Enable");
    Serial.println ("\t [3:2]   Reserved");
    Serial.println ("\t [1]     RXRDY1: 0x" + String(I2S->INTENSET.bit.RXRDY1, HEX) + "\t, 0b" + String(I2S->INTENSET.bit.RXRDY1, BIN) + "\t Receive Ready 1 Interrupt Enable");
    Serial.println ("\t [0]     RXRDY0: 0x" + String(I2S->INTENSET.bit.RXRDY0, HEX) + "\t, 0b" + String(I2S->INTENSET.bit.RXRDY0, BIN) + "\t Receive Ready 0 Interrupt Enable");
    Serial.println ();

    // I2S_INTFLAG
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Interrupt Flag Status and Clear)");
    Serial.println ("(16-bit) I2S->INTFLAG: 0x" + String(I2S->INTFLAG.reg,    HEX) + "\t, 0b" + String(I2S->INTFLAG.reg,        BIN));
    Serial.println ("\t [15:14] Reserved");
    Serial.println ("\t [13]    TXUR1:  0x" + String(I2S->INTFLAG.bit.TXUR1,  HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.TXUR1,  BIN) + "\t Transmit Underrun 1");
    Serial.println ("\t [12]    TXUR0:  0x" + String(I2S->INTFLAG.bit.TXUR0,  HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.TXUR0,  BIN) + "\t Transmit Underrun 0");
    Serial.println ("\t [11:10] Reserved");
    Serial.println ("\t [9]     TXRDY1: 0x" + String(I2S->INTFLAG.bit.TXRDY1, HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.TXRDY1, BIN) + "\t Transmit Ready 1");
    Serial.println ("\t [8]     TXRDY0: 0x" + String(I2S->INTFLAG.bit.TXRDY0, HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.TXRDY0, BIN) + "\t Transmit Ready 0");
    Serial.println ("\t [7:6]   Reserved");
    Serial.println ("\t [5]     RXOR1:  0x" + String(I2S->INTFLAG.bit.RXOR1,  HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.RXOR1,  BIN) + "\t Receive Overrun 1");
    Serial.println ("\t [4]     RXOR0:  0x" + String(I2S->INTFLAG.bit.RXOR0,  HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.RXOR0,  BIN) + "\t Receive Overrun 0");
    Serial.println ("\t [3:2]   Reserved");
    Serial.println ("\t [1]     RXRDY1: 0x" + String(I2S->INTFLAG.bit.RXRDY1, HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.RXRDY1, BIN) + "\t Receive Ready 1");
    Serial.println ("\t [0]     RXRDY0: 0x" + String(I2S->INTFLAG.bit.RXRDY0, HEX) + "\t, 0b" + String(I2S->INTFLAG.bit.RXRDY0, BIN) + "\t Receive Ready 0");
    Serial.println ();

    // I2S_SYNCBUSY
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Synchronization Status)");
    Serial.println ("(16-bit) I2S->SYNCBUSY: 0x" + String(I2S->SYNCBUSY.reg,    HEX) + "\t, 0b" + String(I2S->SYNCBUSY.reg,        BIN));
    Serial.println ("\t [15:10] Reserved");
    Serial.println ("\t [9]     RXDATA:  0x" + String(I2S->SYNCBUSY.bit.RXDATA, HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.RXDATA, BIN) + "\t Rx Data Synchronization Status");
    Serial.println ("\t [8]     TXDATA:  0x" + String(I2S->SYNCBUSY.bit.TXDATA, HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.TXDATA, BIN) + "\t Tx Data Synchronization Status");
    Serial.println ("\t [7:6]   Reserved");
    Serial.println ("\t [5]     RXEN:    0x" + String(I2S->SYNCBUSY.bit.RXEN,   HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.RXEN,   BIN) + "\t Rx Serializer Enable Synchronization Status");
    Serial.println ("\t [4]     TXEN:    0x" + String(I2S->SYNCBUSY.bit.TXEN,   HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.TXEN,   BIN) + "\t Tx Serializer Enable Synchronization Status");
    Serial.println ("\t [3]     CKEN1:   0x" + String(I2S->SYNCBUSY.bit.CKEN1,  HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.CKEN1,  BIN) + "\t Clock Unit 1 Enable Synchronization Status");
    Serial.println ("\t [2]     CKEN0:   0x" + String(I2S->SYNCBUSY.bit.CKEN0,  HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.CKEN0,  BIN) + "\t Clock Unit 0 Enable Synchronization Status");
    Serial.println ("\t [1]     ENABLE:  0x" + String(I2S->SYNCBUSY.bit.ENABLE, HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.ENABLE, BIN) + "\t Enable Synchronization Status");
    Serial.println ("\t [0]     SWRST:   0x" + String(I2S->SYNCBUSY.bit.SWRST,  HEX) + "\t, 0b" + String(I2S->SYNCBUSY.bit.SWRST,  BIN) + "\t Software Reset Synchronization Status");
    Serial.println ();

    // I2S_TXCTRL
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Tx Serializer Control)");
    Serial.println ("(32-bit) I2S->TXCTRL: 0x" + String(I2S->TXCTRL.reg,            HEX) + "\t, 0b" + String(I2S->TXCTRL.reg,           BIN));
    Serial.println ("\t [31:26] Reserved");
    Serial.println ("\t [25]    DMA:        0x" + String(I2S->TXCTRL.bit.DMA,       HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.DMA,       BIN) + "\t Single or Multiple DMA Channels");
    Serial.println ("\t [24]    MONO:       0x" + String(I2S->TXCTRL.bit.MONO,      HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.MONO,      BIN) + "\t Mono Mode");
    Serial.println ("\t [23:16] SLOTDIS:    0x" + String(I2S->TXCTRL.vec.SLOTDIS,   HEX) + "\t, 0b" + String(I2S->TXCTRL.vec.SLOTDIS,   BIN) + "\t Slot 0:7 Disabled for this Serializer");
    Serial.println ("\t [15]    BITREV:     0x" + String(I2S->TXCTRL.bit.BITREV,    HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.BITREV,    BIN) + "\t Data Formatting Bit Reverse");
    Serial.println ("\t [14:13] EXTEND:     0x" + String(I2S->TXCTRL.bit.EXTEND,    HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.EXTEND,    BIN) + "\t Data Formatting Bit Extension");
    Serial.println ("\t [12]    WORDADJ:    0x" + String(I2S->TXCTRL.bit.WORDADJ,   HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.WORDADJ,   BIN) + "\t Data Word Formatting Adjust");
    Serial.println ("\t [11]    Reserved");
    Serial.println ("\t [10:8]  DATASIZE:   0x" + String(I2S->TXCTRL.bit.DATASIZE,  HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.DATASIZE,  BIN) + "\t Data Word Size");
    Serial.println ("\t [7]     SLOTADJ:    0x" + String(I2S->TXCTRL.bit.SLOTADJ,   HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.SLOTADJ,   BIN) + "\t Data Slot Formatting Adjust");
    Serial.println ("\t [6]     Reserved");
    Serial.println ("\t [5]     Reserved (Datasheet errata says: CLKSEL)");
    Serial.println ("\t [4]     TXSAME:     0x" + String(I2S->TXCTRL.bit.TXSAME,    HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.TXSAME,    BIN) + "\t Transmit Data when Underrun");
    Serial.println ("\t [3:2]   TXDEFAULT:  0x" + String(I2S->TXCTRL.bit.TXDEFAULT, HEX) + "\t, 0b" + String(I2S->TXCTRL.bit.TXDEFAULT, BIN) + "\t Line Default Line when Slot Disabled");
    Serial.println ("\t [1:0]   Reserved (Datasheet errata says: SERMODE)"); 
    Serial.println ();

    // I2S_RXCTRL
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Rx Serializer Control)");
    Serial.println ("(32-bit) I2S->RXCTRL: 0x" + String(I2S->RXCTRL.reg,            HEX) + "\t, 0b" + String(I2S->RXCTRL.reg,           BIN));
    Serial.println ("\t [31:27] Reserved");
    Serial.println ("\t [25]    RXLOOP:     0x" + String(I2S->RXCTRL.bit.RXLOOP,    HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.RXLOOP,    BIN) + "\t Loop-back Test Mode");
    Serial.println ("\t [25]    DMA:        0x" + String(I2S->RXCTRL.bit.DMA,       HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.DMA,       BIN) + "\t Single or Multiple DMA Channels");
    Serial.println ("\t [24]    MONO:       0x" + String(I2S->RXCTRL.bit.MONO,      HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.MONO,      BIN) + "\t Mono Mode");
    Serial.println ("\t [23:16] SLOTDIS:    0x" + String(I2S->RXCTRL.vec.SLOTDIS,   HEX) + "\t, 0b" + String(I2S->RXCTRL.vec.SLOTDIS,   BIN) + "\t Slot 0:7 Disabled for this Serializer");
    Serial.println ("\t [15]    BITREV:     0x" + String(I2S->RXCTRL.bit.BITREV,    HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.BITREV,    BIN) + "\t Data Formatting Bit Reverse");
    Serial.println ("\t [14:13] EXTEND:     0x" + String(I2S->RXCTRL.bit.EXTEND,    HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.EXTEND,    BIN) + "\t Data Formatting Bit Extension");
    Serial.println ("\t [12]    WORDADJ:    0x" + String(I2S->RXCTRL.bit.WORDADJ,   HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.WORDADJ,   BIN) + "\t Data Word Formatting Adjust");
    Serial.println ("\t [11]    Reserved");
    Serial.println ("\t [10:8]  DATASIZE:   0x" + String(I2S->RXCTRL.bit.DATASIZE,  HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.DATASIZE,  BIN) + "\t Data Word Size");
    Serial.println ("\t [7]     SLOTADJ:    0x" + String(I2S->RXCTRL.bit.SLOTADJ,   HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.SLOTADJ,   BIN) + "\t Data Slot Formatting Adjust");
    Serial.println ("\t [6]     Reserved");
    Serial.println ("\t [5]     CLKSEL:     0x" + String(I2S->RXCTRL.bit.CLKSEL,    HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.CLKSEL,    BIN) + "\t Clock Unit Selection");
    Serial.println ("\t [4:2]   Reserved");
    Serial.println ("\t [1:0]   SERMODE:    0x" + String(I2S->RXCTRL.bit.SERMODE,   HEX) + "\t, 0b" + String(I2S->RXCTRL.bit.SERMODE,   BIN) + "\t Serializer Mode");
    Serial.println ();

    // I2S_TXDATA
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Tx Data)");
    Serial.println ("(32-bit) I2S->TXDATA: 0x" + String(I2S->TXDATA.reg, HEX) + "\t, 0b" + String(I2S->TXDATA.reg, BIN) + "\t ");
    Serial.println ();

    // I2S_RXDATA
    Serial.println ("SAMD51_I2S Inter-IC Sound Interface (Rx Data)");
    Serial.println ("(32-bit) I2S->RXDATA: 0x" + String(I2S->RXDATA.reg, HEX) + "\t, 0b" + String(I2S->RXDATA.reg, BIN) + "\t ");
    Serial.println ();
  }

private:
  volatile I2s &i2s;
};
