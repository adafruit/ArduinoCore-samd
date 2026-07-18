/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif


#if defined(ARDUINO_SAMD51_E51) || defined(ARDUINO_SAME53_E54)
uint32_t SystemCoreClock=F_CPU;
#else
/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;
#endif

/*
void calibrateADC()
{
  volatile uint32_t valeur = 0;

  for(int i = 0; i < 5; ++i)
  {
    ADC->SWTRIG.bit.START = 1;
    while( ADC->INTFLAG.bit.RESRDY == 0 || ADC->STATUS.bit.SYNCBUSY == 1 )
    {
      // Waiting for a complete conversion and complete synchronization
    }

    valeur += ADC->RESULT.bit.RESULT;
  }

  valeur = valeur/5;
}*/

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz.
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.
 */
void init( void )
{
  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }
  NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);  /* set Priority for Systick Interrupt (2nd lowest) */

  // Clock PORT for Digital I/O
//  PM->APBBMASK.reg |= PM_APBBMASK_PORT ;
//
//  // Clock EIC for I/O interrupts
//  PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

#if defined(ARDUINO_SAMD51_E51)
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1;
  
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2;
  
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5;
  
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7;

#elif defined(ARDUINO_SAME53_E54)
  MCLK_REGS->MCLK_APBAMASK |= MCLK_APBAMASK_SERCOM0_Msk | MCLK_APBAMASK_SERCOM1_Msk |
                              MCLK_APBAMASK_TC0_Msk | MCLK_APBAMASK_TC1_Msk;
  MCLK_REGS->MCLK_APBBMASK |= MCLK_APBBMASK_SERCOM2_Msk | MCLK_APBBMASK_SERCOM3_Msk |
                              MCLK_APBBMASK_TCC0_Msk | MCLK_APBBMASK_TCC1_Msk |
                              MCLK_APBBMASK_TC2_Msk | MCLK_APBBMASK_TC3_Msk;
  MCLK_REGS->MCLK_APBCMASK |= MCLK_APBCMASK_TCC2_Msk | MCLK_APBCMASK_TCC3_Msk |
                              MCLK_APBCMASK_TC4_Msk | MCLK_APBCMASK_TC5_Msk;
  MCLK_REGS->MCLK_APBDMASK |= MCLK_APBDMASK_DAC_Msk | MCLK_APBDMASK_SERCOM4_Msk |
                              MCLK_APBDMASK_SERCOM5_Msk | MCLK_APBDMASK_ADC0_Msk |
                              MCLK_APBDMASK_ADC1_Msk | MCLK_APBDMASK_TCC4_Msk |
                              MCLK_APBDMASK_TC6_Msk | MCLK_APBDMASK_TC7_Msk |
                              MCLK_APBDMASK_SERCOM6_Msk | MCLK_APBDMASK_SERCOM7_Msk;

#else
  // Clock SERCOM for Serial
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;

  // Clock TC/TCC for Pulse and Analog
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7;

  // ATSAMR, for example, doesn't have a DAC
  #ifdef PM_APBCMASK_DAC
   // Clock ADC/DAC for Analog
   PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
  #endif
#endif

/* 
  Commented out to leave pins in default tri-state.  This is
  aimed at avoiding power consumption in DeepSleep.
  
  // Setup all pins (digital and analog) in INPUT mode (default is nothing)
  for (uint32_t ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
  {
    pinMode( ul, INPUT ) ;
  }
*/

  // Initialize Analog Controller
  // Setting clock
#if defined(ARDUINO_SAMD51_E51)
  //set to 1/(1/(48000000/32) * 6) = 250000 SPS
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
	GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
	Adc *adcs[] = {ADC0, ADC1};
		for(int i=0; i<2; i++){

		adcs[i]->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;
		adcs[i]->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync

		adcs[i]->SAMPCTRL.reg = 5;                        // sampling Time Length

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_SAMPCTRL );  //wait for sync

		adcs[i]->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL );  //wait for sync

		// Averaging (see datasheet table in AVGCTRL register description)
		adcs[i]->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
							ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );  //wait for sync
	}

	analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)
	
	GCLK->PCHCTRL[DAC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 4 (12mhz)
	while (GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN == 0);
	
	while ( DAC->SYNCBUSY.bit.SWRST == 1 ); // Wait for synchronization of registers between the clock domains
	DAC->CTRLA.bit.SWRST = 1;
	while ( DAC->SYNCBUSY.bit.SWRST == 1 ); // Wait for synchronization of registers between the clock domains
	
	DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU; // TODO: fix this once silicon bug is fixed
	
	//set refresh rates
	DAC->DACCTRL[0].bit.REFRESH = 2;
	DAC->DACCTRL[1].bit.REFRESH = 2;

#elif defined(ARDUINO_SAME53_E54)
  GCLK_REGS->GCLK_PCHCTRL[ADC0_GCLK_ID] = GCLK_PCHCTRL_GEN_GCLK1 |
                                                 GCLK_PCHCTRL_CHEN_Msk;
  GCLK_REGS->GCLK_PCHCTRL[ADC1_GCLK_ID] = GCLK_PCHCTRL_GEN_GCLK1 |
                                                 GCLK_PCHCTRL_CHEN_Msk;

  adc_registers_t *adcs[] = {ADC0_REGS, ADC1_REGS};
  for (uint32_t i = 0; i < 2; i++) {
    adcs[i]->ADC_CTRLA = (adcs[i]->ADC_CTRLA & ~ADC_CTRLA_PRESCALER_Msk) |
                         ADC_CTRLA_PRESCALER_DIV32;
    adcs[i]->ADC_CTRLB = (adcs[i]->ADC_CTRLB & ~ADC_CTRLB_RESSEL_Msk) |
                         ADC_CTRLB_RESSEL_10BIT;
    while (adcs[i]->ADC_SYNCBUSY & ADC_SYNCBUSY_CTRLB_Msk);

    adcs[i]->ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(5);
    while (adcs[i]->ADC_SYNCBUSY & ADC_SYNCBUSY_SAMPCTRL_Msk);

    adcs[i]->ADC_INPUTCTRL = ADC_INPUTCTRL_MUXNEG_GND;
    while (adcs[i]->ADC_SYNCBUSY & ADC_SYNCBUSY_INPUTCTRL_Msk);

    adcs[i]->ADC_AVGCTRL = ADC_AVGCTRL_SAMPLENUM_1 | ADC_AVGCTRL_ADJRES(0);
    while (adcs[i]->ADC_SYNCBUSY & ADC_SYNCBUSY_AVGCTRL_Msk);
  }

  analogReference(AR_DEFAULT);

  GCLK_REGS->GCLK_PCHCTRL[DAC_GCLK_ID] = GCLK_PCHCTRL_GEN_GCLK4 |
                                                GCLK_PCHCTRL_CHEN_Msk;
  while ((GCLK_REGS->GCLK_PCHCTRL[DAC_GCLK_ID] & GCLK_PCHCTRL_CHEN_Msk) == 0);

  while (DAC_REGS->DAC_SYNCBUSY & DAC_SYNCBUSY_SWRST_Msk);
  DAC_REGS->DAC_CTRLA |= DAC_CTRLA_SWRST_Msk;
  while (DAC_REGS->DAC_SYNCBUSY & DAC_SYNCBUSY_SWRST_Msk);

  DAC_REGS->DAC_CTRLB = DAC_CTRLB_REFSEL_VREFPU;
  DAC_REGS->DAC_DACCTRL[0] =
      (DAC_REGS->DAC_DACCTRL[0] & ~DAC_DACCTRL_REFRESH_Msk) | DAC_DACCTRL_REFRESH(2);
  DAC_REGS->DAC_DACCTRL[1] =
      (DAC_REGS->DAC_DACCTRL[1] & ~DAC_DACCTRL_REFRESH_Msk) | DAC_DACCTRL_REFRESH(2);

#else
  //set to 1/(1/(48000000/32) * 6) = 250000 SPS

  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |    // Divide Clock by 32.
                   ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 5;                        // Sampling Time Length

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)

  // Initialize DAC
  // Setting clock
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_DAC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

 // ATSAMR, for example, doesn't have a DAC
 #ifdef DAC
  while ( DAC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization of registers between the clock domains
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | // Using the 3.3V reference
                   DAC_CTRLB_EOEN ;        // External Output Enable (Vout)
 #endif


#endif //SAMD51
}

#ifdef __cplusplus
}
#endif
