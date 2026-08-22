#include <ADC.h>

#ifndef ADC_HAS_D5X_E5X_REGISTERS
static inline void waitAdcSyncRaw() {
  while (ADC->STATUS.bit.SYNCBUSY) {
  }
}

uint16_t readTempRawDirectSync() {
  SYSCTRL->VREF.reg |= SYSCTRL_VREF_TSEN;

  const uint16_t oldCtrlB = ADC->CTRLB.reg;
  const uint8_t oldSampCtrl = ADC->SAMPCTRL.reg;
  const uint8_t oldAvgCtrl = ADC->AVGCTRL.reg;
  const uint8_t oldRefSel = ADC->REFCTRL.bit.REFSEL;
  const uint8_t oldGain = ADC->INPUTCTRL.bit.GAIN;

  ADC->CTRLA.bit.ENABLE = 0;
  waitAdcSyncRaw();

  ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV256;
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x3F);
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_64 | ADC_AVGCTRL_ADJRES(0x4);

  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;
  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS(ADC_INPUTCTRL_MUXPOS_TEMP_Val) |
                       ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_GND_Val);

  waitAdcSyncRaw();

  ADC->CTRLA.bit.ENABLE = 1;
  waitAdcSyncRaw();

  ADC->SWTRIG.bit.START = 1;
  while (!ADC->INTFLAG.bit.RESRDY) {
  }
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  ADC->SWTRIG.bit.START = 1;
  while (!ADC->INTFLAG.bit.RESRDY) {
  }
  const uint16_t value = ADC->RESULT.reg;

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
  ADC->CTRLA.bit.ENABLE = 0;
  waitAdcSyncRaw();

  ADC->CTRLB.reg = oldCtrlB;
  ADC->SAMPCTRL.reg = oldSampCtrl;
  ADC->AVGCTRL.reg = oldAvgCtrl;
  ADC->INPUTCTRL.bit.GAIN = oldGain;
  ADC->REFCTRL.bit.REFSEL = oldRefSel;
  waitAdcSyncRaw();

  return value;
}
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

#ifndef ADC_HAS_D5X_E5X_REGISTERS
  Serial.println("Sync temperature register read example");
#else
  Serial.println("SyncTempRegisterRead is SAMD21-specific (direct ADC register path).");
#endif
}

void loop() {
#ifndef ADC_HAS_D5X_E5X_REGISTERS
  const uint16_t raw = readTempRawDirectSync();
  Serial.print("Raw temp register ADC: ");
  Serial.println(raw);
#else
  Serial.println("Use AsyncChipTemperature example for SAMD5X.");
#endif
  delay(1000);
}
