#include <ADC.h>

volatile bool g_readComplete = false;
volatile uint16_t g_lastValue = 0;

void onTempRead(ChannelADC *channel, uint16_t result, void *userData) {
  (void)channel;
  (void)userData;
  g_lastValue = result;
  g_readComplete = true;
}

ChannelADC g_temp;
#ifdef ADC_HAS_D5X_E5X_REGISTERS
ChannelADC g_ctat;
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial);

  g_temp.setReadCallback(onTempRead, nullptr);
#ifdef ADC_HAS_D5X_E5X_REGISTERS
  // On SAMD5x/SAME5x, the PTAT/CTAT sensors are routed by SUPC.VREF.TSSEL
  // with VREFOE disabled. Use a non-SUPC ADC reference for these channels.
  g_temp.setReference(AdcRefSel::ADC_REFSEL_INTVCC1);
#else
  g_temp.setReference(AdcRefSel::ADC_REFSEL_INT1V);
#endif
  g_temp.setCtrlB(AdcResSel::ADC_RESSEL_12BIT, AdcPrescaler::ADC_PRESCALER_DIV32);

#ifdef ADC_HAS_D5X_E5X_REGISTERS
  g_ctat.setReadCallback(onTempRead, nullptr);
  g_ctat.setReference(AdcRefSel::ADC_REFSEL_INTVCC1);
  g_ctat.setCtrlB(AdcResSel::ADC_RESSEL_12BIT, AdcPrescaler::ADC_PRESCALER_DIV32);

  if (!g_temp.attach(AdcMuxPos::ADC_MUXPOS_TEMP, AdcMuxNeg::ADC_MUXNEG_GND,
                     AdcSampleNum::ADC_SAMPLENUM_16)) {
    Serial.println("Attach PTAT failed");
  }

  if (!g_ctat.attach(AdcMuxPos::ADC_MUXPOS_TEMP_CTAT, AdcMuxNeg::ADC_MUXNEG_GND,
                     AdcSampleNum::ADC_SAMPLENUM_16)) {
    Serial.println("Attach CTAT failed");
  }
#else
  if (!g_temp.attach(AdcMuxPos::ADC_MUXPOS_TEMP, AdcMuxNeg::ADC_MUXNEG_GND,
                     AdcSampleNum::ADC_SAMPLENUM_16)) {
    Serial.println("Attach temperature channel failed");
  }
#endif
}

void loop() {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
  g_readComplete = false;
  if (!g_temp.read()) {
    Serial.println("PTAT async enqueue failed");
    delay(1000);
    return;
  }
  while (!g_readComplete) {
    AdcEngine::instance().service();
  }
  const uint16_t ptat = g_lastValue;

  g_readComplete = false;
  if (!g_ctat.read()) {
    Serial.println("CTAT async enqueue failed");
    delay(1000);
    return;
  }
  while (!g_readComplete) {
    AdcEngine::instance().service();
  }
  const uint16_t ctat = g_lastValue;

  const float tempC = analogReadTemperatureC(ptat, ctat);
  Serial.print("PTAT raw: ");
  Serial.print(ptat);
  Serial.print(" CTAT raw: ");
  Serial.print(ctat);
  Serial.print(" ");
  Serial.print("Async chip temp (C): ");
  Serial.println(tempC, 2);
#else
  g_readComplete = false;
  if (!g_temp.read()) {
    Serial.println("Async enqueue failed");
    delay(1000);
    return;
  }
  while (!g_readComplete) {
    AdcEngine::instance().service();
  }

  const uint16_t raw = g_lastValue;
  const float tempC = analogReadTemperatureC(raw);

  Serial.print("Async raw: ");
  Serial.print(raw);
  Serial.print(" tempC: ");
  Serial.print(tempC, 2);
  if (tempC >= 20.0f && tempC <= 24.0f) {
    Serial.println(" (PASS 20-24C)");
  } else {
    Serial.println(" (OUTSIDE 20-24C)");
  }
#endif

  delay(1000);
}
