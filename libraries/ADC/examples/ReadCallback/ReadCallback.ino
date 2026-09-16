#include <ADC.h>

ChannelADC adc;
volatile uint16_t latestValue = 0;
volatile bool sampleReady = false;

void onAdcRead(ChannelADC *channel, uint16_t result, void *userData) {
  (void)channel;
  (void)userData;
  latestValue = result;
  sampleReady = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!adc.attach(A0))
    Serial.println("ADC attach failed");
  adc.setReadCallback(onAdcRead, nullptr);
}

void loop() {
  static uint32_t lastTriggerMs = 0;
  const uint32_t now = millis();

  if (now - lastTriggerMs >= 100) {
    lastTriggerMs = now;
    if (!adc.read())
      Serial.println("ADC read enqueue failed");
  }

  if (sampleReady) {
    noInterrupts();
    const uint16_t value = latestValue;
    sampleReady = false;
    interrupts();

    Serial.print("A0: ");
    Serial.println(value);
  }
}
