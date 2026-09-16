#include <ADC.h>

ChannelADC adc;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Attach a single-ended channel on A0.
  if (!adc.attach(A0))
    Serial.println("ADC attach failed");
}

void loop() {
  if (adc.read()) {
    delay(5);
    Serial.print("A0: ");
    Serial.println(adc.value());
  } else {
    Serial.println("ADC read enqueue failed");
  }

  delay(250);
}
