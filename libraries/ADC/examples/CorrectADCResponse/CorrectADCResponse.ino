/*
  ADC correction calibration example using the ADC library.

  This follows the SAMD_AnalogCorrection workflow, but applies values through
  ChannelADC::setCalibration() instead of analogReadCorrection().

  Wiring:
  - A1 -> GND
  - A2 -> 3.3V
*/

#include <ADC.h>

#define ADC_GND_PIN A1
#define ADC_3V3_PIN A2

#define ADC_READS_SHIFT 8
#define ADC_READS_COUNT (1 << ADC_READS_SHIFT)

#define ADC_MIN_GAIN 0x0400
#define ADC_UNITY_GAIN 0x0800
#define ADC_MAX_GAIN (0x1000 - 1)
#define ADC_RESOLUTION_BITS 12
#define ADC_RANGE (1 << ADC_RESOLUTION_BITS)
#define ADC_TOP_VALUE (ADC_RANGE - 1)

#define MAX_TOP_VALUE_READS 10

ChannelADC gndChannel;
ChannelADC vccChannel;

static uint16_t readChannelAverage(ChannelADC &channel) {
  uint32_t accumulator = 0;

  for (int i = 0; i < ADC_READS_COUNT; ++i) {
    if (channel.read()) {
      delayMicroseconds(200);
      accumulator += channel.value();
    }
  }

  return static_cast<uint16_t>(accumulator >> ADC_READS_SHIFT);
}

static uint16_t readGndLevel() {
  const uint16_t value = readChannelAverage(gndChannel);
  Serial.print("ADC(GND) = ");
  Serial.println(value);
  return value;
}

static uint16_t read3V3Level() {
  uint16_t value = readChannelAverage(vccChannel);

  if (value < (ADC_RANGE >> 1)) {
    value += ADC_RANGE;
  }

  Serial.print("ADC(3.3V) = ");
  Serial.println(value);
  return value;
}

static void applyCalibration(int offset, uint16_t gain) {
  gndChannel.setCalibration(gain, static_cast<int16_t>(offset), true);
  vccChannel.setCalibration(gain, static_cast<int16_t>(offset), true);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!gndChannel.attach(ADC_GND_PIN)) {
    Serial.println("Failed to attach GND channel");
    return;
  }

  if (!vccChannel.attach(ADC_3V3_PIN)) {
    Serial.println("Failed to attach 3.3V channel");
    return;
  }

  gndChannel.setCtrlB(AdcResSel::ADC_RESSEL_12BIT, AdcPrescaler::ADC_PRESCALER_DIV32);
  vccChannel.setCtrlB(AdcResSel::ADC_RESSEL_12BIT, AdcPrescaler::ADC_PRESCALER_DIV32);

  Serial.println("\nCalibrating ADC with default correction values");
  Serial.println("\nReading GND and 3.3V ADC levels");
  Serial.print("   ");
  readGndLevel();
  Serial.print("   ");
  read3V3Level();

  int offsetCorrectionValue = 0;
  uint16_t gainCorrectionValue = ADC_UNITY_GAIN;

  Serial.print("\nOffset correction (@gain = ");
  Serial.print(gainCorrectionValue);
  Serial.println(" unity)");

  applyCalibration(offsetCorrectionValue, gainCorrectionValue);

  for (int offset = 0; offset < 2048; ++offset) {
    applyCalibration(offset, gainCorrectionValue);

    Serial.print("   Offset = ");
    Serial.print(offset);
    Serial.print(", ");

    if (readGndLevel() == 0) {
      offsetCorrectionValue = offset;
      break;
    }
  }

  Serial.println("\nGain correction");

  uint8_t topValueReadsCount = 0;
  uint16_t minGain = 0;
  uint16_t maxGain = 0;

  applyCalibration(offsetCorrectionValue, gainCorrectionValue);
  Serial.print("   Gain = ");
  Serial.print(gainCorrectionValue);
  Serial.print(", ");
  uint16_t highLevelRead = read3V3Level();

  if (highLevelRead < ADC_TOP_VALUE) {
    for (uint16_t gain = ADC_UNITY_GAIN + 1; gain <= ADC_MAX_GAIN; ++gain) {
      applyCalibration(offsetCorrectionValue, gain);

      Serial.print("   Gain = ");
      Serial.print(gain);
      Serial.print(", ");
      highLevelRead = read3V3Level();

      if (highLevelRead == ADC_TOP_VALUE) {
        if (minGain == 0)
          minGain = gain;

        if (++topValueReadsCount >= MAX_TOP_VALUE_READS) {
          maxGain = minGain;
          break;
        }

        maxGain = gain;
      }

      if (highLevelRead > ADC_TOP_VALUE)
        break;
    }
  } else {
    if (highLevelRead == ADC_TOP_VALUE)
      maxGain = ADC_UNITY_GAIN;

    for (int gain = ADC_UNITY_GAIN - 1; gain >= ADC_MIN_GAIN; --gain) {
      applyCalibration(offsetCorrectionValue, gain);

      Serial.print("   Gain = ");
      Serial.print(gain);
      Serial.print(", ");
      highLevelRead = read3V3Level();

      if (highLevelRead == ADC_TOP_VALUE) {
        if (maxGain == 0)
          maxGain = gain;

        minGain = gain;
      }

      if (highLevelRead < ADC_TOP_VALUE)
        break;
    }
  }

  gainCorrectionValue = (minGain + maxGain) >> 1;
  applyCalibration(offsetCorrectionValue, gainCorrectionValue);

  Serial.println("\nReadings after corrections");
  Serial.print("   ");
  readGndLevel();
  Serial.print("   ");
  read3V3Level();

  Serial.println("\n==================");
  Serial.println("\nCorrection values:");
  Serial.print("   Offset = ");
  Serial.println(offsetCorrectionValue);
  Serial.print("   Gain = ");
  Serial.println(gainCorrectionValue);
  Serial.println("\nApply in your sketch:");
  Serial.print("   channel.setCalibration(");
  Serial.print(gainCorrectionValue);
  Serial.print(", ");
  Serial.print(offsetCorrectionValue);
  Serial.println(", true);");
  Serial.println("\n==================");
}

void loop() {
}
