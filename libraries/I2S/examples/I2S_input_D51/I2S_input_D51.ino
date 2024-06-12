/*
  This example reads audio data from an Invensense's ICS43432 I2S microphone
  breakout board, and prints out the samples to the Serial console. The
  Serial Plotter built into the Arduino IDE can be used to plot the audio
  data (Tools -> Serial Plotter)

  Circuit:
   Arduino/Genuino Zero, MKRZero or MKR1000 board
   ICS43432:
     GND connected GND
     3.3V connected 3.3V (Zero) or VCC (MKR1000, MKRZero)
     WS connected to pin 0 (Zero) or pin 3 (MKR1000, MKRZero)
     CLK connected to pin 1 (Zero) or pin 2 (MKR1000, MKRZero)
     SD connected to pin 9 (Zero) or pin A6 (MKR1000, MKRZero)

  created 17 November 2016
  by Sandeep Mistry
*/


// Used by Adafruit on
// It eventually hangs, so it's recommend to use "ArduinoSound" Library on SAMD21G is discussed here:
//    Dictaphone project - MKRZero hangs indefinitely: https://forum.arduino.cc/index.php?topic=455963.0
//    i2s mic returns 0 or -1: https://forums.adafruit.com/viewtopic.php?f=19&t=115089
// This sketch is NOT compatible with SAMD51J

#include <I2S.h>
//#define Serial SerialUSB
uint8_t _buffer[I2S_BUFFER_SIZE];

void setup() {
  //I2S = I2SClass(I2S_DEVICE, I2S_CLOCK_GENERATOR, PIN_I2S_SDI, PIN_I2S_SCK, PIN_I2S_FS);
  // Open serial communications and wait for port to open:
  // A baud rate of 115200 is used instead of 9600 for a faster data rate
  // on non-native USB ports
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("I2S_input_D51");
  // start I2S at 8 kHz with 32-bits per sample
  if (!I2S.begin(I2S_PHILIPS_MODE, 48000, 32)) { // 48,000,000 / sampleRate must be a multiple of 64
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  I2S.read(_buffer, I2S_BUFFER_SIZE);
}

void loop() 
{

  int32_t sampleLeft, sampleRight = 0;

// Scan & Print samples using I2S DMA double buffer technique
  while (!I2S.available());
  int bytes = I2S.read(_buffer, I2S_BUFFER_SIZE);
  //Serial.println("bytes: " + String(bytes)); // must be 512 bytes
  
  for (int i = 0; i < bytes; i += 8)
  {
    // Get Samples
    memcpy(&sampleLeft, &_buffer[i], 4);
    memcpy(&sampleRight, &_buffer[i+4], 4);

    // convert to 24 bits
    //sampleLeft >>= 8;
    //sampleLeft  &= 0x00ffffff;
    //sampleRight >>= 8;
    //sampleRight  &= 0x00ffffff;
    
    // Get samples another way
    //sampleLeft = 0;
    //sampleLeft |= ((uint32_t)_buffer[i])   << 16; // MSB
    //sampleLeft |= ((uint32_t)_buffer[i+1]) << 8;
    //sampleLeft |= ((uint32_t)_buffer[i+2]) << 0;
    //sampleLeft |= ((uint32_t)_buffer[i+3] << 24); // LSB, commented for 24 bit resolution

    //sampleRight = 0;
    //sampleRight |= ((uint32_t)_buffer[i+4]) << 16; // MSB 
    //sampleRight |= ((uint32_t)_buffer[i+5]) << 8;
    //sampleRight |= ((uint32_t)_buffer[i+6]) << 0;
    //sampleRight |= ((uint32_t)_buffer[i+7] << 24); // LSB

    // print samples
    Serial.print(sampleLeft);
    Serial.print(",");
    Serial.println(sampleRight);
  }

// Scan using one read
  //sampleLeft = I2S.read();    Serial.println(sampleLeft);
  //sampleRight = I2S.read(); Serial.println(sampleRight);
   
  
/*
//Scan using Adafruit Zero I2S Read
    I2S.read(&sampleLeft, &sampleRight);

    Serial.print(sampleLeft); // sampleLeft
    Serial.print(",");
    Serial.println(sampleRight); // sampleRight
  */
 
  /*
    I2S.read(_buffer, I2S_BUFFER_SIZE);
    for (int i=0; i<I2S_BUFFER_SIZE; ++i)
      if (i%4 == 0)
        Serial.print("\t0x");
      else
        Serial.print(_buffer[i], HEX); // sampleLeft
    Serial.println();
  */
  delay(1);
  //delayMicroseconds(125); // 1/SAMPLERATE seconds
}
