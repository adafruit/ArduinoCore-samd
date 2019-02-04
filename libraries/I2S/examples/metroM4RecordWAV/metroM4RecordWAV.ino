/*
 This example reads audio data from an Adafruit I2S MEMS Microphone Breakout board 
 (Knowles SPH0645 microphone), and save out the samples to a WAV file.

 Circuit:
 * Adafruit Feather M0/M4, Adafruit Metro M0/M4
 * SPH0645:
   * GND  connected to GND
   * 3.3V connected to 3.3V
   * LRCL connected to pin 0 or 7 (Metro/Feather M0), or pin 9 (Metro/Feather M4) [FS_PIN]
   * BCLK connected to pin 1 or 6 (Metro/Feather M0), or pin 3 (Metro/Feather M4) [SCK_PIN]
   * DOUT connected to pin 9 or 12(Metro/Feather M0), or pin 1 (Metro/Feather M4) [RX_PIN]
 * Micro SD:
   * GND  connected to GND 
   * 3.3V connected to 3.3V
   * MISO connected to MOSI/MO (Metro/Feather M0/M4)
   * MOSI connected to MISO/MI (Metro/Feather M0/M4)
   * SCK  connected to SCK     (Metro/Feather M0/M4)
   * CS   connected to 10      (Metro/Feather M0/M4)

 created 04 February 2019
 by NeKuNeKo by merging @PaulStoffregen, @JarvusChen & @MohitB sketches.
 */

#include <SD.h>
#include <I2S.h>

#define Serial Serial

#undef AUDIO_SAMPLE_RATE
#undef AUDIO_BITS_PER_SAMPLE

#define AUDIO_SAMPLE_RATE     48000
#define AUDIO_BITS_PER_SAMPLE 16


const String fileName = "MyRECORD.WAV";
//write wav
unsigned long ChunkSize = 0L;
unsigned long Subchunk1Size = 16;
unsigned int AudioFormat = 1;
unsigned int numChannels = 2; //1
unsigned long sampleRate = AUDIO_SAMPLE_RATE;
unsigned int bitsPerSample = AUDIO_BITS_PER_SAMPLE; // 16
unsigned long byteRate = sampleRate*numChannels*(bitsPerSample/8);// samplerate x channels x (bitspersample / 8)
unsigned int blockAlign = numChannels*bitsPerSample/8;
unsigned long Subchunk2Size = 0L;
unsigned long recByteSaved = 0L;
unsigned long NumSamples = 0L;
byte byte1, byte2, byte3, byte4;

int mode = 0;  // 0=stopped, 1=recording, 2=playing
File frec;
long  msecs; //elapsedMillis

// Use these with the Teensy 3.5 & 3.6 SD card
#define SDCARD_CS_PIN    10
volatile int available = 0;
volatile int read = 0;
uint8_t buffer[I2S_BUFFER_SIZE];

#define WRITE_BUFFER_SIZE 512
byte bufferToWrite[WRITE_BUFFER_SIZE];
int lenOfBufferToWrite = 0;
long startTime = 0;

void setup() 
{
  Serial.begin(115200);
  while(!Serial);

  Serial.println("metroM4RecordWAV.ino");
  while (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
      Serial.println("Unable to access the SD card");
      delay(500);
  }

  I2S.onReceive(onI2SReceive);

  //int sampleRateForI2S = 31250;  // 48,000,000 / sampleRate must be a multiple of 64
  while (!I2S.begin(I2S_PHILIPS_MODE, AUDIO_SAMPLE_RATE, 32))
  {
      Serial.println("Failed to initialize I2S!");
      delay(500);
  }

/*
  startRecording();
  stopRecording();
  while(true);
  */
}


void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    byte incomingByte = Serial.read();
    // Respond to button presses
    if ( incomingByte == '1' ) {
      Serial.println("Record Button Press");
      if (mode == 2) stopPlaying();
      if (mode == 0) startRecording();
    }
    if ( incomingByte == '2' ) {
      Serial.println("Stop Button Press");
      if (mode == 1) stopRecording();
      if (mode == 2) stopPlaying();
    }
    if ( incomingByte == '3' ) {
      Serial.println("Play Button Press");
      if (mode == 1) stopRecording();
      if (mode == 0) startPlaying();   
    }
  }
  if (mode == 1) {
    continueRecording();
  }

}

void onI2SReceive()
{
    // This function will run at a frequency of (sampleRate / 64)
    // At 31.25khz, this is every 1962.5 microseconds so make sure any processing here takes less
    // time than that about 1.1798, 1817, 1768
    //Serial.println("onI2SReceive called after " + String((micros()-startTime)) + " milliseconds");
    read = I2S.read(buffer, I2S_BUFFER_SIZE);  // Will actually read 256 bytes
    available = 1;
    //startTime = micros();
}

void startRecording() {
  Serial.println("startRecording");
  if (SD.exists(fileName))
    SD.remove(fileName);
  
   frec = SD.open(fileName,  O_CREAT | O_TRUNC | O_RDWR); // if opened as FILE_WRITE cannot seek to another position
  if (frec) {
    I2S.read(); // Needed to start the interrupt handler
    //queue1.begin(); // teensy doble buffer
    mode = 1;
    recByteSaved = 0L;
  }
  startTime = millis();
}




void continueRecording() {

  if (available)//(I2S.available() >= 2)
  {
   /*
// Technique from @MohitB
    for (int i = 0; i < 32; i++)//32, 512/8/2 = 32
    {
      // buffer[0] to buffer[3] are zeros (it is the other channel), so we skip those
      // buffer[4] is always zero
      // buffer[5], [6] and [7] are the interesting data, on 24 bits (6 of which are always zeros)
      // Etc
      // We only want the 256 first values, hence we stope at 8 * 31 + 7 = 255
      // More info on robsgreen's great post: https://forums.adafruit.com/viewtopic.php?f=19&t=115089&start=15#wrap
      memcpy(&bufferToWrite[lenOfBufferToWrite + 3 * i], &buffer[(8 * i) + 5], 3);       //3
    }
      lenOfBufferToWrite += 96;     //96
    
    if (lenOfBufferToWrite == 2016)
    {
      // Write on SD card by batches of 2048 bits for better performance
      frec.write(bufferToWrite, 2016);
      lenOfBufferToWrite = 0;
      recByteSaved += 2016;
      //Serial.println(recByteSaved);
    }
    read = 0;
    */
// NekuNeko technique 1
    // write all 512 bytes to the SD card
    frec.write(buffer, read);
    recByteSaved += read;

    /*
// NekuNeko technique 2
    for (int i=0; i<read; i+=4)
    {
      memcpy(&bufferToWrite[lenOfBufferToWrite], &buffer[i], 2);
      lenOfBufferToWrite += 2;
    }
    //Serial.println("Copying " + String(read)+ " bytes has taken about: " + String((micros()-startTime)) + " microseconds");
      

    if (lenOfBufferToWrite == WRITE_BUFFER_SIZE)
    {
      // startTime = micros();
      // swap MSB & LSB
      for (int i=0; i<=lenOfBufferToWrite-2; i+=2)
      {
        uint8_t aux = bufferToWrite[i];
        bufferToWrite[i] = bufferToWrite[i+1];
        bufferToWrite[i+1] = aux;
      }
      //Serial.println("Swaping " + String(lenOfBufferToWrite)+ " bytes has taken about: " + String((micros()-startTime)) + " microseconds");
      //startTime = micros();
      frec.write(bufferToWrite, lenOfBufferToWrite);
      //Serial.println("Saving " + String(lenOfBufferToWrite)+ " bytes has taken about: " + String((micros()-startTime)) + " microseconds");
      recByteSaved += lenOfBufferToWrite;
      lenOfBufferToWrite = 0;
    }
    */

    
  /*
// another testing...
    uint32_t sampleLeft, sampleRight = 0;
    //uint8_t buffer[I2S_BUFFER_SIZE];
    int bytes = I2S.read(buffer, I2S_BUFFER_SIZE);
    for (int i=0; i<bytes; i+=4)
      buffer[i+3] = 0x00;
    */
    /*
    uint32_t sample = 0;
    memcpy(&sample, &buffer[0], 4);

    if (sample == 0xffffffff) // First value is Right
      for (int i=0, j=4; i<read; ++i, j+=8)
        writeBuffer[i] = buffer[j];
    else // First value is Left
      for (int i=0, j=0; i<read; ++i, j+=8)
        writeBuffer[i] = buffer[j];
*/
/*
    memcpy(writeBuffer, buffer, read);
    for (int i=0; i<read; i+=8)
      writeBuffer[i] = 0;
  */
  /*
    frec.write(buffer, read);
    recByteSaved += read;
    */
    /*
    for (int i=0; i<bytes; i+=8)
    {
       memcpy(&sampleLeft, &buffer[i], 4);
       memcpy(&sampleRight, &buffer[i+4], 4);
    }
    Serial.print(sampleLeft);
    Serial.print(",");
    Serial.println(sampleRight);
*/
    //    elapsedMicros usec = 0;
    //    Serial.print("SD write, us=");
    //    Serial.println(usec);

    available = 0;
  }
  /*
   @JarvusChen teensy technique
  if (queue1.available() >= 2) {
   
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer + 256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    // write all 512 bytes to the SD card
    frec.write(buffer, 512);
    recByteSaved += 512;
//    elapsedMicros usec = 0;
//    Serial.print("SD write, us=");
//    Serial.println(usec);
  }
  */
}

void stopRecording() {
  Serial.println("stopRecording");
  //I2S.end();
  if (mode == 1) {
    /*
    // teensy technique
    while (I2S.available() > 0) {
      int bytes = I2S.read(buffer, I2S_BUFFER_SIZE);
      frec.write((byte*)queue1.readBuffer(), 256);
      queue1.freeBuffer();
      recByteSaved += 256;
    }*/


/*
// Nekuneko technique 2
    if (lenOfBufferToWrite > 0)
    { // swap MSB & LSB
      for (int i=0; i<=lenOfBufferToWrite-2; i+=2)
      {
        uint8_t aux = bufferToWrite[i];
        bufferToWrite[i] = bufferToWrite[i+1];
        bufferToWrite[i+1] = aux;
      }

      frec.write(bufferToWrite, lenOfBufferToWrite);
      recByteSaved += lenOfBufferToWrite;
      lenOfBufferToWrite = 0;
    }
    */
    Serial.println("Recording has taken about: " + String((millis()-startTime)/1000.0) + " seconds");
    writeOutHeader();
    frec.close();
  }
  mode = 0;
}


void startPlaying() {
  Serial.println("startPlaying");
  //audioSD.play("RECORD.WAV"); // teensy
  mode = 2;

}


void stopPlaying() {
  Serial.println("stopPlaying");
  //if (mode == 2) audioSD.stop(); // teensy
  mode = 0;
}
void writeOutHeader() { // update WAV header with final filesize/datasize

//  NumSamples = (recByteSaved*8)/bitsPerSample/numChannels;
//  Subchunk2Size = NumSamples*numChannels*bitsPerSample/8; // number of samples x number of channels x number of bytes per sample
  Subchunk2Size = recByteSaved;
  ChunkSize = Subchunk2Size + 36;
  frec.seek(0);
  delay(10);
  frec.write("RIFF");
  byte1 = ChunkSize & 0xff;
  byte2 = (ChunkSize >> 8) & 0xff;
  byte3 = (ChunkSize >> 16) & 0xff;
  byte4 = (ChunkSize >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.write("WAVE");
  frec.write("fmt ");
  byte1 = Subchunk1Size & 0xff;
  byte2 = (Subchunk1Size >> 8) & 0xff;
  byte3 = (Subchunk1Size >> 16) & 0xff;
  byte4 = (Subchunk1Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = AudioFormat & 0xff;
  byte2 = (AudioFormat >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = numChannels & 0xff;
  byte2 = (numChannels >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = sampleRate & 0xff;
  byte2 = (sampleRate >> 8) & 0xff;
  byte3 = (sampleRate >> 16) & 0xff;
  byte4 = (sampleRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = byteRate & 0xff;
  byte2 = (byteRate >> 8) & 0xff;
  byte3 = (byteRate >> 16) & 0xff;
  byte4 = (byteRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = blockAlign & 0xff;
  byte2 = (blockAlign >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = bitsPerSample & 0xff;
  byte2 = (bitsPerSample >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  frec.write("data");
  byte1 = Subchunk2Size & 0xff;
  byte2 = (Subchunk2Size >> 8) & 0xff;
  byte3 = (Subchunk2Size >> 16) & 0xff;
  byte4 = (Subchunk2Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.close();
  Serial.println("header written"); 
  Serial.print("Subchunk2: "); 
  Serial.println(Subchunk2Size); 
}
