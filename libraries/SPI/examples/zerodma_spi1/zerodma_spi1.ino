// DMA-based SPI buffer write.
// Will setup a dma transaction to read and write data over the spi
// Callback will be called when the dma is completed

#include <SPI.h>
#include <Math.h>
//#define PRINT_MEMMORY_BUFFERS

#define SS	2

// The memory we'll be issuing to SPI:
#define DATA_LENGTH 4096
uint8_t send_memory[DATA_LENGTH];
uint8_t receive_memory[DATA_LENGTH];

//******************************************************************************************

// Show contents of array
void dump(uint8_t *memory)
{
  const int BASE 		= 16; //hex formatting
  const int PAD_AMMOUNT = 4;  //digits

  for(uint32_t i=0; i<DATA_LENGTH; i++)
  {
    // pas numbers this many digits
    for(int c=1; c<PAD_AMMOUNT; ++c)
    {
    	if( memory[i] < pow(BASE, c) )
    	{
    		// pad with spaces
    		Serial.print(" ");
    	}
    }

    // print the number
    Serial.print(memory[i], BASE);

    // new line when about to wrap around in base
    if ((i % BASE) == BASE-1)
    {
    	Serial.println();
    }
  }
  Serial.println();
}

//******************************************************************************************

// Callback for end-of-DMA-transfer
volatile bool dmaDone = false;
void callback_dmaDone()
{
	//Serial.println("Callback: Dma Done");
	dmaDone = true;
}

//******************************************************************************************

void setup()
{
	pinMode(SS, OUTPUT);
	digitalWrite(SS, HIGH); //high is spi deaserted

	Serial.begin(115200);
	//while(!Serial);                 // Wait for Serial monitor before continuing
	Serial.println("***********************");
	Serial.println("Program Start");
	Serial.println("***********************");
	delay(1000);

	// initialize spi
	SPI.begin();
	//*************

	Serial.println("Initing buffers");
	// init the data buffers
	for(uint32_t i=0; i<DATA_LENGTH; i++)
	{
	  send_memory[i] 	= 0x33;
	  receive_memory[i] = 0x55;
	}
	#ifdef PRINT_MEMMORY_BUFFERS
		Serial.println("send_memory: ");
		dump(send_memory);
		Serial.println("destination_memory: ");
		dump(receive_memory);
		delay(1000);
	#endif

}

//******************************************************************************************

void loop()
{
	Serial.println("***********************");
	Serial.println("Starting transfer ...");

	// enable Slave Select
	SPI.beginTransaction(SPISettings(25000000, MSBFIRST, SPI_MODE1));
	digitalWrite(SS, LOW);

	// send the command byte
	//SPI.transfer(255);

	#define DMA_TEST 3 //select the type of dma to test
	#if(DMA_TEST == 1)
		// asyncronous dma transfer
		// reset the transaction flag
		dmaDone = false;

		// calls the dma transfer, this call does not block
		// will call the callback frunction when the dma is completed
		SPI.transfer(send_memory, receive_memory, DATA_LENGTH, callback_dmaDone); //dma

		// can do other things here...
		// can do rtos things like block/yield rtos task and schedule others
		// can do rtos things like block on a dma semaphores here
		
		// wait here until transfer is completed
		while(!dmaDone); // this is updated by our callback function

	#elif(DMA_TEST == 2)
		// dma transfer pole for completion
		// calls the dma transfer, this call will block
		SPI.transfer(send_memory, receive_memory, DATA_LENGTH, true); //dma

	#elif(DMA_TEST == 3)
		// dma transfer pole for completion
		// calls the dma transfer, this call will not block
		SPI.transfer(send_memory, receive_memory, DATA_LENGTH, false); //dma

		// can do other things here...

		// wait here until transfer is completed
		SPI.waitForTransfer();

	#elif(DMA_TEST == 4)
		//non dma transfer
		SPI.transfer(send_memory, receive_memory, DATA_LENGTH);

	#endif

	// disable Slave Select
	digitalWrite(SS, HIGH);
	SPI.endTransaction();
	Serial.println("Done! ");

	//*************

	#ifdef PRINT_MEMMORY_BUFFERS
		Serial.println("send_memory: ");
		dump(send_memory);
		Serial.println("receive_memory: ");
		dump(receive_memory);
		delay(1000);
	#endif

	delay(100);
}
