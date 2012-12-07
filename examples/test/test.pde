#include <Wire.h>
#include <Adafruit_OV7670.h>

Adafruit_OV7670 ov7670;

#define SIZEX (160)
#define SIZEY (120)

void setup(void) 
{
  uint32_t currentFrequency;
    
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Configuring the OV7670 Camera ...");
  int8_t results = ov7670.begin();
  
  if (results)
  {
    Serial.println("Unable to configure the OV7670 (unexpected ProdID)");
	// Wait around forever
	while(1);
  }
  else
  {
    Serial.println("Found OV7670");
  }
  
  // Capture a single frame
  ov7670.capture();
  
  // Wait until the capture is done
  while(ov7670.captureDone() == false);

  // Output the results to the display
  Serial.println("Starting buffer dump");
  ov7670.bufferStart();
   
  for (int y = 0; y < SIZEY; y++) {
	int r,g,b,d1,d2 ;
        Serial.print(y, DEC);
        Serial.print(" : ");
	for (int x = 0;x < SIZEX;x++) {
	    // Data is in RGB565 format
		d1 = ov7670.bufferReadByte(); // bbbbbggg
		d2 = ov7670.bufferReadByte(); // gggrrrrr
		b = (d1 & 0xF8) >> 3;
		g = ((d2 & 0x07) << 3) | ((d1 & 0xE0) >> 5);
		r = (d2 & 0x1F);
		Serial.print("[");
		Serial.print(r, HEX); Serial.print(".");
		Serial.print(g, HEX); Serial.print(".");
		Serial.print(b, HEX); Serial.print("] ");
	}
	Serial.println("") ;
  }
  ov7670.bufferStop() ;
  Serial.println("Finished dumping image");
}

void loop(void) 
{
  delay(2000);
}