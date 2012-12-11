//OV7670             UNO
//------             ---
//SCL                A5
//SDA                A4
//VSYNC              2   (HW Int 0)
//HREF               3   (HW Int 1)
//WEN                5
//RRST               A0
//OE                 A1
//RCLK               A2
//D0                 6
//D1                 7
//D2                 8
//D3                 9
//D4                 10
//D5                 11
//D6                 12
//D7                 13

//ST7735             UNO
//------             ---
//SCK                13
//MOSI               11
//TFT_CS             4   Must be unique
//D/C                10

// You can use any (4 or) 5 pins
#define sclk 13
#define mosi 11
#define lcdcs   4
#define dc   10
#define rst  -1  // you can also connect this to the Arduino reset
//#define DEBUG  // Leave this uncommented to display debug information as well

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <Adafruit_OV7670.h> // Camera library

// Option 1: use any pins but a little slower
Adafruit_ST7735 tft = Adafruit_ST7735(lcdcs, dc, mosi, sclk, rst);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_ST7735 tft = Adafruit_ST7735(lcdcs, dc, rst);

// Camera (pins are hard coded in the class)
Adafruit_OV7670 ov7670;

#define SIZEX (160)
#define SIZEY (120)

void setLCDPins(void)
{
  // Reconfigure SPI pins for TFT since they're shared
  // by the 8-bit bus on the AL422B
  ov7670.deassertDataBus();  // Deasserts OE on the buffer
  pinMode(sclk, OUTPUT);
  pinMode(mosi, OUTPUT);
  pinMode(dc, OUTPUT);
}

void setOV7670Pins(void)
{
  // Deassert the LCD CS line to avoid problems (high=inactive)
  digitalWrite(lcdcs, 1);
  // Set the pins back to the 8-bit inputs for the AL422B
  ov7670.assertDataBus();
}

void setup(void) 
{
  uint32_t currentFrequency;
    
  Serial.begin(115200);
  Serial.println("Hello!");
  
  Serial.println("Configuring the ST7735 Display ...");
  setLCDPins();

  // Our supplier changed the 1.8" display slightly after Jan 10, 2012
  // so that the alignment of the TFT had to be shifted by a few pixels
  // this just means the init code is slightly different. Check the
  // color of the tab to see which init code to try. If the display is
  // cut off or has extra 'random' pixels on the top & left, try the
  // other option!

  // If your TFT's plastic wrap has a Red Tab, use the following:
  // tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
  // If your TFT's plastic wrap has a Green Tab, use the following:
  tft.initR(INITR_REDTAB); // initialize a ST7735R chip, green tab
  
  tft.fillScreen(ST7735_BLACK);

  Serial.println("Configuring the OV7670 Camera ...");
  // Make sure pins are setup for the camera
  setOV7670Pins();
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
  Serial.println("Capturing next image frame");
  ov7670.capture();
  
  // Wait until the capture is done
  while(ov7670.captureDone() == false);
  Serial.println("Image stored in the buffer");

  // Output the results to the display
  Serial.println("Starting buffer read");
  ov7670.bufferStart();
   ov7670.bufferReadByte();
  for (int y = 0; y < SIZEY; y++) {
    uint8_t r,g,b,d1,d2 ;
    uint16_t rgb565;
    #ifdef DEBUG
      Serial.print(y, DEC); Serial.print(": ");
    #endif
    for (int x = 0;x < SIZEX;x++) {
      setOV7670Pins();
      // Data is in RGB565 format
      d1 = ov7670.bufferReadByte(); // bbbbbggg
      d2 = ov7670.bufferReadByte(); // gggrrrrr
      rgb565 = d1; rgb565 <<=8; rgb565 |= d2;
      setLCDPins();
      tft.drawPixel(y, x, rgb565);
      // tft.drawPixel(y, x, ST7735_RED);
      #ifdef DEBUG
        // Display bitmap data on the screen
        b = (d1 & 0xF8) >> 3;
        g = ((d2 & 0x07) << 3) | ((d1 & 0xE0) >> 5);
        r = (d2 & 0x1F);
        Serial.print("[");
        Serial.print(r, HEX); Serial.print(".");
        Serial.print(g, HEX); Serial.print(".");
        Serial.print(b, HEX);
        Serial.print("] ");
      #endif
    }
    #ifdef DEBUG
      Serial.println("") ;
    #endif
  }
  ov7670.bufferStop() ;
  Serial.println("Finished reading buffered image");
}

void loop(void) 
{
  delay(2000);
}