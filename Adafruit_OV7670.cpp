/**************************************************************************/
/*! 
    @file     Adafruit_OV7670.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "Adafruit_OV7670.h"

/*=========================================================================
    PIN LAYOUT
	
    These pins should not be modified since several of them are HW
	dependent. This leaves A3 free for the TFT CS pin and 4 for the SD
	card CS pin, and pins 11, 12 and 13 are reserved for HW SPI. A5 and A4
	are used for HW I2C.
    -----------------------------------------------------------------------*/
    #define _OV7670_VSYNC (2)  // HW Int 0
    #define _OV7670_HREF  (3)  // HW Int 1
    #define _AL422B_WEN   (5)
    #define _AL422B_RRST  (A0)
    #define _AL422B_OE    (A1)
    #define _AL422B_RCLK  (A2)
    #define _AL422B_D0    (6)
    #define _AL422B_D1    (7)
    #define _AL422B_D2    (8)
    #define _AL422B_D3    (9)
    #define _AL422B_D4    (10)
    #define _AL422B_D5    (11)
    #define _AL422B_D6    (12)
    #define _AL422B_D7    (13)
/*=========================================================================*/

// Line counters to know how many lines of data have been passed in
volatile uint16_t _lineCounter, _lastLine;

// Flags to know if we're busy capturing a frame, etc.
volatile bool _captureRequested, _busy, _done;

/**************************************************************************/
/*! 
    @brief  VSYNC Interrupt Handler
*/
/**************************************************************************/
void vsync_handler() {
  if (_captureRequested) 
  {
    digitalWrite(_AL422B_WEN, 1);
	_done = false ;
	_captureRequested = false ;
  } 
  else 
  {
    digitalWrite(_AL422B_WEN, 0);
	if (_busy)
	{
      _busy = false ;
      _done = true ;
	}
  }
		
  // Reset line counter
  _lastLine = _lineCounter;
  _lineCounter = 0;
}

/**************************************************************************/
/*! 
    @brief  HREF Interrupt Handler
*/
/**************************************************************************/
void href_handler() {
  // Increment line counter
  _lineCounter++;
}

/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void Adafruit_OV7670::wireWriteRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(ov7670_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
    Wire.write(value);
  #else
    Wire.send(reg);                        // Register
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Reads an 8-bit value over I2C
*/
/**************************************************************************/
void Adafruit_OV7670::wireReadRegister(uint8_t reg, uint8_t *value)
{

  Wire.beginTransmission(ov7670_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
  #else
    Wire.send(reg);                        // Register
  #endif
  Wire.endTransmission();  

  Wire.requestFrom(ov7670_i2caddr, (uint8_t)1);  
  #if ARDUINO >= 100
    // Shift values to create properly formed integer
    *value = Wire.read();
  #else
    // Shift values to create properly formed integer
    *value = Wire.receive();
  #endif
}

/**************************************************************************/
/*! 
    @brief  Instantiates a new OV7670 class
*/
/**************************************************************************/
Adafruit_OV7670::Adafruit_OV7670(uint8_t addr) {
  ov7670_i2caddr = addr;  

  // Setup pin states
  pinMode(_OV7670_VSYNC, INPUT);
  pinMode(_OV7670_HREF,  INPUT);
  pinMode(_AL422B_WEN,   OUTPUT);
  pinMode(_AL422B_RRST,  OUTPUT);
  pinMode(_AL422B_OE,    OUTPUT);
  pinMode(_AL422B_RCLK,  OUTPUT);
  // Setup the data bus pins
  assertDataBus();
  
  // Reset internal variables
  _lineCounter = _lastLine = 0;
  _captureRequested = false;
  _busy = false;
  _done = false;
  
  // Set pins to known state
  digitalWrite(_AL422B_RRST, 1);
  digitalWrite(_AL422B_OE, 1);
  digitalWrite(_AL422B_RCLK, 1);
  digitalWrite(_AL422B_WEN, 0);
}

/**************************************************************************/
/*! 
    @brief  Sets up the OV7670 for 160x120 RGB565 video at 30fps
*/
/**************************************************************************/
int8_t Adafruit_OV7670::begin() {
  uint8_t value;
  
  Wire.begin();

  // Check product ID  
  wireReadRegister(OV7670_REG_PID, &value);
  if (value != 0x76)
  {
    Serial.print("Read PID "); Serial.println(value, HEX);
    //return -1;
  }
  
  // Check version ID
  wireReadRegister(OV7670_REG_VER, &value);
  if (value != 0x70)
  {
    Serial.print("Read VER "); Serial.println(value, HEX);
    //return -1;
  }
  
  // Put the camera registers in a known state
  reset();
  
  // Initialise the OV7670 for 160x120 video and RGB565
  wireWriteRegister(OV7670_REG_CLKRC,       0x80); // Reserved?
  wireWriteRegister(OV7670_REG_COM11,       0x0A); // Banding Filter (50Hz)
  wireWriteRegister(OV7670_REG_TSLB,        0x04); // Reserved?
  wireWriteRegister(OV7670_REG_COM7,        0x04); // RGB Output (can also be YUV or Bayer RAW)
  wireWriteRegister(OV7670_REG_RGB444,      0x00); // Don't use RGB444
  wireWriteRegister(OV7670_REG_COM15,       0xd0); // Output Range 00..FF, RGB565
  wireWriteRegister(OV7670_REG_HSTART,      0x16); // HREF Start Bit
  wireWriteRegister(OV7670_REG_HSTOP,       0x04); // HREF Stop Bit
  wireWriteRegister(OV7670_REG_HREF,        0x24); // HREF 3 LSBs
  wireWriteRegister(OV7670_REG_VSTART,      0x02); // Vertical Frame Start
  wireWriteRegister(OV7670_REG_VSTOP,       0x7a); // Vertical Frame End
  wireWriteRegister(OV7670_REG_VREF,        0x0a); // Vertical Frame 2 LSBs
  wireWriteRegister(OV7670_REG_COM10,       0x02); // VSYNC Negative (must be inverted on V2 modules!)
  wireWriteRegister(OV7670_REG_COM3,        0x04); // DCW Enable (pre-defined output format via COM7)
  wireWriteRegister(OV7670_REG_COM14,       0x1a); // Divide PCLK by 4, Manual Scaling Adj
  wireWriteRegister(OV7670_REG_SCALINGDCW,  0x22); // Horizontal divide by 4, Vertical divide by 4
  wireWriteRegister(OV7670_REG_SCALINGPCLK, 0xf2); // Divide by 4, Enable clock divider

  // color bars!
  //wireWriteRegister(OV7670_REG_COM7,        0x06);  // enable color bar
  //wireWriteRegister(OV7670_REG_COM17, 0x00);            // DSP color bars
  // wireWriteRegister(OV7670_REG_SCALING_XSC, 0x3A | 0x80);      // select test output
  //wireWriteRegister(OV7670_REG_SCALING_YSC, 0x35);


  // Color Settings
  wireWriteRegister(0x4f, 0x80);
  wireWriteRegister(0x50, 0x80);
  wireWriteRegister(0x51, 0x00);
  wireWriteRegister(0x52, 0x22);
  wireWriteRegister(0x53, 0x5e);
  wireWriteRegister(0x54, 0x80);
  wireWriteRegister(0x56, 0x40);
  wireWriteRegister(0x58, 0x9e);
  wireWriteRegister(0x59, 0x88);
  wireWriteRegister(0x5a, 0x88);
  wireWriteRegister(0x5b, 0x44);
  wireWriteRegister(0x5c, 0x67);
  wireWriteRegister(0x5d, 0x49);
  wireWriteRegister(0x5e, 0x0e);
  wireWriteRegister(0x69, 0x00);
  wireWriteRegister(0x6a, 0x40);
  wireWriteRegister(0x6b, 0x0a);
  wireWriteRegister(0x6c, 0x0a);
  wireWriteRegister(0x6d, 0x55);
  wireWriteRegister(0x6e, 0x11);
  wireWriteRegister(0x6f, 0x9f);

  wireWriteRegister(0xB0, 0x84);  // Reserved?
  
  // Setup HW interrupts for VSYNC and HREF (must be on pins 2 and 3)
  attachInterrupt(0, vsync_handler, FALLING); // Pin 2 on UNO = INT0
  attachInterrupt(1, href_handler, RISING);   // Pin 3 on UNO = INT1
  
  return 0;
}

/**************************************************************************/
/*! 
    @brief  Performs a SW reset on the OV7670 camera
*/
/**************************************************************************/
void Adafruit_OV7670::reset() {
  // SCCB Register Reset
  wireWriteRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
  delay(200);
}

/**************************************************************************/
/*! 
    @brief  Will start a capture on the next VSYNC interrupt
*/
/**************************************************************************/
void Adafruit_OV7670::capture() {
  _captureRequested = true ;
  _busy = true ;
}

/**************************************************************************/
/*! 
    @brief  Indicates if the last capture request is complete
*/
/**************************************************************************/
bool Adafruit_OV7670::captureDone() {
  bool result;
  
  if (_busy)
  {
    result = false;
  }
  else
  {
    result = _done;
	_done = false;
  }
  
  return result;
}

/**************************************************************************/
/*! 
    @brief  Enables the buffer output and resets the read index to 0
*/
/**************************************************************************/
void Adafruit_OV7670::bufferStart(void) {
  digitalWrite(_AL422B_RRST, 0);
  digitalWrite(_AL422B_OE, 0);
  delay(1);
  digitalWrite(_AL422B_RCLK, 0);
  delay(1);
  digitalWrite(_AL422B_RCLK, 1);
  delay(1);
  digitalWrite(_AL422B_RRST, 1);
}

/**************************************************************************/
/*! 
    @brief  Disables the buffer output
*/
/**************************************************************************/
void Adafruit_OV7670::bufferStop(void) {
  digitalWrite(_AL422B_OE, 1);
  bufferReadByte();
  digitalWrite(_AL422B_RCLK, 1);
}

/**************************************************************************/
/*! 
    @brief  Reads a single byte from the buffer
*/
/**************************************************************************/
uint8_t Adafruit_OV7670::bufferReadByte(void) {
  uint8_t result;
  digitalWrite(_AL422B_RCLK, 1);
  result  = digitalRead (_AL422B_D0);
  result |= digitalRead (_AL422B_D1) << 1;
  result |= digitalRead (_AL422B_D2) << 2;
  result |= digitalRead (_AL422B_D3) << 3;
  result |= digitalRead (_AL422B_D4) << 4;
  result |= digitalRead (_AL422B_D5) << 5;
  result |= digitalRead (_AL422B_D6) << 6;
  result |= digitalRead (_AL422B_D7) << 7;
  digitalWrite(_AL422B_RCLK, 0);
  return result;
}

/**************************************************************************/
/*! 
    @brief  Reconfigures the 8-bit data bus as inputs and re-asserts the
            OE pin on the AL422B Buffer.
*/
/**************************************************************************/
void Adafruit_OV7670::assertDataBus(void) {
  pinMode(_AL422B_D0,    INPUT);
  pinMode(_AL422B_D1,    INPUT);
  pinMode(_AL422B_D2,    INPUT);
  pinMode(_AL422B_D3,    INPUT);
  pinMode(_AL422B_D4,    INPUT);
  pinMode(_AL422B_D5,    INPUT);
  pinMode(_AL422B_D6,    INPUT);
  pinMode(_AL422B_D7,    INPUT);
  // Assert OE pin to enable buffer pins
  digitalWrite(_AL422B_OE, 0);
}

/**************************************************************************/
/*! 
    @brief  Disables the OE pin on the AL422B buffer putting the IC-side
	        pins in a Hi-Z state so that the pins can be reused for SPI
			(SD Cards or an SPI display).
*/
/**************************************************************************/
void Adafruit_OV7670::deassertDataBus(void) {
  // Deassert OE (puts pins in Hi-Z IC-side)
  digitalWrite(_AL422B_OE, 1);
}
