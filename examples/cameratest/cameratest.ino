/*
Example for Adafruit_OV7670 library. With an ILI9341 TFT shield and
OV7670 camera connected to Grand Central, displays a continuous live
feed. That's all, keeping this a fairly minimal example.
US
HARDWARE REQUIRED:
- Adafruit Grand Central board
- Adafruit 2.8" TFT Touch Shield (touch is NOT USED in this example)
- OV7670 camera w/2.2K pullups to SDA+SCL, 3.3V+GND wires to shield
*/

#include <Wire.h>             // I2C comm to camera
#include "Adafruit_OV7670.h"  // Camera library
#include "Adafruit_ILI9341.h" // TFT display library

// CAMERA CONFIG -----------------------------------------------------------

#if defined(__SAMD51__) // Grand Central or other M4 boards
// Set up arch and pins structures for Grand Central's SAMD51.
// PCC data pins are not configurable and do not need specifying.
OV7670_arch arch = {.timer = TCC1, .xclk_pdec = false};
OV7670_pins pins = {.enable = PIN_PCC_D8, .reset = PIN_PCC_D9,
                    .xclk = PIN_PCC_XCLK};
#define CAM_I2C Wire1 // Second I2C bus next to PCC pins
#endif // end __SAMD51__

#define CAM_SIZE OV7670_SIZE_DIV2 // QVGA (320x240 pixels)
#define CAM_MODE OV7670_COLOR_RGB // RGB plz

Adafruit_OV7670 cam(OV7670_ADDR, &pins, &CAM_I2C, &arch);

// SHIELD AND DISPLAY CONFIG -----------------------------------------------

// TFT shield pinout. Switch these if using Feather (DC=10, CS=9)
#define TFT_DC 9
#define TFT_CS 10
#define TFT_SPI SPI

// Screen DMA allows faster updates, but USE_SPI_DMA must be
// manually enabled in Adafruit_SPITFT.h (it's not on by default
// on Grand Central). If DMA is not enabled there, a helper class
// makes screen writes a little bit faster (but not in background
// like with DMA).
#if !defined(USE_SPI_DMA)
#include "SPIBrute.h" // Direct-to-SPI-registers helper class for SAMD51
#endif

Adafruit_ILI9341 tft(&TFT_SPI, TFT_DC, TFT_CS);
#if defined(USE_SPI_BRUTE)
SPIBrute brute(&TFT_SPI);
#endif

// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Hello");

  // 50 MHz to screen is OK if wiring is clean (e.g. shield or FeatherWing).
  // Otherwise (if using jumper wires to screen), stick to 24 MHz.
#if defined(__SAMD51__)
  TFT_SPI.setClockSource(SERCOM_CLOCK_SOURCE_100M);
  tft.begin(50000000);
#else
  tft.begin(24000000);
#endif
  tft.setRotation(3); // Match camera orientation on Grand Central
  tft.fillScreen(ILI9341_BLACK);
#if defined(USE_SPI_BRUTE)
  brute.begin();
#endif

  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  OV7670_status status = cam.begin(CAM_MODE, CAM_SIZE, 30.0);
  if (status != OV7670_STATUS_OK) {
    Serial.println("Camera begin() fail");
    for(;;);
  }

  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

// TFT setAddrWindow() involves a lot of context switching that can slow
// things down a bit, so we don't do it on every frame. Instead, it's only
// set periodically, and we just keep writing data to the same area of the
// screen (it wraps around automatically). We do need an OCCASIONAL
// setAddrWindow() in case SPI glitches, as this syncs things up to a
// known region of the screen again.
#define KEYFRAME 30        // Number of frames between setAddrWindow commands
uint16_t frame = KEYFRAME; // Force 1st frame as keyframe

void loop() {
  // This was for empirically testing window settings in src/arch/ov7670.c.
  // Your code doesn't need this. Just keeping around for future reference.
  if(Serial.available()) {
    uint32_t vstart = Serial.parseInt();
    uint32_t hstart = Serial.parseInt();
    uint32_t edge_offset = Serial.parseInt();
    uint32_t pclk_delay = Serial.parseInt();
    OV7670_frame_control((void *)&cam, CAM_SIZE, vstart, hstart,
                         edge_offset, pclk_delay);
  }

  if (++frame >= KEYFRAME) { // Time to sync up a fresh address window?
    frame = 0;
#if defined(USE_SPI_DMA)
    tft.dmaWait(); // Wait for prior transfer to complete
#elif defined(USE_SPI_BRUTE)
    brute.wait();
#endif
    tft.endWrite();   // Close out prior transfer
    tft.startWrite(); // and start a fresh one (required)
    // Address window centers QVGA image on screen. NO CLIPPING IS
    // PERFORMED, it is assumed here that the camera image is equal
    // or smaller than the screen.
    tft.setAddrWindow((tft.width() - cam.width()) / 2,
                      (tft.height() - cam.height()) / 2,
                      cam.width(), cam.height());
  }

  // Pause the camera DMA - hold buffer steady to avoid tearing
  cam.suspend();

  //cam.capture(); // Manual capture instead of PCC DMA

  // Postprocessing effects. These modify a previously-captured
  // image in memory, they are NOT in-camera effects.
  // Most only work in RGB mode (not YUV).
  //cam.image_negative();
  //cam.image_threshold(150);
  //cam.image_posterize(5);  // # of levels
  //cam.image_mosaic(21, 9); // Tile width, height
  //cam.image_median();
  //cam.image_edges(4);      // 0-31, smaller = more edges

  if(CAM_MODE == OV7670_COLOR_YUV) {
    cam.Y2RGB565(); // Convert grayscale for TFT preview
  }

  // Camera data arrives in big-endian order...same as the TFT,
  // so data can just be issued directly, no byte-swap needed.
  // Both the DMA and brute cases handle this.
#if defined(USE_SPI_DMA)
  tft.dmaWait();
  tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), false, true);
#elif defined(USE_SPI_BRUTE)
  brute.wait();
  brute.write((uint8_t *)cam.getBuffer(), cam.width() * cam.height() * 2);
#else
  tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), false, true);
#endif

  cam.resume(); // Resume DMA to camera buffer
}
