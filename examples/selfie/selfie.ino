/*
Example for Adafruit_OV7670 library. With camera plugged directly
into Grand Central PCC pins, it faces "up," same way as a TFT
shield, so think of it as a selfie camera. Tap "A" button to save
image to card. Use card slot on Grand Central, not the TFT shield!

This is a simple demo, NOT something for actual productive use, and
it's not especially thoughtful with your files. Existing images on
the SD card WILL be overwritten if they exist! Files will be in the
'SELFIES' folder and increment from #0001.

HARDWARE REQUIRED:
- Adafruit Grand Central board
- Adafruit 1.8" TFT shield V2
- OV7670 camera w/2.2K pullups to SDA+SCL, 3.3V+GND wires to shield
- microSD card (in card slot on Grand Central, not shield)

Board/shield is held with screen to the left, camera to the right,
buttons at bottom left (i.e. Grand Central silkscreen is readable).
This is actually upside-down with respect to the camera's native
orientation, but we compensate by rotating both the display output
and BMP writing 180 degrees.
*/

#include <Wire.h>                 // I2C comm to camera
#include <SD.h>                   // SD card support
#include "Adafruit_OV7670.h"      // Camera library
#include "Adafruit_ST7735.h"      // TFT display library
#include "Adafruit_seesaw.h"      // For TFT shield
#include "Adafruit_TFTShield18.h" // More TFT shield

// SD CARD CONFIG ----------------------------------------------------------

// The SD card library expects to use the Grand Central's onboard microSD
// slot, not the one on the shield. Being on a separate SPI bus, this avoids
// some transactions that would be needed when sharing SD+TFT on the shield.
#define SD_CS SDCARD_SS_PIN // Grand Central onboard SD card select

// CAMERA CONFIG -----------------------------------------------------------

// Set up arch and pins structures for Grand Central's SAMD51.
// PCC data pins are not configurable and do not need specifying.
OV7670_arch arch = {.timer = TCC1, .xclk_pdec = false};
OV7670_pins pins = {.enable = PIN_PCC_D8, .reset = PIN_PCC_D9,
                    .xclk = PIN_PCC_XCLK};
#define CAM_I2C  Wire1 // Second I2C bus next to PCC pins
#define CAM_SIZE OV7670_SIZE_DIV4 // QQVGA (160x120 pixels)
#define CAM_MODE OV7670_COLOR_RGB // RGB plz

Adafruit_OV7670 cam(OV7670_ADDR, &pins, &CAM_I2C, &arch);

// SHIELD AND DISPLAY CONFIG -----------------------------------------------

Adafruit_TFTShield18 shield;

#define TFT_CS   10  // Chip select for TFT on shield
#define TFT_DC    8  // Data/command line for TFT on Shield
#define TFT_RST  -1  // TFT reset is handled by seesaw
#define TFT_SPI SPI  // 6-pin ICSP header

// Screen DMA allows faster updates, but USE_SPI_DMA must be
// manually enabled in Adafruit_SPITFT.h (it's not on by default
// on Grand Central). If DMA is not enabled there, a helper class
// makes screen writes a little bit faster (but not in background
// like with DMA).
#if !defined(USE_SPI_DMA)
#include "SPIBrute.h" // Direct-to-SPI-registers helper for SAMD51
#endif                // end USE_SPI_DMA

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
#if defined(USE_SPI_BRUTE)
SPIBrute brute(&TFT_SPI);
#endif

// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup() {
  pinMode(SD_CS, OUTPUT);  
  digitalWrite(SD_CS, HIGH); // Disable SD card select during setup
  pinMode(4, OUTPUT);     
  digitalWrite(4, HIGH);     // Disable shield SD card select just in case

  Serial.begin(9600);
  //while (!Serial);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD begin() fail (continuing without)");
  }
  SD.mkdir("/selfies");

  if (!shield.begin()) { // Start seesaw helper chip on shield
    Serial.println("seesaw begin() fail");
    for(;;);
  }

  // TFT backlight OFF during setup
  shield.setBacklight(TFTSHIELD_BACKLIGHT_OFF);
  shield.tftReset();

  tft.initR(INITR_BLACKTAB);    // Initialize TFT on shield
  tft.setRotation(3);           // See notes earlier re: orientation
  tft.fillScreen(ST77XX_BLACK); // Clear background
#if defined(USE_SPI_BRUTE)
  brute.begin();
#endif

  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  // Camera buffer is allocated for the larger 320x240 pixel size,
  // though the preview is only 160x120. This is to ensure that the
  // RAM is available later when we go to take a higher-resolution
  // still to save.
  OV7670_status status = cam.begin(CAM_MODE, CAM_SIZE, 30.0, 320 * 240 * 2);
  if (status != OV7670_STATUS_OK) {
    Serial.println("Camera begin() fail");
    for(;;);
  }

  // Once everything's ready, turn TFT backlight on...
  shield.setBacklight(TFTSHIELD_BACKLIGHT_ON);
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
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
uint16_t bmp_num = 1;      // Image number increments with each BMP saved

void loop() {
  if (++frame >= KEYFRAME) { // Time to sync up a fresh address window?
    frame = 0;
#if defined(USE_SPI_DMA)
    tft.dmaWait(); // Wait for prior transfer to complete
#elif defined(USE_SPI_BRUTE)
    brute.wait();
#endif
    tft.endWrite();   // Close out prior transfer
    tft.startWrite(); // and start a fresh one (required)
    // Address window centers QQVGA image on screen. NO CLIPPING IS
    // PERFORMED, it is assumed here that the camera image is equal
    // or smaller than the screen.
    tft.setAddrWindow((tft.width() - cam.width()) / 2,
                      (tft.height() - cam.height()) / 2,
                      cam.width(), cam.height());
  }

  // Pause the camera DMA - hold buffer steady to avoid tearing
  cam.suspend();

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

  if(!(shield.readButtons() & TFTSHIELD_BUTTON_1)) {
    char filename[50];
    sprintf(filename, "/selfies/img%04d.bmp", bmp_num++);
#if defined(USE_SPI_DMA)
    tft.dmaWait(); // Wait for prior transfer to complete
#elif defined(USE_SPI_BRUTE)
    brute.wait();
#endif
    // Set camera capture to larger size (320x240). The REALLOC_NONE
    // tells it to keep the original buffer in place, which we allocated
    // large enough in setup() to handle these stills. Continual
    // reallocation would just be asking for trouble.
    cam.setSize(OV7670_SIZE_DIV2, OV7670_REALLOC_NONE);
    delay(100);         // Stabilize for a few frames
    tft.endWrite();     // Close out prior pixel write
    tft.setRotation(1); // Put text in readable orientation
    tft.fillRect(42, 58, 74, 18, 0x0000);
    tft.setCursor(44, 60);
    tft.print("SAVING");
    tft.setRotation(3); // Go back to 180 degree screen rotation
    frame = 999;        // Force keyframe on next update
    cam.capture();      // Manual (non-DMA) capture
    write_bmp(filename, cam.getBuffer(), cam.width(), cam.height());
    // Restore the original preview size from camera. Again, use
    // REALLOC_NONE to maintain our original camera buffer.
    cam.setSize(CAM_SIZE, OV7670_REALLOC_NONE);
  }

  cam.resume(); // Resume DMA into camera buffer
}

// Write 16-bit value to BMP file in little-endian order
void writeLE16(File *file, uint16_t value) {
#if(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
  file->write((char *)&value, sizeof value);
#else
  file->write( value       & 0xFF);
  file->write((value >> 8) & 0xFF);
#endif
}

// Write 32-bit value to BMP file in little-endian order
void writeLE32(File *file, uint32_t value) {
#if(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
  file->write((char *)&value, sizeof value);
#else
  file->write( value        & 0xFF);
  file->write((value >>  8) & 0xFF);
  file->write((value >> 16) & 0xFF);
  file->write((value >> 24) & 0xFF);
#endif
}

// Minimalist RGB565 BMP-writing function.
void write_bmp(char *filename, uint16_t *addr,
               uint16_t width, uint16_t height) {
  SD.remove(filename); // Delete existing file, if any
  File file = SD.open(filename, FILE_WRITE);
  if(file) {
    // BMP header, 14 bytes:
    file.write(0x42);                               // Windows BMP signature
    file.write(0x4D);                               // "
    writeLE32(&file, 14 + 56 + width * height * 2); // File size in bytes
    writeLE32(&file, 0);                            // Creator bytes (ignored)
    writeLE32(&file, 14 + 56);                      // Offset to pixel data
  
    // DIB header, 56 bytes "BITMAPV3INFOHEADER" type (for RGB565):
    writeLE32(&file, 56);                 // Header size in bytes
    writeLE32(&file, width);              // Width in pixels
    writeLE32(&file, height);             // Height in pixels (bottom-to-top)
    writeLE16(&file, 1);                  // Planes = 1
    writeLE16(&file, 16);                 // Bits = 16
    writeLE32(&file, 3);                  // Compression = bitfields
    writeLE32(&file, width * height);     // Bitmap size (adobe adds 2 here also)
    writeLE32(&file, 2835);               // Horiz resolution (72dpi)
    writeLE32(&file, 2835);               // Vert resolution (72dpi)
    writeLE32(&file, 0);                  // Default # colors in palette
    writeLE32(&file, 0);                  // Default # "important" colors
    writeLE32(&file, 0b1111100000000000); // Red mask
    writeLE32(&file, 0b0000011111100000); // Green mask
    writeLE32(&file, 0b0000000000011111); // Blue mask
    writeLE32(&file, 0);                  // Alpha mask

    // As mentioned in beginning, the camera is upside-down when board is
    // held at the intended orientation. BMPs are normally stored bottom-
    // to-top, so walking through rows incrementally takes care of this
    // axis, and horizontal axis is manually flipped.
    for(int y=0; y<height; y++) {
      uint16_t *row = &addr[y * width];
      for(int x=width-1;x >= 0; x--) { // Flip X
        // Raw data from camera is big-endian, BMP needs little-endian
        uint16_t pixel = __builtin_bswap16(row[x]);
        file.write((char *)&pixel, 2);
        // I mean yeah, since we're flipping both the X axis and endian-
        // swapping, we COULD just write out the individual row bytes in
        // reverse order. Handling pixels & bytes separately here so
        // there's no confusion if this code gets used elsewhere and
        // changed with X in the + direction.
      }
      // No scanline pad is added. We know the OV7670 library image sizes
      // are always an even width (4 bytes). Something to keep in mind if
      // flexible sizing and/or cropping options are added later.
    }

    file.close();
  }
}
