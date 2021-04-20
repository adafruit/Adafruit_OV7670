// Camera test for Pico RP2040 + ST7789 240x240 display.
// Eventually this might get merged into existing cameratest sketch
// so the same sketch compiles for either SAMD51 or RP2040, but at
// the moment this diverges a lot (different display, etc.), maybe
// later when we have solid RP2040 Feather support, a good camera
// adapter PCB and some other things firm up.
// REQUIRES EARLE PHILHOWER RP2040 BOARD PACKAGE - Arduino mbed
// one won't work.

#include <Wire.h>            // I2C comm to camera
#include "Adafruit_OV7670.h" // Camera library
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// CAMERA CONFIG -----------------------------------------------------------

#if defined(PICO_SDK_VERSION_MAJOR)
#include "hardware/spi.h"

OV7670_arch arch;
OV7670_pins pins = {
  .enable = -1, // Also called PWDN, or set to -1 and tie to GND
  .reset  = 14, // Cam reset, or set to -1 and tie to 3.3V
  .xclk   = 13, // MCU clock out / cam clock in
  .pclk   = 10, // Cam clock out / MCU clock in
  .vsync  = 11, // Also called DEN1
  .hsync  = 12, // Also called DEN2
  .data   = {2, 3, 4, 5, 6, 7, 8, 9}, // Camera parallel data out
  .sda    = 20, // I2C data
  .scl    = 21, // I2C clock
};

#define CAM_I2C Wire

#endif

#define CAM_SIZE OV7670_SIZE_DIV4 // QQVGA (160x120 pixels)
#define CAM_MODE OV7670_COLOR_RGB // RGB plz

Adafruit_OV7670 cam(OV7670_ADDR, &pins, &CAM_I2C, &arch);
// Adafruit_OV7670 cam(0x30, &pins, &CAM_I2C, &arch); // OIV2640 WIP

// DISPLAY CONFIG ----------------------------------------------------------

#define TFT_CS  17 // Near SPI0 at south end of board
#define TFT_DC  16
#define TFT_RST -1 // Connect to MCU reset

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup(void) {
  Serial.begin(9600);
  //while(!Serial);
  delay(1000);
  Serial.println(F("Hello! Camera Test"));

  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);

  // These are currently RP2040 Philhower-specific
  SPI.setSCK(18); // SPI0
  SPI.setTX(19);
  Wire.setSDA(pins.sda); // I2C0
  Wire.setSCL(pins.scl);

  tft.init(240, 240);
  tft.setSPISpeed(48000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.println("Howdy");
tft.setRotation(3);
  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  OV7670_status status = cam.begin(CAM_MODE, CAM_SIZE, 30.0);
  if (status != OV7670_STATUS_OK) {
    Serial.println("Camera begin() fail");
    for(;;);
  }

#if 0
// OV2640 WIP
  cam.writeRegister(0xFF, 0x01);
  uint8_t pid = cam.readRegister(0x0A); // Should be 0x26
  uint8_t ver = cam.readRegister(0x0B); // Should be 0x41
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);
#else
  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);
#endif
//  cam.test_pattern(OV7670_TEST_PATTERN_COLOR_BAR);
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

  gpio_xor_mask(1 << 25); // Toggle LED each frame

  // This was for empirically testing window settings in src/arch/ov7670.c.
  // Your code doesn't need this. Just keeping around for future reference.
  if(Serial.available()) {
    uint32_t vstart = Serial.parseInt();
    uint32_t hstart = Serial.parseInt();
    uint32_t edge_offset = Serial.parseInt();
    uint32_t pclk_delay = Serial.parseInt();
    while(Serial.read() >= 0); // Delete line ending or other cruft
    OV7670_frame_control((void *)&cam, CAM_SIZE, vstart, hstart,
                         edge_offset, pclk_delay);
  }

  if (++frame >= KEYFRAME) { // Time to sync up a fresh address window?
    frame = 0;

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
//  cam.suspend();

  //cam.capture(); // Manual capture instead of PIO DMA

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
  tft.writePixels(cam.getBuffer(), cam.width() * cam.height(), false, true);
  // To do: add a DMA SPI transfer here -- even if it's blocking,
  // should still go a ton faster due to endian match.

//  cam.resume(); // Resume DMA to camera buffer
}
