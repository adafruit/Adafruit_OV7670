// Camera test for Pico RP2040 + ST7789 240x240 display.
// Eventually this might get merged into existing cameratest sketch
// so the same sketch compiles for either SAMD51 or RP2040, but at
// the moment this diverges a lot (different display, etc.), maybe
// later when we have solid RP2040 Feather support, a good camera
// adapter PCB and some other things firm up.
// REQUIRES EARLE PHILHOWER RP2040 BOARD PACKAGE - Arduino mbed
// one won't work.

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "hardware/spi.h"

#define TFT_CS  16 // SPI0 at south end of board
#define TFT_DC  17
#define TFT_RST -1 // Connect to MCU reset

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void setup(void) {
  Serial.begin(9600);
  delay(1000);
  Serial.print(F("Hello! ST77xx TFT Test"));

  // These are currently RP2040 Philhower-specific
  SPI.setSCK(18); // SPI0
  SPI.setTX(19);

  tft.init(240, 240);
  tft.setSPISpeed(48000000);
  tft.fillScreen(ST77XX_BLACK);
  tft.println("Howdy");
}

void loop() {
}
