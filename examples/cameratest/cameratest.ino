// OV7670 camera WIP for Grand Central.

#include "Adafruit_ILI9341.h"
#include "Adafruit_OV7670.h"
#include <Wire.h>

#if defined(__SAMD51__) // Grand Central or other M4 boards
// The arch structure is different for each supported device.
// On SAMD51, these indicate which timer peripheral is connected
// to the camera's XCLK pin, and (if the timer is a TCC rather than
// TC), whether the pin MUX for that timer requires PDEC vs TIMER_ALT.
OV7670_arch arch = {.timer = TCC1, .xclk_pdec = false};
OV7670_pins pins = {.enable = PIN_PCC_D8, .reset = PIN_PCC_D9, .xclk = PIN_PCC_XCLK};
// To use screen DMA on SAMD51, USE_SPI_DMA *must* be manually enabled
// in Adafruit_SPITFT.h. Only a few boards with built-in screens have
// it on by default.
#if !defined(USE_SPI_DMA)
#include "SPIBrute.h" // Direct-to-SPI-registers helper class for SAMD51
#endif                // end USE_SPI_DMA
#endif                // end __SAMD51__

// TFT shield pinout. Switch these if using Feather (DC=10, CS=9)
#define TFT_DC 9
#define TFT_CS 10
#define TFT_SPI SPI
Adafruit_ILI9341 tft(&TFT_SPI, TFT_DC, TFT_CS);
#if defined(USE_SPI_BRUTE)
SPIBrute brute(&TFT_SPI);
#endif

Adafruit_OV7670 cam(OV7670_ADDR, &pins, &Wire1, &arch);
#define CAM_MODE OV7670_COLOR_RGB
#define CAM_SIZE OV7670_SIZE_DIV4

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
   while (!Serial);
  Serial.println("Hello");
  Serial.flush();

  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  OV7670_status status = cam.begin(CAM_MODE, CAM_SIZE, 30.0);
  if (status != OV7670_STATUS_OK) {
    Serial.println("Camera begin() fail");
    Serial.flush();
  }

  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73

  Serial.println(pid, HEX);
  Serial.println(ver, HEX);

  // 50 MHz to screen is OK if wiring is clean (e.g. shield or FeatherWing).
  // Otherwise (if using jumper wires to screen), stick to 24 MHz.
#if defined(__SAMD51__)
  TFT_SPI.setClockSource(SERCOM_CLOCK_SOURCE_100M);
  tft.begin(50000000);
#else
  tft.begin(24000000);
#endif
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLUE);
#if defined(USE_SPI_BRUTE)
  brute.begin();
#endif
}

#define KEYFRAME 30   // Periodically issue setAddrWindow commands
uint32_t frame = 999; // Force 1st frame as keyframe

void loop() {
  Serial.println("ping");

  if(Serial.available()) {
    uint32_t vstart = Serial.parseInt();
    uint32_t hstart = Serial.parseInt();
    uint32_t edge_offset = Serial.parseInt();
    uint32_t pclk_delay = Serial.parseInt();
    OV7670_frame_control((void *)&cam, CAM_SIZE, vstart, hstart, edge_offset, pclk_delay);
  }

  // setAddrWindow() involves a lot of context switching that can
  // slow things down a bit, so we don't do it on every frame.
  // Instead, it's only set periodically, and we just keep writing
  // data to the same area of the screen (it wraps around automatically).
  // We do need an OCCASIONAL setAddrWindow() in case SPI glitches,
  // as this syncs things up to a known region of the screen again.
  if (++frame >= KEYFRAME) {
    frame = 0;
#if defined(USE_SPI_DMA)
    tft.dmaWait(); // Wait for prior transfer to complete
#elif defined(USE_SPI_BRUTE)
    brute.wait();
#endif
    tft.endWrite();   // Close out prior transfer
    tft.startWrite(); // and start a fresh one (required)
    tft.setAddrWindow(0, 0, cam.width(), cam.height());
  }

  // Pause the camera DMA - hold buffer steady to avoid tearing
  cam.suspend();

  if(CAM_MODE == OV7670_COLOR_YUV) {
    // Convert YUV gray component to RGB565 for TFT display
    uint16_t *ptr = cam.getBuffer();
    for(uint32_t i=cam.width() * cam.height(); i; i--) {
      uint8_t y = *ptr & 0xFF; // Y (brightness) component of YUV
      uint16_t rgb = ((y >> 3) * 0x801) | ((y & 0xFC) << 3); // RGB565
      *ptr++ = __builtin_bswap16(rgb); // Big-endianify rgb
    }
  }

//    cam.capture(); // Manual capture instead of PCC DMA

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
