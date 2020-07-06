#pragma once

#include "arch/ov7670.h"
#include <Adafruit_ZeroDMA.h>
#include <Wire.h>

extern "C" {
void OV7670_print(char *str);
uint8_t OV7670_read_register(void *obj, uint8_t reg);
void OV7670_write_register(void *obj, uint8_t reg, uint8_t value);
};

/** OV7670 camera resolutions. */
enum OV7670_size {
  OV7670_SIZE_VGA = 0, ///< 640 x 480
  OV7670_SIZE_CIF,     ///< 352 x 288
  OV7670_SIZE_QVGA,    ///< 320 x 240
  OV7670_SIZE_QCIF,    ///< 176 x 144
};

/** OV7670 camera downsampling options. */
enum OV7670_downsample {
  OV7670_DOWNSAMPLE_1 = 0, ///< 1:1 (no downsampling, full resolution)
  OV7670_DOWNSAMPLE_2,     ///< 1:2 downsampling
  OV7670_DOWNSAMPLE_4,     ///< 1:4 downsampling
  OV7670_DOWNSAMPLE_8,     ///< 1:8 downsampling
};

/*
// These go into host struct:
// Or wait, no, I think TwoWire can be normal, is an Arduino thing.
void *timer = NULL,
bool pdec = false);
*/

/*!
    @brief  Class encapsulating OV7670 camera functionality.
*/

class Adafruit_OV7670 {
public:
  Adafruit_OV7670(uint8_t addr = OV7670_ADDR, OV7670_pin_t enable = -1,
    OV7670_pin_t reset = -1, OV7670_pin_t xclk = -1, TwoWire *twi_ptr = &Wire,
    OV7670_arch_t *arch_ptr = NULL);
  ~Adafruit_OV7670();
  // TO DO: pass initial resolution to begin() function
  OV7670_status begin(void);
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  uint16_t *getBuffer(void) { return buffer; }
  void suspend(void);
  void resume(void);
  uint16_t width(void) { return _width; }
  uint16_t height(void) { return _height; }

  bool setResolution(OV7670_size size, OV7670_downsample x,
                     OV7670_downsample y);
  bool setResolution(OV7670_size size, OV7670_downsample d);

  void capture(void);

  // Add options for resolution, mirror X/Y, maybe color bars, maybe endianism,
  // maybe negative, maybe night mode
  // CIF = 352 x 288
  // QVGA = 320 x 240
  // QCIF = 176 x 144
  // 2, 4 and 8x downsampling are available on both X & Y
  // Should PCC pins be passed to constructor?
  // Should all the constructor args go into a struct?
  // Should RAM be reallocated on resolution change, or alloc once at max?
  // 352x288x2 = 202752 bytes
  // 320x240x2 = 153600 bytes
  // Should probably realloc because 200K is excessive (have 256K on SAMD51),
  // don't want to use all that RAM if it's not needed.
  // In that case, should camera start up at least resolution?
  // That'd be QCIF w/8x downsample: 22 x 18 pixels = 792 bytes

  void cap(void);
  void cap2(void);

private:
  OV7670_status arch_begin(void);
  OV7670_arch_t arch;
  const uint8_t i2c_address;  ///< I2C address
  const int enable_pin;       ///< Pin # for camera enable (active low)
  const int reset_pin;        ///< Pin # for camera reset
  const int xclk_pin;         ///< Pin # for camera XCLK
// These are now in arch struct:
//  const void *xclk_timer;     ///< TC or TCC peripheral for XCLK
//  const bool pdec;            ///< XCLK pin periph is TCC_PDEC, not TIMER_ALT
  TwoWire *wire;              ///< I2C interface
  uint16_t *buffer;           ///< Camera buffer allocated by lib
  uint16_t _width;            ///< Current settings width
  uint16_t _height;           ///< Current settings height
  // These are SAMD-specific and should maybe just go in the arch/samd51.cpp
  // file as standalone variables (only a single instance is allowed anyway).
  Adafruit_ZeroDMA dma;       ///< DMA channel
  DmacDescriptor *descriptor; ///< DMA descriptor
};
