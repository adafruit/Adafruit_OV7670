/*!
 * @file Adafruit_OV7670.h
 *
 * This is documentation for Adafruit's OV7670 camera driver for the
 * Arduino platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 */

#pragma once

#include "arch/ov7670.h"
#include <Adafruit_ZeroDMA.h>
#include <Wire.h>

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

/*!
    @brief  Class encapsulating OV7670 camera functionality.
*/
class Adafruit_OV7670 {
public:
  /*!
    @brief  Constructor for Adafruit_OV7670 class.
    @param  addr      I2C address of camera.
    @param  enable    Arduino pin number connected to OV7670 enable or PWDN
                      pin (low = enable, high = power-down), or -1 if unused
                      (pin can be tied to ground).
    @param  reset     Arduino pin number connected to OV7670 reset (active
                      low), or -1 if unused (pin can be tied to 3.3V).
    @param  xclk      Arduino pin number connected to OV7670 XCLK (clock in).
    @param  twi_ptr   Pointer to TwoWire instance (e.g. &Wire or &Wire1),
                      used for I2C communication with camera.
    @param  arch_ptr  Pointer to structure containing architecture-specific
                      settings. For example, on SAMD51, this structure
                      includes a pointer to a timer peripheral's base address,
                      used to generate the xclk signal. The structure is
                      always of type OV7670_arch, but the specific elements
                      within will vary with each supported architecture.
  */
  Adafruit_OV7670(uint8_t addr = OV7670_ADDR, OV7670_pin enable = -1,
                  OV7670_pin reset = -1, OV7670_pin xclk = -1,
                  TwoWire *twi_ptr = &Wire, OV7670_arch *arch_ptr = NULL);
  ~Adafruit_OV7670(); // Destructor

  /*!
    @brief   Allocate and initialize resources behind an Adafruit_OV7670
             instance.
    @return  Status code. OV7670_STATUS_OK on successful init.
  */
  OV7670_status begin(void);

  /*!
    @brief   Reads value of one register from the OV7670 camera over I2C.
    @param   reg  Register to read, from values defined in src/arch/ov7670.h.
    @return  Integer value: 0-255 (register contents) on successful read,
             -1 on error.
  */
  int readRegister(uint8_t reg);

  /*!
    @brief  Writes value of one register to the OV7670 camera over I2C.
    @param  reg    Register to read, from values defined in src/arch/ov7670.h.
    @param  value  Value to write, 0-255.

  */
  void writeRegister(uint8_t reg, uint8_t value);

  /*!
    @brief   Get address of image buffer being used by camera.
    @return  uint16_t pointer to image data in RGB565 format.
  */
  uint16_t *getBuffer(void) { return buffer; }

  /*!
    @brief  Pause camera before capturing, to avoid tearing.
  */
  void suspend(void);

  /*!
    @brief  Resume camera after suspend/capture.
  */
  void resume(void);

  /*!
    @brief   Get image width of camera's current resolution setting.
    @return  Width in pixels.
  */
  uint16_t width(void) { return _width; }

  /*!
    @brief   Get image height of camera's current resolution setting.
    @return  Height in pixels.
  */
  uint16_t height(void) { return _height; }

  /*!
    @brief   This will change.
    @param   size  One of the size settings from ov7670.h.
    @param   x     Horizontal scaling.
    @param   y     Vertical scaling.
    @return  true on success (image buffer reallocated, camera configured),
             false on error (usu. malloc).
  */
  bool setResolution(OV7670_size size, OV7670_downsample x,
                     OV7670_downsample y);
  /*!
    @brief   Ditto.
    @param   size  One of the size settings from ov7670.h.
    @param   d     Scaling factor (both horizontal and vertical).
    @return  true on success (image buffer reallocated, camera configured),
             false on error (usu. malloc).
  */
  bool setResolution(OV7670_size size, OV7670_downsample d);

  /*!
    @brief  Capture still to buffer.
  */
  void capture(void);

private:
  OV7670_status arch_begin(void); ///< Architecture-specific periph setup
  TwoWire *wire;                  ///< I2C interface
  uint16_t *buffer;               ///< Camera buffer allocated by lib
  OV7670_arch arch;               ///< Architecture-specific peripheral info
  uint16_t _width;                ///< Current settings width in pixels
  uint16_t _height;               ///< Current settings height in pixels
  const uint8_t i2c_address;      ///< I2C address
  const OV7670_pin enable_pin;    ///< Pin # for camera enable (active low)
  const OV7670_pin reset_pin;     ///< Pin # for camera reset
  const OV7670_pin xclk_pin;      ///< Pin # for camera XCLK
  const bool arch_defaults;       ///< If set, ignore arch struct, use defaults
};

// C-ACCESSIBLE FUNCTIONS --------------------------------------------------

// These functions are declared in an extern "C" so that arch/*.c code can
// access them here. They provide a route from the mid- and low-level C code
// back up to the Arduino level, so object-based functions like
// Serial.print() and Wire.write() can be called.

extern "C" {

/*!
    @brief  A Serial Console print function for crude debugging and status
            info -- simply forwards a string to Serial.print().
    @param  str  String to print to the Serial Console. Must contain its
                 own newline character if needed.
*/
void OV7670_print(char *str);

/*!
    @brief   Reads value of one register from the OV7670 camera over I2C.
             This is a C wrapper around the C++ readRegister() function.
    @param   obj  Pointer to Adafruit_OV7670 object (passed down to the C
                  code on init, now passed back). Adafruit_OV7670 contains
                  a pointer to a TwoWire object, allowing I2C operations
                  (via C++ class) in the lower-level C code.
    @param   reg  Register to read, from values defined in src/arch/ov7670.h.
    @return  Integer value: 0-255 (register contents) on successful read,
             -1 on error.
*/
int OV7670_read_register(void *obj, uint8_t reg);

/*!
    @brief  Writes value of one register to the OV7670 camera over I2C.
            This is a C wrapper around the C++ writeRegister() function.
    @param  obj    Pointer to Adafruit_OV7670 object (passed down to the C
                   code on init, now passed back). Adafruit_OV7670 contains
                   a pointer to a TwoWire object, allowing I2C operations
                   (via C++ class) in the lower-level C code.
    @param  reg    Register to read, from values defined in src/arch/ov7670.h.
    @param  value  Value to write, 0-255.

*/
void OV7670_write_register(void *obj, uint8_t reg, uint8_t value);

}; // end extern "C"

