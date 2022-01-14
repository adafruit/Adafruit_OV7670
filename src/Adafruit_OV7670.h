// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

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

#include "image_ops.h"
#include "ov7670.h"
#include <Adafruit_ZeroDMA.h>
#include <Wire.h>

/** Buffer reallocation behaviors requested of setSize() */
typedef enum {
  OV7670_REALLOC_NONE = 0, ///< No realloc, error if new size > current buffer
  OV7670_REALLOC_CHANGE,   ///< Reallocate image buffer if size changes
  OV7670_REALLOC_LARGER,   ///< Realloc only if new size is larger
} OV7670_realloc;

/*!
    @brief  Class encapsulating OV7670 camera functionality.
*/
class Adafruit_OV7670 {
public:
  /*!
    @brief  Constructor for Adafruit_OV7670 class.
    @param  addr      I2C address of camera.
    @param  pins_ptr  Pointer to OV7670_pins structure, describing physical
                      connection to the camera.
    @param  twi_ptr   Pointer to TwoWire instance (e.g. &Wire or &Wire1),
                      used for I2C communication with camera.
    @param  arch_ptr  Pointer to structure containing architecture-specific
                      settings. For example, on SAMD51, this structure
                      includes a pointer to a timer peripheral's base address,
                      used to generate the xclk signal. The structure is
                      always of type OV7670_arch, but the specific elements
                      within will vary with each supported architecture.
  */
  Adafruit_OV7670(uint8_t addr = OV7670_ADDR, OV7670_pins *pins_ptr = NULL,
                  TwoWire *twi_ptr = &Wire, OV7670_arch *arch_ptr = NULL);
  ~Adafruit_OV7670(); // Destructor

  /*!
    @brief   Allocate and initialize resources behind an Adafruit_OV7670
             instance.
    @param   colorspace  OV7670_COLOR_RGB or OV7670_COLOR_YUV.
    @param   size        Frame size as a power-of-two reduction of VGA
                         resolution. Available sizes are OV7670_SIZE_DIV1
                         (640x480), OV7670_SIZE_DIV2 (320x240),
                         OV7670_SIZE_DIV4 (160x120), OV7670_SIZE_DIV8 and
                         OV7670_SIZE_DIV16.
    @param   fps         Desired capture framerate, in frames per second,
                         as a float up to 30.0. Actual device frame rate may
                         differ from this, depending on a host's available
                         PWM timing. Generally, the actual device fps will
                         be equal or nearest-available below the requested
                         rate, only in rare cases of extremely low requested
                         frame rates will a higher value be used. Since
                         begin() only returns a status code, if you need to
                         know the actual framerate you can call
                         OV7670_set_fps(NULL, fps) at any time before or
                         after begin() and that will return the actual
                         resulting frame rate as a float.
    @param   bufsiz      Image buffer size, in bytes. This is configurable so
                         code can do things like change image sizes without
                         reallocating (which risks losing the existing buffer)
                         or double-buffered transfers. Pass 0 to use default
                         buffer size equal to 2 bytes per pixel times the
                         number of pixels corresponding to the 'size'
                         argument. If you later call setSize() with an image
                         size exceeding the buffer size, it will fail.
    @return  Status code. OV7670_STATUS_OK on successful init.
  */
  OV7670_status begin(OV7670_colorspace colorspace = OV7670_COLOR_RGB,
                      OV7670_size size = OV7670_SIZE_DIV4, float fps = 30.0,
                      uint32_t bufsiz = 0);

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
    @brief  Pause DMA background capture (if supported by architecture)
            before capturing, to avoid tearing. Returns as soon as the
            current frame has finished loading. If DMA background capture
            is not supported, this function has no effect. This is NOT a
            camera sleep function!
  */
  void suspend(void);

  /*!
    @brief  Resume DMA background capture after suspend. If DMA is not
            supported, this function has no effect.
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
    @brief   Change camera resolution post-begin(). Not yet implemented.
    @param   size  One of the OV7670_size values ranging from full VGA
                   (640x480 pixels) to 1/16 VGA (40x30 pixels).
    @param   allo  Camera buffer reallocation behavior:
                   - OV7670_REALLOC_NONE to not reallocate buffer.
                     Function will return OV7670_STATUS_OK if new size fits
                     in existing buffer, or OV7670_ERR_MALLOC if existing
                     buffer is too small (buffer is not freed and current
                     camera size is maintained).
                   - OV7670_REALLOC_CHANGE to reallocate buffer on ANY
                     size change, up or down. Function will return
                     OV7670_STATUS_OK if reallocation was successful and
                     camera size changed, or OV7670_ERR_MALLOC if
                     reallocation failed (camera width and height will
                     subsequently both poll as 0, or buffer to NULL, in
                     this case).
                   - OV7670_REALLOC_LARGER to reallocate buffer ONLY if new
                     size exceeds current buffer size. Function will return
                     OV7670_STATUS_OK if reallocation was successful and
                     camera size changed, or OV7670_ERR_MALLOC if
                     reallocation failed (camera width and height will
                     subsequently both poll as 0 in this case).
    @return  Status code. OV7670_STATUS_OK on success (image buffer
             successfully reallocated as requested, camera reconfigured),
             OV7670_STATUS_ERR_MALLOC in several situations explained above.
    @note    Reallocating the camera buffer is fraught with peril and should
             only be done if you're prepared to handle any resulting error.
             In most cases, code should pass the size of LARGEST buffer it
             anticipates needing (including any double buffering, etc.) to
             begin(), which allocates it once on startup. Some RAM will go
             untilized at times, but it's favorable to entirely losing the
             camera mid-run. The default request here is CHANGE in case one
             passes an improper initial value to begin().
  */
  OV7670_status setSize(OV7670_size size,
                        OV7670_realloc allo = OV7670_REALLOC_CHANGE);

  /*!
    @brief  Capture still image to buffer. If background DMA capture is
            supported, this has no effect; latest image is always there.
  */
  void capture(void);

  /*!
    @brief  Select one of the camera's night modes. Images are less
            grainy in low light, tradeoff being a reduced frame rate.
    @param  night  One of the OV7670_night_mode types:
                   OV7670_NIGHT_MODE_OFF  Disable night mode, full frame rate
                   OV7670_NIGHT_MODE_2    1/2 frame rate
                   OV7670_NIGHT_MODE_4    1/4 frame rate
                   OV7670_NIGHT_MODE_8    1/8 frame rate
  */
  void night(OV7670_night_mode night) { OV7670_night(this, night); }

  /*!
    @brief  Flip camera output on horizontal and/or vertical axes.
            Flipping both axes is equivalent to 180 degree rotation.
    @param  flip_x  If true, flip camera output on horizontal axis.
    @param  flip_y  If true, flip camera output on vertical axis.
    @note   Datasheet refers to horizontal flip as "mirroring," but
            avoiding that terminology here that it might be mistaken for a
            split-down-middle-and-reflect funhouse effect, which it isn't.
  */
  void flip(bool flip_x, bool flip_y) { OV7670_flip(this, flip_x, flip_y); }

  /*!
    @brief  Enable/disable camera test pattern output.
    @param  pattern  One of the OV7670_pattern values:
                     OV7670_TEST_PATTERN_NONE
                       Disable test pattern, display normal camera video.
                     OV7670_TEST_PATTERN_SHIFTING_1
                       "Shifting 1" test pattern (seems to be single-column
                       vertical RGB lines).
                     OV7670_TEST_PATTERN_COLOR_BAR
                       Eight color bars.
                     OV7670_TEST_PATTERN_COLOR_BAR_FADE
                       Eight color bars with fade to white.
    @note   This basically works but has some artifacts...color bars are
            wrapped around such that a few pixels of the leftmost (white)
            bar appear at to the right of the rightmost (black) bar. It
            seems that the frame control settings need to be slightly
            different for image sensor vs test patterns (frame control
            that displays the bars correctly has green artifacts along
            right edge when using image sensor). Eventually will want to
            make this handle the different cases and sizes correctly.
            In the meantime, there's minor uglies in test mode.
            Similar issue occurs with image flips.
  */
  void test_pattern(OV7670_pattern pattern) {
    OV7670_test_pattern(this, pattern);
  }

  /*!
    @brief  Produces a negative image. This is a postprocessing effect,
            not in-camera, and must be applied to frame(s) manually.
            Image in memory will be overwritten.
  */
  void image_negative(void) { OV7670_image_negative(buffer, _width, _height); };

  /*!
    @brief  Decimate an image to only it's min/max values (ostensibly
            "black and white," but works on color channels separately
            so that's not strictly the case). This is a postprocessing
            effect, not in-camera, and must be applied to frame(s) manually.
            Image in memory will be overwritten.
    @param  threshold  Threshold level, 0-255; pixel brightnesses at or
                       above this level are set to the maximum, below this
                       level are set to the minimum. Input value is scaled
                       to accommodate the lower color fidelity of the RGB
                       colorspace -- use 0 to 255, not 0 to 31 or 63.
  */
  void image_threshold(uint8_t threshold = 128) {
    OV7670_image_threshold(space, buffer, _width, _height, threshold);
  };

  /*!
    @brief  Decimate an image to a limited number of brightness levels or
            steps. This is a postprocessing effect, not in-camera, and must
            be applied to frame(s) manually. Image in memory will be
            overwritten.
    @param  levels  Number of brightness levels -- 2 to 32 for RGB
                    colorspace, 2 to 255 for YUV.
  */
  void image_posterize(uint8_t levels = 4) {
    OV7670_image_posterize(space, buffer, _width, _height, levels);
  };

  /*!
    @brief  Mosaic or "shower door effect," downsamples an image into
            rectangular tiles, each tile's color being the average of all
            source image pixels within that tile's area. This is a
            postprocessing effect, not in-camera, and must be applied to
            frame(s) manually. Image in memory will be overwritten. If
            image size does not divide equally by tile size, fractional
            tiles will always be along the right and/or bottom edge(s);
            top left corner is always a full tile.
            YUV colorspace is not currently supported.
    @param  tile_width   Tile width in pixels (1 to 255)
    @param  tile_height  Tile height in pixels (1 to 255)
  */
  void image_mosaic(uint8_t tile_width = 8, uint8_t tile_height = 8) {
    OV7670_image_mosaic(space, buffer, _width, _height, tile_width,
                        tile_height);
  };

  /*!
    @brief  3x3 pixel median filter, reduces visual noise in image.
            This is a postprocessing effect, not in-camera, and must be
            applied to frame(s) manually. Image in memory will be
            overwritten. YUV colorspace is not currently supported.
  */
  void image_median(void) {
    OV7670_image_median(space, buffer, _width, _height);
  };

  /*!
    @brief  Edge detection filter.
            This is a postprocessing effect, not in-camera, and must be
            applied to frame(s) manually. Image in memory will be
            overwritten. YUV colorspace is not currently supported.
    @param  sensitivity  Smaller value = more sensitive to edge changes.
  */
  void image_edges(uint8_t sensitivity = 7) {
    OV7670_image_edges(space, buffer, _width, _height, sensitivity);
  };

  /*!
    @brief  Convert Y (brightness) component YUV image in RAM to RGB565
            big-endian format for preview on TFT display. Camera buffer is
            overwritten in-place, Y is truncated and UV elements are lost.
            No practical use outside TFT preview. If you need actual
            grayscale 0-255 data, just access the low byte of each 16-bit
            YUV pixel.
  */
  void Y2RGB565(void);

private:
  OV7670_status arch_begin(OV7670_colorspace colorspace, OV7670_size size,
                           float fps);
  TwoWire *wire;             ///< I2C interface
  uint16_t *buffer;          ///< Camera buffer allocated by lib
  uint32_t buffer_size;      ///< Size of camera buffer, in bytes
  OV7670_pins pins;          ///< Camera physical connections
  OV7670_arch arch;          ///< Architecture-specific peripheral info
  uint16_t _width;           ///< Current settings width in pixels
  uint16_t _height;          ///< Current settings height in pixels
  OV7670_colorspace space;   ///< RGB or YUV colorspace
  const uint8_t i2c_address; ///< I2C address
  const bool arch_defaults;  ///< If set, ignore arch struct, use defaults
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
