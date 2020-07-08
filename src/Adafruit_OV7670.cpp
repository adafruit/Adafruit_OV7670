/*
TEMPORARY NOTES:

Constructor might accept an array of pin numbers rather than
just a few. On SAMD51, the PCC peripheral pins are set in stone,
but on other devices there might be more flexibility, and that
needs to be passed to this code.

Add initial resolution to begin() function.

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

*/


/*!
 * @file Adafruit_OV7670.cpp
 *
 * @mainpage Adafruit OV7670 Camera Library
 *
 * @section intro_sec Introduction
 *
 * This is documentation for Adafruit's OV7670 camera driver for the
 * Arduino platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on
 * <a href="https://github.com/adafruit/Adafruit_ZeroDMA">
 * Adafruit_ZeroDMA</a> being present on your system. Please make sure you
 * have installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 */

#include "Adafruit_OV7670.h"
#include <Arduino.h>
#include <Wire.h>

Adafruit_OV7670::Adafruit_OV7670(uint8_t addr, OV7670_pin enable,
                                 OV7670_pin reset, OV7670_pin xclk,
                                 TwoWire *twi_ptr, OV7670_arch *arch_ptr)
    : i2c_address(addr & 0x7f), enable_pin(enable), reset_pin(reset),
      xclk_pin(xclk), wire(twi_ptr), arch_defaults((arch_ptr == NULL)),
      buffer(NULL) {
  if (arch_ptr) {
    memcpy(&arch, arch_ptr, sizeof(OV7670_arch));
  }
}

Adafruit_OV7670::~Adafruit_OV7670() {
  if (buffer) {
    free(buffer);
  }
  // TO DO: arch-specific code should have a function to clean up DMA
  // and timer. No rush really, destructor is unlikely to ever be used.
}

// CAMERA INIT AND CONFIG FUNCTIONS ----------------------------------------

OV7670_status Adafruit_OV7670::begin(void) {

  wire->begin();
  wire->setClock(100000); // Datasheet claims 400 KHz, but no, use 100 KHz

  _width = 320;
  _height = 240;

  // Alloc buffer for camera
  buffer = (uint16_t *)malloc(_width * _height * sizeof(uint16_t));
  if (buffer == NULL)
    return OV7670_STATUS_ERR_MALLOC;

  arch_begin(); // Device-specific setup
}

int Adafruit_OV7670::readRegister(uint8_t reg) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->endTransmission();
  wire->requestFrom(i2c_address, (uint8_t)1);
  return wire->read();
}

void Adafruit_OV7670::writeRegister(uint8_t reg, uint8_t value) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->write(value);
  wire->endTransmission();
}

bool Adafruit_OV7670::setResolution(OV7670_size size, OV7670_downsample x,
                                    OV7670_downsample y) {
  return true; // TO DO: make this realloc buf & return status
}

bool Adafruit_OV7670::setResolution(OV7670_size size, OV7670_downsample d) {
  return setResolution(size, d, d); // Call X/Y sizer w/same value for both
}

// C-ACCESSIBLE FUNCTIONS --------------------------------------------------

// These functions are declared in an extern "C" block in Adafruit_OV7670.h
// so that arch/*.c code can access them here. The Doxygen comments in the
// .h explain their use in more detail.

void OV7670_print(char *str) { Serial.print(str); }

int OV7670_read_register(void *obj, uint8_t reg) {
  return ((Adafruit_OV7670 *)obj)->readRegister(reg);
}

void OV7670_write_register(void *obj, uint8_t reg, uint8_t value) {
  ((Adafruit_OV7670 *)obj)->writeRegister(reg, value);
}
