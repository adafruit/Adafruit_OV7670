// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

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

Adafruit_OV7670::Adafruit_OV7670(uint8_t addr, OV7670_pins *pins_ptr,
                                 TwoWire *twi_ptr, OV7670_arch *arch_ptr)
    : i2c_address(addr & 0x7f), wire(twi_ptr),
      arch_defaults((arch_ptr == NULL)), buffer(NULL), buffer_size(0) {
  if (pins_ptr) {
    memcpy(&pins, pins_ptr, sizeof(OV7670_pins));
  }
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

OV7670_status Adafruit_OV7670::begin(OV7670_colorspace colorspace,
                                     OV7670_size size, float fps,
                                     uint32_t bufsiz) {

  wire->begin();
  wire->setClock(100000); // Datasheet claims 400 KHz, but no, use 100 KHz

  _width = 640 >> (int)size;  // 640, 320, 160, 80, 40
  _height = 480 >> (int)size; // 480, 240, 120, 60, 30
  space = colorspace;

  // Allocate buffer for camera
  buffer_size = bufsiz ? bufsiz : _width * _height * sizeof(uint16_t);
  buffer = (uint16_t *)malloc(buffer_size);
  if (buffer == NULL) {
    buffer_size = 0;
    return OV7670_STATUS_ERR_MALLOC;
  }

  return arch_begin(colorspace, size, fps); // Device-specific setup
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

OV7670_status Adafruit_OV7670::setSize(OV7670_size size, OV7670_realloc allo) {
  uint16_t new_width = 640 >> (int)size;
  uint16_t new_height = 480 >> (int)size;
  uint32_t new_buffer_size = new_width * new_height * sizeof(uint16_t);
  bool ra = false;

  switch (allo) {
  case OV7670_REALLOC_NONE:
    if (new_buffer_size > buffer_size) { // New size won't fit
      // Don't realloc. Keep current camera settings, return error.
      return OV7670_STATUS_ERR_MALLOC;
    }
    break;
  case OV7670_REALLOC_CHANGE:
    ra = (new_buffer_size != buffer_size); // Realloc on size change
    break;
  case OV7670_REALLOC_LARGER:
    ra = (new_buffer_size > buffer_size); // Realloc on size increase
    break;
  }

  if (ra) { // Reallocate?
    uint16_t *new_buffer = (uint16_t *)realloc(buffer, new_buffer_size);
    if (new_buffer == NULL) { // FAIL
      _width = _height = buffer_size = 0;
      buffer = NULL;
      // Calling code had better poll width(), height() or getBuffer() in
      // this case so it knows the camera buffer is gone, doesn't attempt
      // to read camera data into unknown RAM.
      return OV7670_STATUS_ERR_MALLOC;
    }
    buffer_size = new_buffer_size;
  }

  _width = new_width;
  _height = new_height;

  OV7670_set_size(this, size);

  return OV7670_STATUS_OK;
}

void Adafruit_OV7670::Y2RGB565(void) {
  OV7670_Y2RGB565(buffer, _width * _height);
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
