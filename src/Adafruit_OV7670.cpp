#include "Adafruit_OV7670.h"
#include <Arduino.h>
#include <Wire.h>

// timer and pdec will instead go in host struct
Adafruit_OV7670::Adafruit_OV7670(uint8_t addr, OV7670_pin enable,
  OV7670_pin reset, OV7670_pin xclk, TwoWire *twi_ptr,
  OV7670_arch *arch_ptr) : i2c_address(addr & 0x7f), enable_pin(enable),
  reset_pin(reset), xclk_pin(xclk), wire(twi_ptr), buffer(NULL) {
  if(arch_ptr) {
    memcpy(&arch, arch_ptr, sizeof(OV7670_arch));
  }
}

Adafruit_OV7670::~Adafruit_OV7670() {
  if (buffer) {
    free(buffer);
  }
  // TO DO: clean up DMA and timer here. No rush, destructor is unlikely
  // to ever be used.
}

// CAMERA INIT AND CONFIG FUNCTIONS ----------------------------------------

OV7670_status Adafruit_OV7670::begin(void) {

  wire->begin();
  wire->setClock(100000); // Datasheet claims 400 KHz, but no, use 100 KHz

  // Alloc buffer for camera
  buffer = (uint16_t *)malloc(_width * _height * sizeof(uint16_t));
  if (buffer == NULL)
    return OV7670_STATUS_ERR_MALLOC;

  arch_begin(); // Device-specific setup
}

uint8_t Adafruit_OV7670::readRegister(uint8_t reg) {
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

// These are extern "C" in .h so arch/*.c files can access them
void OV7670_print(char *str) {
  Serial.print(str);
}

uint8_t OV7670_read_register(void *obj, uint8_t reg) {
  return ((Adafruit_OV7670 *)obj)->readRegister(reg);
}

void OV7670_write_register(void *obj, uint8_t reg, uint8_t value) {
  ((Adafruit_OV7670 *)obj)->writeRegister(reg, value);
}
