// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once
#if defined(__SAMD51__)
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif // end platforms

typedef int8_t OV7670_pin;

// OV7670 datasheet claims 10-48 MHz clock input, with 24 MHz typical.
// SAMD can do up to 24 MHz if camera connection is super clean. If any
// trouble, try dialing this down to 16 or 12 MHz. Even 8 MHz is OK if
// that's what's available. SAMD timer peripheral as used by this code
// is clocked from a 48 MHz source, so it's always going to be some
// integer divisor of that. OV7670 has internal PLL and can step up from
// lower frequencies (to a point) if needed.
#define OV7670_XCLK_HZ 24000000 ///< XCLK to camera, 8-24 MHz

// Device-specific structure attached to the OV7670_host.arch pointer.
typedef struct {
  void *timer;    ///< TC or TCC peripheral base address for XCLK out
  bool xclk_pdec; ///< If true, XCLK needs special PDEC pin mux
} OV7670_arch;

#ifdef __cplusplus
extern "C" {
#endif

extern void OV7670_capture(uint32_t *dest, uint16_t width, uint16_t height,
                           volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                           volatile uint32_t *hsync_reg, uint32_t hsync_bit);

#ifdef __cplusplus
};
#endif

#endif // __SAMD51__
