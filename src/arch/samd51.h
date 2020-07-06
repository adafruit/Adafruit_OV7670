#pragma once
#if defined(__SAMD51__)
#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif // end platforms

typedef int8_t OV7670_pin_t;

// Device-specific structure attached to the OV7670_host.arch pointer.
typedef struct {
  void   *timer; ///< TC or TCC peripheral base address for XCLK out
  bool    xclk_pdec; ///< If true, XCLK needs special PDEC pin mux
} OV7670_arch_t;

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
