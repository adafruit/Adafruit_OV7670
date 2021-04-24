#pragma once

// This will need to change since ARDUINO_ARCH_RP2040 is Arduino-platform-
// specific (possibly Philhower specific), while this file is supposed to
// be platform neutral and RP2040-specific.
#if defined(ARDUINO_ARCH_RP2040)

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdbool.h>
#include <stdint.h>
#endif // end platforms

#include "../../hardware_dma/include/hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

typedef int8_t OV7670_pin;

// OV7670 datasheet claims 10-48 MHz clock input, with 24 MHz typical.
// 24(ish) MHz is OK if camera connection is super clean. If any trouble,
// try dialing this down to 16 or 12 MHz. Even 8 MHz is OK if that's what's
// available. OV7670 has internal PLL and can step up from lower frequencies
// (to a point) if needed.
#define OV7670_XCLK_HZ 12500000 ///< XCLK to camera, 8-24 MHz

// Device-specific structure attached to the OV7670_host.arch pointer.
typedef struct {
  PIO pio;                       ///< PIO peripheral (derived from pins)
  uint8_t sm;                    ///< State machine #
  int dma_channel;               ///< DMA channel #
  dma_channel_config dma_config; ///< DMA configuration
  uint32_t pclk_mask;            ///< GPIO bitmask for PCLK pin
  uint32_t vsync_mask;           ///< GPIO bitmask for VSYNC pin
  uint32_t hsync_mask;           ///< GPIO bitmask for HSYNC pin
} OV7670_arch;

#ifdef __cplusplus
extern "C" {
#endif

extern void OV7670_capture(uint16_t *dest, uint16_t width, uint16_t height,
                           volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                           volatile uint32_t *hsync_reg, uint32_t hsync_bit);

#ifdef __cplusplus
};
#endif

#endif // ARDUINO_ARCH_RP2040
