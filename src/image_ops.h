#pragma once
#include "ov7670.h"
#include <stdint.h>

// These are declared in an extern "C" so Arduino platform C++ code can
// access them.

#ifdef __cplusplus
extern "C" {
#endif

// Works, document
extern void OV7670_image_invert(uint16_t *pixels, uint16_t width,
                                uint16_t height);

// Works, document
extern void OV7670_image_threshold(OV7670_colorspace space, uint16_t *pixels,
                                   uint16_t width, uint16_t height,
                                   uint8_t threshold);

// Works, document
extern void OV7670_image_posterize(OV7670_colorspace space, uint16_t *pixels,
                                   uint16_t width, uint16_t height,
                                   uint8_t levels);

extern void OV7670_image_mosaic(OV7670_colorspace space, uint16_t *pixels,
                                uint16_t width, uint16_t height,
                                uint8_t tile_width, uint8_t tile_height);

extern void OV7670_image_edges(OV7670_colorspace space, uint16_t *pixels,
                               uint16_t width, uint16_t height,
                               uint8_t sensitivity);

#ifdef __cplusplus
};
#endif
