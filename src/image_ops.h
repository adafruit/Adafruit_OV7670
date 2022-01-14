// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once
#include "ov7670.h"
#include <stdint.h>

// The "image ops" functions perform postprocessing on a captured OV7670
// image. These are not in-camera effects, though some might be possible
// to implement as such. Image is overwritten -- destination buffer is
// always the same as the source buffer, same dimensions, same colorspace.

// These are declared in an extern "C" so Arduino platform C++ code can
// access them.

#ifdef __cplusplus
extern "C" {
#endif

// Image invert -- produces a negative image. This could probably be done
// in-camera with a different gamma curve or something, but for now it's
// available as a postprocess filter. Works in RGB and YUV colorspaces.
extern void OV7670_image_negative(uint16_t *pixels, uint16_t width,
                                  uint16_t height);

// Image threshold -- decimates an image to only its min/max values
// (ostensibly "black and white," but works on color channels separately
// so that's not strictly the case). Works in RGB and YUV colorspaces.
// Might be possible as an in-camera effect using gamma curve.
extern void OV7670_image_threshold(OV7670_colorspace space, uint16_t *pixels,
                                   uint16_t width, uint16_t height,
                                   uint8_t threshold);

// Image posterize -- decimates an image to a limited number of brightness
// levels -- 2 to 32 levels in RGB colorspace, 2 to 255 levels in YUV.
// As with threshold, color channels are separately processed.
// Might be possible as an in-camera effect using gamma curve.
extern void OV7670_image_posterize(OV7670_colorspace space, uint16_t *pixels,
                                   uint16_t width, uint16_t height,
                                   uint8_t levels);

// Image mosaic -- or "shower door effect," downsamples an image into
// rectangular "tiles" of selectable width and height, each tile's color
// being the average of all source image pixels within that tile's area.
// For some tile sizes (2x2, 4x4, etc.) it might be more efficient to use
// a lower-resolution camera setting and upscale (if needed) in your own
// code...but this function is easy if you need to maintain a specific
// image size, or want arbitrary X/Y tile sizes. If image size does not
// divide equally by tile size, fractional tiles will always be along the
// right and/or bottom edge(s); top left corner is always a full tile.
extern void OV7670_image_mosaic(OV7670_colorspace space, uint16_t *pixels,
                                uint16_t width, uint16_t height,
                                uint8_t tile_width, uint8_t tile_height);

// 3x3 median filter, WIP, not yet available
extern void OV7670_image_median(OV7670_colorspace space, uint16_t *pixels,
                                uint16_t width, uint16_t height);

// Edge detection, WIP, not yet available
extern void OV7670_image_edges(OV7670_colorspace space, uint16_t *pixels,
                               uint16_t width, uint16_t height,
                               uint8_t sensitivity);

#ifdef __cplusplus
};
#endif
