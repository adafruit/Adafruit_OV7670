#include "image_ops.h"

// These functions are preceded by "OV7670" even though they're not
// tied to the camera, just that they're part of this lib.
// Some of these probably could be done in camera with proper
// gamma settings, but aren't yet.

void OV7670_image_invert(uint16_t *pixels, uint16_t width, uint16_t height) {
  // Working 32 bits at a time is slightly faster. This is dirty pool,
  // relying on the fact that the camera lib currently only supports
  // even image sizes (the pixel count will always be a multiple of 2)
  // and that the image buffer, coming from malloc(), will always be
  // on a 32-bit-safe boundary. This is one of those operations that
  // can probably be implemented through the camera's gamma curve
  // settings, and if so this function will go away.
  uint32_t *p32 = (uint32_t *)pixels;
  uint32_t i, num_pairs = width * height / 2;
  for (i=0; i<num_pairs; i++) {
    p32[i] ^= 0xFFFFFFFF;
  }
}

// Pass in threshold as 0-255, this will be quantized to an
// appropriate range for the color mode.
void OV7670_image_threshold(OV7670_colorspace space, uint16_t *pixels,
                            uint16_t width, uint16_t height,
                            uint8_t threshold) {
  uint32_t i, num_pixels = width * height;
  if (space == OV7670_COLOR_RGB) {
    // Testing RGB thresholds "in place" in the packed RGB565 value
    // avoids some bit-shifting on every pixel (just bit masking).
    uint16_t rlimit = (threshold >> 3) << 11;   // In-place 565 red threshold
    uint16_t glimit = (threshold >> 2) << 5;    // In-place 565 green threshold
    uint16_t blimit = (threshold >> 3);         // In-place 565 blue threshold
    uint16_t rgb565in, rgb565out;               // Packed RGB565 pixel values
    for (i=0; i<num_pixels; i++) {              // For each pixel...
      rgb565in = __builtin_bswap16(pixels[i]);  //   Swap endian from cam
      rgb565out = 0;                            //   Start with 0 result
      if ((rgb565in & 0xF800) >= rlimit) {      //   If red exceeds limit
        rgb565out |= 0xF800;                    //     Set all red bits
      }
      if ((rgb565in & 0x07E0) >= glimit) {      //   Ditto, green
        rgb565out |= 0x07E0;
      }
      if ((rgb565in & 0x001F) >= blimit) {      //   Ditto, blue
        rgb565out |= 0x001F;
      }
      pixels[i] = __builtin_bswap16(rgb565out); //   Back to cam-native endian
    }
  } else {                                      // YUV...
    uint8_t *p8 = (uint8_t *)pixels;            // Separate Ys, Us, Vs
    num_pixels *= 2;                            // Actually num bytes now
    for (i=0; i<num_pixels; i++) {              // For each byte...
      p8[i] = (p8[i] >= threshold) ? 255 : 0;   //   Threshold to 0 or 255
    }
  }
}

void OV7670_image_posterize(OV7670_colorspace space, uint16_t *pixels,
                            uint16_t width, uint16_t height, uint8_t levels) {
  uint32_t i, num_pixels = width * height;

  if (levels < 1) {
    levels = 1;
  }
  uint8_t lm1 = levels - 1; // Values used in fixed-
  uint8_t lm1d2 = lm1 / 2;  // point interpolation

  if (space == OV7670_COLOR_RGB) {
    if (levels >= 32) {
      return;
    } else {
      // Good posterization requires a fair bit of fixed-point math.
      // Rather than repeat all those steps over every pixel, lookup tables
      // are generated first, and pixels are quickly filtered through these.
      // Green is a special case here -- with RGB565 colors, the extra bit
      // of green would make for posterization thresholds that are not
      // uniform and may have weird halos. So the input is decimated to
      // RGB555, posterized, and result scaled to RGB565.
      uint16_t rtable[32], gtable[32], rgb;
      uint8_t  btable[32];
      for(i=0; i<32; i++) { // 5 bits each
        btable[i] = (((i * levels + lm1d2) / 32) * 31 + lm1d2) / lm1;
        rtable[i] = btable[i] << 11;
        gtable[i] = (btable[i] << 6) | ((btable[i] & 0x10) << 1);
      }
      for (i=0; i<num_pixels; i++) {        // For each pixel...
        rgb = __builtin_bswap16(pixels[i]); // Data from camera is big-endian
        // Dismantle RGB into components, remap each through color table
        rgb = rtable[rgb >> 11] | gtable[(rgb >> 6) & 31] | btable[rgb & 31];
        pixels[i] = __builtin_bswap16(rgb); // Back to big-endian
      }
    }
  } else {
    if (levels == 255) {
      return;
    } else {
      uint8_t table[256];
      for(i=0; i<256; i++) {
        table[i] = (((i * levels + lm1d2) / 256) * 255 + lm1d2) / lm1;
      }
      uint8_t *p8 = (uint8_t *)pixels; // Separate Ys, Us, Vs
      num_pixels *= 2;                 // Actually num bytes now
      for (i=0; i<num_pixels; i++) {   // For each byte...
        p8[i] = table[p8[i]];          //   Remap through lookup table
      }
    }
  }
}

// Fractional tiles are on right and/or bottom edge(s)
void OV7670_image_mosaic(OV7670_colorspace space, uint16_t *pixels,
                         uint16_t width, uint16_t height, uint8_t tile_width,
                         uint8_t tile_height) {
  if ((tile_width <= 1) && (tile_height <= 1)) {
    return;
  }
  if(tile_width < 1) {
    tile_width = 1;
  }
  if(tile_height < 1) {
    tile_height = 1;
  }

  uint8_t tiles_across = (width + (tile_width - 1)) / tile_width;
  uint8_t tiles_down = (height + (tile_height - 1)) / tile_height;
  uint16_t tile_x, tile_y;
  uint16_t x1, x2, y1, y2, xx, yy; // Tile bounds, counters
  uint32_t pixels_in_tile;

  if (space == OV7670_COLOR_RGB) {
    uint16_t rgb;
    uint32_t red_sum, green_sum, blue_sum;
    uint32_t yy1, yy2;
    for (y1=tile_y=0; tile_y<tiles_down; tile_y++) { // Each tile row...
      y2 = y1 + tile_height - 1; // Last pixel row in current tile row
      if (y2 >= height) {        // Clip to bottom of image
        y2 = height - 1;
      }
      // Recalc this each tile row because tile x loop may alter it:
      pixels_in_tile = tile_width * (y2 - y1 + 1);
      for (x1=tile_x=0; tile_x<tiles_across; tile_x++) { // Each tile column...
        x2 = x1 + tile_width - 1; // Last pixel column in current tile column
        if (x2 >= width) { // Clip to right of image
          x2 = width - 1;
          pixels_in_tile = (x2 - x1 + 1) * (y2 - y1 + 1);
        }
        // Accumulate red, green, blue sums for all pixels in tile
        red_sum = green_sum = blue_sum = 0;
        yy2 = y1 * width; // Index of first pixel in tile
        for (yy = y1; yy <= y2; yy++) { // Each pixel row in tile...
          for (xx = x1; xx <= x2; xx++) { // Each pixel column in tile...
            rgb = __builtin_bswap16(pixels[yy2 + xx]);
            red_sum += rgb & 0b1111100000000000;   // Accumulate in-place,
            green_sum += rgb & 0b0000011111100000; // no shift down needed
            blue_sum += rgb & 0b0000000000011111;
          }
          yy2 += width; // Advance by one image row
        }
        red_sum = (red_sum / pixels_in_tile) & 0b1111100000000000;
        green_sum = (green_sum / pixels_in_tile) & 0b0000011111100000;
        blue_sum = (blue_sum / pixels_in_tile) & 0b0000000000011111;
        rgb = __builtin_bswap16(red_sum | green_sum | blue_sum);
        yy2 = y1 * width; // Index of first pixel in tile
        for (xx = x1; xx <= x2; xx++) { // Overwrite top row of tile
          pixels[yy2 + xx] = rgb;       // with averaged tile value
        }
        x1 += tile_width; // Advance pixel index by one tile column
      }
      // Duplicate scanlines to fill tiles on Y axis
      yy1 = y1 * width;  // Index of first pixel in tile
      yy2 = yy1 + width; // Index of pixel one row down
      for(yy=y1+1; yy<=y2; yy++) { // Each subsequent pixel row in tiles...
        memcpy(&pixels[yy2], &pixels[yy1], width * 2);
        yy2 += width; // Advance destination index by one row
      }
      y1 += tile_height; // Advance pixel index by one tile row
    }
  } else {
    // YUV is not handled yet because it's weird, with U & V
    // on alternating pixels. This will require Some Doing.
  }
}

void OV7670_image_edges(OV7670_colorspace space, uint16_t *pixels,
                        uint16_t width, uint16_t height, uint8_t sensitivity) {
}
