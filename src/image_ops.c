#include "image_ops.h"

// These functions are preceded by "OV7670" even though they're not tied to
// the camera hardware, just that they're part of this lib. These are not
// in-camera effects, though some might be possible to implement as such.

// Negative image (avoiding 'invert' terminology as that could be confused
// for an image flip operation, which is a different function).
void OV7670_image_negative(uint16_t *pixels, uint16_t width, uint16_t height) {
  // Working 32 bits at a time is slightly faster. This is dirty pool,
  // relying on the fact that the camera lib currently only supports
  // even image sizes (the pixel count will always be a multiple of 2)
  // and that the image buffer, coming from malloc(), will always be
  // on a 32-bit-safe boundary. This is one of those operations that
  // can probably be implemented through the camera's gamma curve
  // settings, and if so this function will go away.
  uint32_t *p32 = (uint32_t *)pixels;
  uint32_t i, num_pairs = width * height / 2;
  for (i = 0; i < num_pairs; i++) {
    p32[i] ^= 0xFFFFFFFF;
  }
}

// Binary threshold, output is "black and white" per-channel. Pass in
// threshold level as 0-255, this will be quantized to an appropriate
// range for the colorspace.
void OV7670_image_threshold(OV7670_colorspace space, uint16_t *pixels,
                            uint16_t width, uint16_t height,
                            uint8_t threshold) {
  uint32_t i, num_pixels = width * height;
  if (space == OV7670_COLOR_RGB) {
    // Testing RGB thresholds "in place" in the packed RGB565 value
    // avoids some bit-shifting on every pixel (just bit masking).
    uint16_t rlimit = (threshold >> 3) << 11;  // In-place 565 red threshold
    uint16_t glimit = (threshold >> 2) << 5;   // In-place 565 green threshold
    uint16_t blimit = (threshold >> 3);        // In-place 565 blue threshold
    uint16_t rgb565in, rgb565out;              // Packed RGB565 pixel values
    for (i = 0; i < num_pixels; i++) {         // For each pixel...
      rgb565in = __builtin_bswap16(pixels[i]); //   Swap endian from cam
      rgb565out = 0;                           //   Start with 0 result
      if ((rgb565in & 0xF800) >= rlimit) {     //   If red exceeds limit
        rgb565out |= 0xF800;                   //     Set all red bits
      }
      if ((rgb565in & 0x07E0) >= glimit) { //   Ditto, green
        rgb565out |= 0x07E0;
      }
      if ((rgb565in & 0x001F) >= blimit) { //   Ditto, blue
        rgb565out |= 0x001F;
      }
      pixels[i] = __builtin_bswap16(rgb565out); //   Back to cam-native endian
    }
  } else {                                    // YUV...
    uint8_t *p8 = (uint8_t *)pixels;          // Separate Y's, U's, V's
    num_pixels *= 2;                          // Actually num bytes now
    for (i = 0; i < num_pixels; i++) {        // For each byte...
      p8[i] = (p8[i] >= threshold) ? 255 : 0; //   Threshold to 0 or 255
    }
  }
}

// Reduce color fidelity to a specified number of steps or levels.
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
      uint8_t btable[32];
      for (i = 0; i < 32; i++) { // 5 bits each
        btable[i] = (((i * levels + lm1d2) / 32) * 31 + lm1d2) / lm1;
        rtable[i] = btable[i] << 11;
        gtable[i] = (btable[i] << 6) | ((btable[i] & 0x10) << 1);
      }
      for (i = 0; i < num_pixels; i++) {    // For each pixel...
        rgb = __builtin_bswap16(pixels[i]); // Data from camera is big-endian
        // Dismantle RGB into components, remap each through color table
        rgb = rtable[rgb >> 11] | gtable[(rgb >> 6) & 31] | btable[rgb & 31];
        pixels[i] = __builtin_bswap16(rgb); // Back to big-endian
      }
    }
  } else { // YUV
    if (levels == 255) {
      return;
    } else {
      uint8_t table[256];
      for (i = 0; i < 256; i++) {
        table[i] = (((i * levels + lm1d2) / 256) * 255 + lm1d2) / lm1;
      }
      uint8_t *p8 = (uint8_t *)pixels;   // Separate Ys, Us, Vs
      num_pixels *= 2;                   // Actually num bytes now
      for (i = 0; i < num_pixels; i++) { // For each byte...
        p8[i] = table[p8[i]];            //   Remap through lookup table
      }
    }
  }
}

// Shower door effect.
void OV7670_image_mosaic(OV7670_colorspace space, uint16_t *pixels,
                         uint16_t width, uint16_t height, uint8_t tile_width,
                         uint8_t tile_height) {
  if ((tile_width <= 1) && (tile_height <= 1)) {
    return;
  }
  if (tile_width < 1) {
    tile_width = 1;
  }
  if (tile_height < 1) {
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
    for (y1 = tile_y = 0; tile_y < tiles_down; tile_y++) { // Each tile row...
      y2 = y1 + tile_height - 1; // Last pixel row in current tile row
      if (y2 >= height) {        // Clip to bottom of image
        y2 = height - 1;
      }
      // Recalc this each tile row because tile x loop may alter it:
      pixels_in_tile = tile_width * (y2 - y1 + 1);
      for (x1 = tile_x = 0; tile_x < tiles_across;
           tile_x++) {            // Each tile column...
        x2 = x1 + tile_width - 1; // Last pixel column in current tile column
        if (x2 >= width) {        // Clip to right of image
          x2 = width - 1;
          pixels_in_tile = (x2 - x1 + 1) * (y2 - y1 + 1);
        }
        // Accumulate red, green, blue sums for all pixels in tile
        red_sum = green_sum = blue_sum = 0;
        yy2 = y1 * width;                 // Index of first pixel in tile
        for (yy = y1; yy <= y2; yy++) {   // Each pixel row in tile...
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
        yy2 = y1 * width;               // Index of first pixel in tile
        for (xx = x1; xx <= x2; xx++) { // Overwrite top row of tile
          pixels[yy2 + xx] = rgb;       // with averaged tile value
        }
        x1 += tile_width; // Advance pixel index by one tile column
      }
      // Duplicate scanlines to fill tiles on Y axis
      yy1 = y1 * width;  // Index of first pixel in tile
      yy2 = yy1 + width; // Index of pixel one row down
      for (yy = y1 + 1; yy <= y2;
           yy++) { // Each subsequent pixel row in tiles...
        memcpy(&pixels[yy2], &pixels[yy1], width * 2);
        yy2 += width; // Advance destination index by one row
      }
      y1 += tile_height; // Advance pixel index by one tile row
    }
  } else { // YUV
    // YUV is not handled yet because it's weird, with U & V
    // on alternating pixels. This will require Some Doing.
  }
}

// Preprocess one scanline in preparation for median filter. Input pixels
// are separated into red, green, blue components for quicker access during
// the median calculations. Destination buffer has 2 extra/ pixels (1 ea.
// left and right), edge pixels are duplicated to allow median to operate
// on all image pixels, no black border or other uglies.
static void OV7670_median_row_prep(uint16_t *src, uint8_t *r_dst,
                                   uint16_t width) {
  uint8_t *g_dst = &r_dst[width + 2]; // +2 for left & right pixels
  uint8_t *b_dst = &g_dst[width + 2];
  uint16_t x, rgb;
  for (x = 1; x <= width; x++) {      // Index in dest bufs (skip 1st, last)
    rgb = __builtin_bswap16(*src++);  // Packed RGB565 pixel
    r_dst[x] = rgb >> 11;             // Extract 5 bits red,
    g_dst[x] = (rgb >> 5) & 0x3F;     // 6 bits green,
    b_dst[x] = rgb & 0x1F;            // 5 bits blue
  }
  r_dst[0] = r_dst[1];                // Duplicate leftmost pixel
  g_dst[0] = g_dst[1];
  b_dst[0] = b_dst[1];
  r_dst[width + 1] = r_dst[width];    // Duplicate rightmost pixel
  g_dst[width + 1] = g_dst[width];
  b_dst[width + 1] = b_dst[width];
}

// 3x3 median filter. WIP.
void OV7670_image_median(OV7670_colorspace space, uint16_t *pixels,
                         uint16_t width, uint16_t height) {
  if (space == OV7670_COLOR_RGB) {
    // Working buffer for three pixel rows: above, current, and below.
    // Each scanline has an extra pixel left and right, and are separated
    // into red, green and blue components so we don't need to do bit
    // masking/shifting every time a pixel is accessed.
    uint8_t *buf;
    if ((buf = (uint8_t *)malloc((width + 2) * 3 * 3))) {
      uint16_t y;

      for(y = 0; y < height; y++) {
        // DO MEDIAN MAGIC HERE
      }

      free(buf);
    }
  } else { // YUV
  }
}

// Edge detection. WIP.
void OV7670_image_edges(OV7670_colorspace space, uint16_t *pixels,
                        uint16_t width, uint16_t height, uint8_t sensitivity) {
  uint16_t x, y, a, b, foo;
  uint16_t buf[320];
  int32_t rgbleft, rgbright, rgb;

  // To do: buffer 3 lines of image, cycle through them,
  // Perform edge math on both horizontal and vertical axes

#if 0
  if (space == OV7670_COLOR_RGB) {
    uint32_t bufpixels = (width + 2) * 3;
    uint16_t *buf = (uint16_t *)malloc(bufpixels * sizeof(uint16_t));
    if (buf) {
      for(uint32_t i=0; i<bufpixels; i++) {
        buf[i] = 0b0111101111101111; // Neutral 50% brightness
      }
      uint16_t *bptr[3]; // trailing, current, leading pointers
      bptr[0] = buf;
      bptr[1] = &buf[width + 2];
      bptr[2] = &buf[(width + 2) * 2];
      memcpy(&bptr[1][1], pixels, width * 2);
      memcpy(&bptr[2][1], &pixels[width], width * 2);

      for(uint16_t y=0; y<height; y++) {
        // memcpy row in, or fill with neutral color if last row
        if (y < (height - 1)) {
          // memcpy
        } else {
          // fill
        }

        // DO EDGE MAGIC HERE

        uint16_t *tmp = bptr[0];
        bptr[0] = btr[1];
        bptr[1] = bptr[2];
        bptr[2] = tmp;
      }
      free(buf);
    }
  } else { // YUV
  }
#endif

  // Nasty kludgey test WIP
  // Do not optimize above "fast" (-O2)

  for (y = 0; y < height; y++) {
    memcpy(buf, &pixels[y * width], width * 2);
    for (x = 1; x < width - 1; x++) {
      rgbleft = (int32_t)__builtin_bswap16(buf[x - 1]);
      rgb = (int32_t)__builtin_bswap16(buf[x]);
      rgbright = (int32_t)__builtin_bswap16(buf[x + 1]);
      foo = 0;
      a = abs((rgbleft >> 11) - (rgb >> 11));
      b = abs((rgbright >> 11) - (rgb >> 11));
      if ((a >= 4) || (b >= 4)) {
        foo |= 0b1111100000000000;
      }
      a = abs(((rgbleft >> 5) & 63) - ((rgb >> 5) & 63));
      b = abs(((rgbright >> 5) & 63) - ((rgb >> 5) & 63));
      if ((a >= 8) || (b >= 8)) {
        foo |= 0b0000011111100000;
      }
      a = abs((rgbleft & 31) - (rgb & 31));
      b = abs((rgbright & 31) - (rgb & 31));
      if ((a >= 4) || (b >= 4)) {
        foo |= 0b0000000000011111;
      }
      pixels[y * width + x] = __builtin_bswap16(foo);
    }
  }
}
