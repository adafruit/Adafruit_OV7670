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

// 3X3 MEDIAN FILTER --------------------------------------------------------

// A median filter helps reduce pixel "snow" in an image while keeping
// larger features intact. It's sometimes helpful before edge detection.
// The process is pretty heinous (every pixel in an image is reassigned
// the median value of itself and 8 neighboring pixels -- separately for
// red, green and blue). The implementation here duplicates edge pixels
// so output size matches input size (no black border or other uglies).
// A few Clever Tricks(tm) are employed to try to speed up the median
// calculations...
// - A buffer is allocated and three rows of the image are unpacked from
//   RGB565 format into separate red, green and blue channels, to reduce
//   the number of bit-masking and shifting operations needed when
//   comparing colors in inner loops. The 3 rows cycle while working down
//   the image -- the 'current' row becomes the 'prior' row, the 'next'
//   row becomes the 'current' row, and a new 'next' row is converted.
//   Requires about 3.6K RAM overhead for a 320x240 pixel image.
// - The format of these rows is Truly Strange, to help exploit spatial
//   coherence. For any two adjacent pixels, six of nine values used in
//   median determination are the same. Still requires evaluating all
//   nine pixels, but we don't have to load up all nine bins every time.
//   Instead of the three rows being in the typical row-major format:
//      0  1  2  3  4  5
//      6  7  8  9 10 11
//     12 13 14 15 16 17
//   They're instead in a column-major format, always three rows:
//      0  3  6  9 12 15
//      1  4  7 10 13 16
//      2  5  8 11 14 17
//   Loading up the 3x3 median list while increasing X by 1 along the row
//   then just involves incrementing an index or pointer by 3's...the data
//   stays in-place, rather than individually copying 3x3 pixels to a
//   working buffer.
//   Weirder though...to increment to the next row, rather than moving the
//   'current' row data to the 'prior' row and the 'next' row data into the
//   'current' row (or keeping a circular set of 3 pointers/indices), the
//   pixel buffer has a 'tail' such that we only need to increment the
//   buffer start index by 1, then reload the new 'next' row data (with
//   the pixel indices incrementing by 3).
//   If it helps to visualize this, picture you have a string coiled around
//   a wooden rod. The rod has a circumference of exactly 3 pixels...so,
//   coiling the string around it, all the pixels in each of the 3 rows are
//   aligned, and the horizontal pixel positions in the buffer will
//   increment by 3's. Turning the rod 1/3 turn (1 pixel) cycles all the
//   rows, and a new row can be loaded affecting just those pixels (by 3's).
//   The 'head' and 'tail' on this string (equal to the image height) let
//   us make the vertical change without having to move or copy any
//   existing data to new locations, just load the new bytes. That's why
//   the buffer allocation is somewhat more than (width * 3).
//
//         0 <- first byte in buffer
//         |
//       __|__/_/_/_/_/|__
//      |____/_/_/_/_/____|     -> +X = increment by 3
//         |/ / / / /  |      |
//                     |      V    +y = increment by 1
//                     |
//                     N <- last byte in buffer
//
//
// - Finding the median in a 9-element (3x3) list doesn't require sorting
//   the list. It's sufficient to identify the maximum of the least five
//   values, or minimum of largest five, same result all around.
//
// Thank you for coming to my TED Talk.

// Preprocess one row of pixels in preparation for median filter. Input
// pixels are separated into red, green, blue buffers for quicker access
// during the median calculations (avoids masking/shifting every time a
// pixel is accessed for comparison). Destination buffer has 2 extra
// pixels (1 ea. left and right), as edge pixels are duplicated to allow
// median to operate on all source image pixels, no black border or other
// uglies. Pixels within each channel are not sequential in memory, but
// increment by 3's -- corresponding to the prior, current and next rows.
static void OV7670_median_row_prep(uint16_t *src, uint8_t *r_dst,
                                   uint16_t width, uint32_t channel_bytes) {
  uint8_t *g_dst = &r_dst[channel_bytes];
  uint8_t *b_dst = &g_dst[channel_bytes];

  uint16_t x, rgb, offset = 3;
  for (x = 0; x < width; x++) {        // For each pixel in row...
    rgb = __builtin_bswap16(*src++);   // Packed RGB565 pixel
    r_dst[offset] = rgb >> 11;         // Extract 5 bits red,
    g_dst[offset] = (rgb >> 5) & 0x3F; // 6 bits green,
    b_dst[offset] = rgb & 0x1F;        // 5 bits blue
    offset += 3;
  }
  r_dst[0] = r_dst[3]; // Duplicate leftmost pixel
  g_dst[0] = g_dst[3];
  b_dst[0] = b_dst[3];
  x = offset - 3;
  r_dst[offset] = r_dst[x]; // Duplicate rightmost pixel
  g_dst[offset] = g_dst[x];
  b_dst[offset] = b_dst[x];
}

// Copy a single row in the median code's weird increment-by-3 pixel format.
static void OV7670_median_row_copy(uint8_t *r_src, uint8_t *r_dst,
                                   uint16_t width, uint32_t channel_bytes) {
  uint8_t *g_src = &r_src[channel_bytes];
  uint8_t *b_src = &g_src[channel_bytes];
  uint8_t *g_dst = &r_dst[channel_bytes];
  uint8_t *b_dst = &g_dst[channel_bytes];
  uint16_t x, offset;

  for (x = offset = 0; x < width; x++, offset += 3) {
    r_dst[offset] = r_src[offset];
    g_dst[offset] = g_src[offset];
    b_dst[offset] = b_src[offset];
  }
}

// Determine median value of 9-element list
static inline uint8_t OV7670_med9(uint8_t *list) {

  // Testing a couple of approaches here...

#if 0 // Might be a tiny bit faster at higher optimization settings

  // Find median value in a list of 9 (index 0 to 8). Ostensibly,
  // conceptually, this would involve sorting the list (ascending or
  // descending, doesn't matter) and returning the value at index 4.
  // Reality is simpler, it's sufficient to take the max of the 5
  // lowest values (or min of the 5 highest), same result. To help
  // exploit spatial coherence in the image (6 of the 9 values will
  // carry over from one pixel to the next), we don't want to sort or
  // modify the list. Making a copy of the list is one option.
  // Different option, used here, is to maintain a bitmask of all 9
  // 'available' list positions and clear bits as each of the 5
  // minimums is found (returning the last one).

  uint16_t min;
  uint8_t min_idx, i, n;
  uint16_t active_mask = ~0; // Bitmask of active values (initially all set)
  for (n = 0; n < 5; n++) {  // Make 5 passes...
    min = ~0;                //   To force initial select of 1st active value
    for(i = 0; i < 9; i++) { //   For each of 9 values...
      // If value #i is less than min and that spot's still active...
      if((list[i] < min) && (active_mask & (1 << i))) {
        min = list[i];       //     Save value of new minimum
        min_idx = i;         //     Save index of new minimum
      }
    }
    active_mask &= ~(1 << min_idx); // Take element min_idx out of running
  }

  return min;

#else // Might be faster at 'normal' optimization settings

  // Ostensibly, conceptually, this would involve sorting the 9-element
  // list (ascending or descending, doesn't matter) and returning the value
  // at index 4. Reality is simpler, it's sufficient to take the max of the
  // 5 lowest values (or min of the 5 highest), same result.

  // A copy of the input list is made so elements can be shuffled
  // without corrupting the original -- must maintain that in order
  // to exploit spatial coherence in the median loop.
  uint8_t buf[9];
  memcpy(buf, list, 9);

  uint8_t n, i, min, min_idx;

  for (n = 0; n < 5; n++) {       // 5 min-finding passes
    min = buf[n];                 //   Initial min guess...
    min_idx = n;                  //   is at index 'n' (0-4)
    for (i = n + 1; i < 9; i++) { //   Compare through end of list...
      if (buf[i] < min) {         //     Keep track of
        min = buf[i];             //     min value and
        min_idx = i;              //     index of min in list
      }
    }
    // Item at min_idx is the nth-least value in list and no longer in
    // the running for subsequent passes...replace value currently in
    // that position with the value at position n (no swap needed, just
    // overwrite, because next pass starts at n+1). In some cases this
    // moves a value to itself, that's OK.
    buf[min_idx] = buf[n];
  }

  return min; // min of 5th pass (max of 5 least values)

#endif // end A/B testing
}

// 3x3 median filter for noise reduction. Even with Clever Optimizations(tm)
// this is a tad slow, it's just the nature of the thing...lots and lots and
// lots of pixel comparisons. Requires a chunk of RAM temporarily,
// ((width + 2) * 3 + height - 1) * 3 bytes, or about 3.6K for a 320x240
// RGB image. YUV is not currently supported.
void OV7670_image_median(OV7670_colorspace space, uint16_t *pixels,
                         uint16_t width, uint16_t height) {
  if (space == OV7670_COLOR_RGB) {
    uint8_t *buf;
    uint32_t buf_bytes_per_channel = (width + 2) * 3 + height - 1;
    if ((buf = (uint8_t *)malloc(buf_bytes_per_channel * 3))) {
      uint8_t *rptr = buf;                          // -> red buffer
      uint8_t *gptr = &rptr[buf_bytes_per_channel]; // -> green buffer
      uint8_t *bptr = &gptr[buf_bytes_per_channel]; // -> blue buffer

      // For each of the three channel pointers (rptr, gptr, bptr),
      // ptr[0] is the first pixel of the row ABOVE the current one,
      // ptr[1] is the first pixel of the current row (0 to height-1),
      // ptr[2] is the first pixel of the row BELOW the current one.
      // Horizontal pixel addresses then increment by 3's...for each
      // column (x) in row, pixel x = ptr[x * 3 + n], where n is 0, 1, 2
      // for the above, current, and below rows, respectively.

      // Convert pixel data into the initial 'current' (1) row buf
      OV7670_median_row_prep(pixels, &rptr[1], width, buf_bytes_per_channel);

      // Copy pixel data from the initial (1) row to the prior (0) row buf
      // (Because edge pixels are repeated so we can 3x3 filter full image)
      OV7670_median_row_copy(&rptr[1], rptr, width + 2, buf_bytes_per_channel);

      uint16_t *ptr = pixels; // Dest pointer, back into source image
      uint16_t x, y, offset, rgb;
      uint8_t r_med, g_med, b_med;
      for (y = 0; y < height; y++) { // For each row of image...
        // Set up 'below' row buffer...
        if (y < (height - 1)) { // If current row is 0 to height-2
          // Convert pixel data into the 'next' (2) row buf
          OV7670_median_row_prep(&pixels[(y + 1) * width], &rptr[2], width,
                                 buf_bytes_per_channel);
        } else { // Last row, y = height-1
          // Copy pixel data from current (1) row to next (2) row buf
          // (Edge pixels are repeated so we can 3x3 filter full image)
          OV7670_median_row_copy(&rptr[1], &rptr[2], width + 2,
                                 buf_bytes_per_channel);
        }

        for (x = offset = 0; x < width; x++, offset += 3) { // Each column...
          r_med = OV7670_med9(&rptr[offset]);               // 3x3 median red
          g_med = OV7670_med9(&gptr[offset]);               // " green
          b_med = OV7670_med9(&bptr[offset]);               // " blue
          rgb = (r_med << 11) | (g_med << 5) | b_med;       // Recombine 565
          *ptr++ = __builtin_bswap16(rgb);                  // back in image
        }
        rptr++; // Next row
        gptr++;
        bptr++;
      }

      free(buf);
    }
  } else { // YUV
    // Not yet supported. Tricky because of alternating U/V pixels.
  }
}

// EDGE DETECTION -----------------------------------------------------------

// WIP.
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