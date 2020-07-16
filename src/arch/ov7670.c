#include "ov7670.h"

// REQUIRED EXTERN FUNCTIONS -----------------------------------------------

// Outside code MUST provide these functions with the same arguments and
// return types (see end of Adafruit_OV7670.cpp for an example). These allow
// basic debug-print and I2C operations without referencing C++ objects or
// accessing specific peripherals from this code. That information is all
// contained in the higher-level calling code, which can set up the platform
// pointer for whatever objects and/or peripherals it needs. Yes this is
// ugly and roundabout, but it makes this part of the code more reusable
// (not having to re-implement for each platform/architecture) and the
// functions in question aren't performance-oriented anyway (typically used
// just once on startup, or during I2C transfers which are slow anyway).

extern void OV7670_print(char *str);
extern void OV7670_read_register(void *platform, uint8_t reg);
extern void OV7670_write_register(void *platform, uint8_t reg, uint8_t value);

// UTILITY FUNCTIONS -------------------------------------------------------

// Write a 0xFF-terminated list of commands to the camera.
// First argument is a pointer to platform-specific data...e.g. on Arduno,
// this points to a C++ object so we can find our way back to the correct
// I2C peripheral (so this code doesn't have to deal with platform-specific
// I2C calls).
void OV7670_write_list(void *platform, OV7670_command *cmd) {
  for (int i = 0; cmd[i].reg <= OV7670_REG_LAST; i++) {
#if 0 // DEBUG
    char buf[50];
    sprintf(buf, "Write reg %02X = %02X\n", cmd[i].reg, cmd[i].value);
    OV7670_print(buf);
#endif
    OV7670_write_register(platform, cmd[i].reg, cmd[i].value);
    OV7670_delay_ms(1); // Required, else lockup on init
  }
}

// CAMERA STARTUP ----------------------------------------------------------

static const OV7670_command
    OV7670_rgb[] =
        {
            // Manual output format, RGB, use RGB565 and full 0-255 output range
            {OV7670_REG_COM7, OV7670_COM7_RGB},
            {OV7670_REG_RGB444, 0},
            {OV7670_REG_COM15, OV7670_COM15_RGB565 | OV7670_COM15_R00FF},
            {0xFF, 0xFF}},
      OV7670_yuv[] =
        {
            // Manual output format, YUV, use full output range
            {OV7670_REG_COM7, OV7670_COM7_YUV},
            {OV7670_REG_COM15, OV7670_COM15_R00FF},
            {0xFF, 0xFF} },
      OV7670_init[] = {
        {OV7670_REG_TSLB, OV7670_TSLB_YLAST},    // No auto window
        {OV7670_REG_COM10, OV7670_COM10_VS_NEG}, // -VSYNC (req by SAMD PCC)
        {OV7670_REG_SLOP, 0x20},
        {OV7670_REG_GAM_BASE, 0x1C},
        {OV7670_REG_GAM_BASE + 1, 0x28},
        {OV7670_REG_GAM_BASE + 2, 0x3C},
        {OV7670_REG_GAM_BASE + 3, 0x55},
        {OV7670_REG_GAM_BASE + 4, 0x68},
        {OV7670_REG_GAM_BASE + 5, 0x76},
        {OV7670_REG_GAM_BASE + 6, 0x80},
        {OV7670_REG_GAM_BASE + 7, 0x88},
        {OV7670_REG_GAM_BASE + 8, 0x8F},
        {OV7670_REG_GAM_BASE + 9, 0x96},
        {OV7670_REG_GAM_BASE + 10, 0xA3},
        {OV7670_REG_GAM_BASE + 11, 0xAF},
        {OV7670_REG_GAM_BASE + 12, 0xC4},
        {OV7670_REG_GAM_BASE + 13, 0xD7},
        {OV7670_REG_GAM_BASE + 14, 0xE8},
        {OV7670_REG_COM8,
         OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BANDING},
        {OV7670_REG_GAIN, 0x00},
        {OV7670_COM2_SSLEEP, 0x00},
        {OV7670_REG_COM4, 0x00},
        {OV7670_REG_COM9, 0x20}, // Max AGC value
        {OV7670_REG_BD50MAX, 0x05},
        {OV7670_REG_BD60MAX, 0x07},
        {OV7670_REG_AEW, 0x75},
        {OV7670_REG_AEB, 0x63},
        {OV7670_REG_VPT, 0xA5},
        {OV7670_REG_HAECC1, 0x78},
        {OV7670_REG_HAECC2, 0x68},
        {0xA1, 0x03},              // Reserved register?
        {OV7670_REG_HAECC3, 0xDF}, // Histogram-based AEC/AGC setup
        {OV7670_REG_HAECC4, 0xDF},
        {OV7670_REG_HAECC5, 0xF0},
        {OV7670_REG_HAECC6, 0x90},
        {OV7670_REG_HAECC7, 0x94},
        {OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP |
                              OV7670_COM8_BANDING | OV7670_COM8_AGC |
                              OV7670_COM8_AEC},
        {OV7670_REG_COM5, 0x61},
        {OV7670_REG_COM6, 0x4B},
        {0x16, 0x02},            // Reserved register?
        {OV7670_REG_MVFP, 0x07}, // 0x07,
        {OV7670_REG_ADCCTR1, 0x02},
        {OV7670_REG_ADCCTR2, 0x91},
        {0x29, 0x07}, // Reserved register?
        {OV7670_REG_CHLF, 0x0B},
        {0x35, 0x0B}, // Reserved register?
        {OV7670_REG_ADC, 0x1D},
        {OV7670_REG_ACOM, 0x71},
        {OV7670_REG_OFON, 0x2A},
        {OV7670_REG_COM12, 0x78},
        {0x4D, 0x40}, // Reserved register?
        {0x4E, 0x20}, // Reserved register?
        {OV7670_REG_GFIX, 0x5D},
        {OV7670_REG_REG74, 0x19},
        {0x8D, 0x4F}, // Reserved register?
        {0x8E, 0x00}, // Reserved register?
        {0x8F, 0x00}, // Reserved register?
        {0x90, 0x00}, // Reserved register?
        {0x91, 0x00}, // Reserved register?
        {OV7670_REG_DM_LNL, 0x00},
        {0x96, 0x00}, // Reserved register?
        {0x9A, 0x80}, // Reserved register?
        {0xB0, 0x84}, // Reserved register?
        {OV7670_REG_ABLC1, 0x0C},
        {0xB2, 0x0E}, // Reserved register?
        {OV7670_REG_THL_ST, 0x82},
        {0xB8, 0x0A}, // Reserved register?
        {OV7670_REG_AWBC1, 0x14},
        {OV7670_REG_AWBC2, 0xF0},
        {OV7670_REG_AWBC3, 0x34},
        {OV7670_REG_AWBC4, 0x58},
        {OV7670_REG_AWBC5, 0x28},
        {OV7670_REG_AWBC6, 0x3A},
        {0x59, 0x88}, // Reserved register?
        {0x5A, 0x88}, // Reserved register?
        {0x5B, 0x44}, // Reserved register?
        {0x5C, 0x67}, // Reserved register?
        {0x5D, 0x49}, // Reserved register?
        {0x5E, 0x0E}, // Reserved register?
        {OV7670_REG_LCC3, 0x04},
        {OV7670_REG_LCC4, 0x20},
        {OV7670_REG_LCC5, 0x05},
        {OV7670_REG_LCC6, 0x04},
        {OV7670_REG_LCC7, 0x08},
        {OV7670_REG_AWBCTR3, 0x0A},
        {OV7670_REG_AWBCTR2, 0x55},
        {OV7670_REG_MTX1, 0x80},
        {OV7670_REG_MTX2, 0x80},
        {OV7670_REG_MTX3, 0x00},
        {OV7670_REG_MTX4, 0x22},
        {OV7670_REG_MTX5, 0x5E},
        {OV7670_REG_MTX6, 0x80}, // 0x40?
        {OV7670_REG_AWBCTR1, 0x11},
        {OV7670_REG_AWBCTR0, 0x9F}, // Or use 0x9E for advance AWB
        {OV7670_REG_BRIGHT, 0x00},
        {OV7670_REG_CONTRAS, 0x40},
        {OV7670_REG_CONTRAS_CENTER, 0x80}, // 0x40?
        {OV7670_REG_LAST + 1, 0x00},       // End-of-data marker
};

OV7670_status OV7670_begin(OV7670_host *host, OV7670_colorspace mode,
                           OV7670_size size, float fps) {
  OV7670_status status;

  // I2C must already be set up and running (@ 100 KHz) in calling code

  // Do device-specific (but platform-agnostic) setup. e.g. on SAMD this
  // function will fiddle registers to start a timer for XCLK output and
  // enable the parallel capture peripheral.
  status = OV7670_arch_begin(host);
  if (status != OV7670_STATUS_OK) {
    return status;
  }

  // Unsure of camera startup time from beginning of input clock.
  // Let's guess it's similar to tS:REG (300 ms) from datasheet.
  OV7670_delay_ms(300);

  // ENABLE AND/OR RESET CAMERA --------------------------------------------

  if (host->pins->enable >= 0) { // Enable pin defined?
    OV7670_pin_output(host->pins->enable)
        OV7670_pin_write(host->pins->enable, 0); // PWDN low (enable)
    OV7670_delay_ms(300);
  }

  if (host->pins->reset >= 0) { // Hard reset pin defined?
    OV7670_pin_output(host->pins->reset);
    OV7670_pin_write(host->pins->reset, 0);
    OV7670_delay_ms(1);
    OV7670_pin_write(host->pins->reset, 1);
  } else { // Soft reset, doesn't seem reliable, might just need more delay?
    OV7670_write_register(host->platform, OV7670_REG_COM7, OV7670_COM7_RESET);
  }
  OV7670_delay_ms(1); // Datasheet: tS:RESET = 1 ms

  (void)OV7670_set_fps(host->platform, fps); // Timing
  if (mode == OV7670_COLOR_RGB) {
    OV7670_write_list(host->platform, OV7670_rgb);
  } else {
    OV7670_write_list(host->platform, OV7670_yuv);
  }
  OV7670_write_list(host->platform, OV7670_init); // Other config
  OV7670_set_size(host->platform, size);          // Frame size

  OV7670_delay_ms(300); // tS:REG = 300 ms (settling time = 10 frames)
}

// Configure camera frame rate. Actual resulting frame rate (returned) may
// be different depending on available clock frequencies. Result will only
// exceed input if necessary for minimum supported rate, but this is very
// rare, typically below 1 fps. In all other cases, result will be equal
// or less than the requested rate, up to a maximum of 30 fps (the "or less"
// is because requested fps may be based on other host hardware timing
// constraints (e.g. screen) and rounding up to a closer-but-higher frame
// rate would be problematic). There is no hardcoded set of fixed frame
// rates because it varies with architecture, depending on OV7670_XCLK_HZ.
// If platform is NULL, no registers are set, a fps request/return can be
// evaluated without reconfiguring the camera, or without it even started.

float OV7670_set_fps(void *platform, float fps) {

  // Pixel clock (PCLK), which determines overall frame rate, is a
  // function of XCLK input frequency (OV7670_XCLK_HZ), a PLL multiplier
  // and then an integer division factor (1-32). These are the available
  // OV7670 PLL ratios:
  static const uint8_t pll_ratio[] = {1, 4, 6, 8};
  const uint8_t num_plls = sizeof pll_ratio / sizeof pll_ratio[0];

  // Constrain frame rate to upper and lower limits
  fps = (fps > 30) ? 30 : fps;               // Max 30 FPS
  float pclk_target = fps * 4000000.0 / 5.0; // Ideal PCLK Hz for target FPS
  uint32_t pclk_min = OV7670_XCLK_HZ / 32;   // Min PCLK determines min FPS
  if (pclk_target < (float)pclk_min) {       // If PCLK target is below limit
    if (platform) {
      OV7670_write_register(platform, OV7670_REG_DBLV, 0);   // 1:1 PLL
      OV7670_write_register(platform, OV7670_REG_CLKRC, 31); // 1/32 div
    }
    return (float)(pclk_min * 5 / 4000000); // Return min frame rate
  }

  // Find nearest available FPS without going over. This is done in a
  // brute-force manner, testing all 127 PLL-up by divide-down permutations
  // and tracking the best fit. I’m certain there are shortcuts but was
  // having trouble with my math, might revisit later. It's not a huge
  // bottleneck...MCUs are fast now, many cases are quickly discarded, and
  // this operation is usually done only once on startup (the I2C transfers
  // probably take longer).

  uint8_t best_pll = 0;    // Index (not value) of best PLL match
  uint8_t best_div = 1;    // Value of best division factor match
  float best_delta = 30.0; // Best requested vs actual FPS (init to "way off")

  for (uint8_t p = 0; p < num_plls; p++) {
    uint32_t xclk_pll = OV7670_XCLK_HZ * pll_ratio[p]; // PLL'd freq
    uint8_t first_div = p ? 2 : 1; // Min div is 1 for PLL 1:1, else 2
    for (uint8_t div = first_div; div <= 32; div++) {
      uint32_t pclk_result = xclk_pll / div; // PCLK-up-down permutation
      if (pclk_result > pclk_target) {       // Exceeds target?
        continue;                            //  Skip it
      }
      float fps_result = (float)pclk_result * 5.0 / 4000000.0;
      float delta = fps - fps_result; // How far off?
      if (delta < best_delta) {       // Best match yet?
        best_delta = delta;           //  Save delta,
        best_pll = p;                 //  pll and
        best_div = div;               //  div for later use
      }
    }
  }

  if (platform) {
    // Set up DBLV and CLKRC registers with best PLL and div values
    if (pll_ratio[best_pll] == best_div) { // If PLL and div are same (1:1)
      // Bypass PLL, use external clock directly
      OV7670_write_register(platform, OV7670_REG_DBLV, 0);
      OV7670_write_register(platform, OV7670_REG_CLKRC, 0x40);
    } else {
      // Set DBLV[7:6] for PLL, CLKRC[5:0] for div-1 (1-32 stored as 0-31)
      OV7670_write_register(platform, OV7670_REG_DBLV, best_pll << 6);
      OV7670_write_register(platform, OV7670_REG_CLKRC, best_div - 1);
    }
  }

  return fps - best_delta; // Return actual frame rate
}

// Sets up PCLK dividers and sets H/V start/stop window. Rather than
// rolling this into OV7670_set_size(), it's kept separate so test code
// can experiment with different settings to find ideal defaults.
void OV7670_frame_control(void *platform, uint8_t size, uint8_t vstart,
                          uint16_t hstart, uint8_t edge_offset, uint8_t delay) {
  uint8_t value;

  // Enable downsampling if sub-VGA, and zoom if 1:16 scale
  value = (size > OV7670_SIZE_DIV1) ? OV7670_COM3_DCWEN : 0;
  if (size == OV7670_SIZE_DIV16)
    value |= OV7670_COM3_SCALEEN;
  OV7670_write_register(platform, OV7670_REG_COM3, value);

  // Enable PCLK division if sub-VGA 2,4,8,16 = 0x19,1A,1B,1C
  value = (size > OV7670_SIZE_DIV1) ? (0x18 + size) : 0;
  OV7670_write_register(platform, OV7670_REG_COM14, value);

  // Horiz/vert downsample ratio, 1:8 max (H,V are always equal for now)
  value = (size <= OV7670_SIZE_DIV8) ? size : OV7670_SIZE_DIV8;
  OV7670_write_register(platform, OV7670_REG_SCALING_DCWCTR, value * 0x11);

  // Pixel clock divider if sub-VGA
  value = (size > OV7670_SIZE_DIV1) ? (0xF0 + size) : 0x08;
  OV7670_write_register(platform, OV7670_REG_SCALING_PCLK_DIV, value);

  // Apply 0.5 digital zoom at 1:16 size (others are downsample only)
  value = (size == OV7670_SIZE_DIV16) ? 0x40 : 0x20; // 0.5, 1.0
  OV7670_write_register(platform, OV7670_REG_SCALING_XSC, value);
  OV7670_write_register(platform, OV7670_REG_SCALING_YSC, value);

  // Window size is scattered across multiple registers.
  // Horiz/vert stops can be automatically calc'd from starts.
  uint16_t vstop = vstart + 480;
  uint16_t hstop = (hstart + 640) % 784;
  OV7670_write_register(platform, OV7670_REG_HSTART, hstart >> 3);
  OV7670_write_register(platform, OV7670_REG_HSTOP, hstop >> 3);
  OV7670_write_register(platform, OV7670_REG_HREF,
                        (edge_offset << 6) | ((hstop & 0b111) << 3) |
                            (hstart & 0b111));
  OV7670_write_register(platform, OV7670_REG_VSTART, vstart >> 2);
  OV7670_write_register(platform, OV7670_REG_VSTOP, vstop >> 2);
  OV7670_write_register(platform, OV7670_REG_VREF,
                        ((vstop & 0b11) << 2) | (vstart & 0b11));

  OV7670_write_register(platform, OV7670_REG_SCALING_PCLK_DELAY, delay);
}

void OV7670_set_size(void *platform, OV7670_size size) {
  // Array of five window settings, index of each (0-4) aligns with the five
  // OV7670_size enumeration values. If enum changes, list must change!
  static struct {
    uint8_t vstart;
    uint8_t hstart;
    uint8_t edge_offset;
    uint8_t pclk_delay;
  } window[] = {
      // Window settings were tediously determined empirically.
      // I hope there's a formula for this, if a do-over is needed.
      {9, 162, 2, 2},  // SIZE_DIV1  640x480 VGA
      {10, 174, 4, 2}, // SIZE_DIV2  320x240 QVGA
      {11, 186, 2, 2}, // SIZE_DIV4  160x120 QQVGA
      {12, 210, 0, 2}, // SIZE_DIV8  80x60   ...
      {15, 252, 3, 2}, // SIZE_DIV16 40x30
  };

  OV7670_frame_control(platform, size, window[size].vstart, window[size].hstart,
                       window[size].edge_offset, window[size].pclk_delay);
}

#if 0

#define OV7670_VGA_WIDTH 640
#define OV7670_VGA_HEIGHT 480
#define OV7670_CIF_WIDTH 352
#define OV7670_CIF_HEIGHT 288
#define OV7670_MIN_WIDTH (OV7670_VGA_WIDTH / 16)
#define OV7670_MIN_HEIGHT (OV7670_VGA_HEIGHT / 16)

// Given a desired image size (along either axis), constrain and return
// a value that the camera is actually capable of supporting.
// OV7670 image scaling uses a combination of integer power-of-two
// downsampling (1:1, 1:2, 1:4 or 1:8) and a fractional "digital zoom"
// from 1.0 (full size) to 0.5 (half size). Native full resolution from
// the sensor is 640x480 pixels (1:1 * 1.0 zoom), minimum resolution is
// 1/16 this (1:8 * 0.5 zoom) or 40x30 pixels. Due to internal buffer
// size limitations, the maximum zoomed image size is CIF (352x288).
// THEREFORE:
// - Any requested size EQUAL or BELOW THE MINIMUM will return the MINIMUM.
// - Any requested size EQUAL or ABOVE THE MAXIMUM will return the MAXIMUM.
// - Any requested size BELOW THE MAXIMUM but EQUAL or ABOVE THE CIF SIZE
//   LIMITS will return the CIF SIZE (e.g. 400 pixel width becomes 352) --
//   there is a gap in available sizes here, this is normal.
// - All other sizes below the CIF size and above the minimum will provide
//   the requested size.
// - Sizes BELOW MINIMUM will return the MINIMUM. Otherwise, result will
//   always be EQUAL OR SMALLER than the requested size, never larger.
// X and Y axes are independent; pixel aspect ratio may be non-square.
// Width can be 40-352 or 640, height can be 30-288 or 480.

// SUPPORTED SCALING RATIOS ARE 0x20 (1.0) to 0x40 (0.5).
// That's 33 distinct values, so NOT ALL PIXEL SIZES CAN EXIST.
// Need to factor that in.

static void size_axis(uint16_t *n, uint16_t cif, uint16_t max,
                      uint8_t *ds, uint8_t *ratio) {
  // Downsample value is a shift from 0 to 3. Actual factor is 2^(ds+1).
  *ds = 0;                         // 1:1 initial downsample guess
  float zoom;
  uint16_t min = max / 16;         // Limit is 1:8 downsample * 0.5 zoom
  if(*n <= min) {                  // If size is at or below minimum
    *n = min;                      //   Use min size
    *ds = 3;                       //   1:8 downsample
    zoom = 0.5;                    //   0.5 zoom
  } else if(*n >= max) {           // If size is at or above maximum
    *n = max;                      //   Use max size
    zoom = 1.0;                    //   Full-size zoom (ds 1:1 above)
  } else if(*n >= cif) {           // If size is at or above CIF limit
    *n = cif;                      //   Use CIF size
    zoom = (float)*n / (float)max; //   Zoom = 0.5 to <1.0
  } else {                         // All other sizes...
    while((*ds < 3) && (*n <= (max / (1 << (*ds + 1))))) ds++; // Downsample X2
    zoom = (float)*n / (float)(max / (1 << *ds));
  }
  // TO DO here: handle the limited zoom granularity
  // Not every size is supported, need to round slightly.
  // Change 'n' that's returned to fit these values
// iz = int(zoom * 33.0);
  // Ratio will be 0x20 (1.0) to 0x40 (0.5)
  *ratio = 0x20;
  // This will change the value of 'n'
}

// Failing to get this to work.
// Initial support will just be VGA powers-of-two sizing
// The canned settings in COM7 aren't especially useful here.
// There's QVGA, but no full-VGA or QQ or QQQ or QQQQ

// Pass in desired width & height (as pointers to uint16_t),
// values will be modified with actual resolution set.
bool OV7670_setResolution(uint16_t *width, uint16_t *height) {

  uint8_t ds_x, ds_y;       // Downsample factors for X & Y axes
  uint8_t ratio_x, ratio_y; // Zoom ratios for X & Y

  // Handle a few automatic sizing options, if width and/or height are 0:
  if(*width == 0) {                    // If width and
    if(*height == 0) {                 // height are both 0...
      *width = OV7670_VGA_WIDTH / 4;   //   Use default size
      *height = OV7670_VGA_HEIGHT / 4; //   (1:4 downsampling)
    } else {                           // If only width is 0...
      *width = (*height * 8 + 3) / 6;  //   Auto width based on height @ 4:3
    } // The 8,3,6 above are for fixed-point +0.5 rounding
  } else if(*height == 0) {            // If only height is 0...
    *height = (*width * 3 + 2) / 4;    //   Auto height based on width @ 4:3
  } // Ditto w 3,2,4 for fixed-point +0.5 rounding

  // Apply upper & lower pixel size constraints to X & Y axes.
  size_axis(*width, OV7670_CIF_WIDTH, OV7670_VGA_WIDTH, &ds_x, &ratio_x);
  size_axis(*height, OV7670_CIF_HEIGHT, OV7670_VGA_HEIGHT, &ds_y, &ratio_y);

#if defined(__SAMD51__)
  // Because the SAMD51 PCC config accumulates 4 bytes in the RHR register
  // (two 16-bit pixels), width * height must be a multiple of 2...
  if((*width * *height) & 1) { // Odd pixel count
    // One axis or other will be reduced by one pixel.
    // If width or height is already at its minimum, choose the opposite
    // axis. We can safely do this because we know the X and Y minimums
    // are both even numbers...if the opposite axis is reduced to an odd
    // value, the product of the two is still even. If both axes are at
    // their minimums, we won't be here, it passes the if() test above.
    if(*width == OV7670_MIN_WIDTH) {          // Width already at minimum,
      *height &= ~1;                          // reduction must be to height
    } else if(*height == OV7670_MIN_HEIGHT) { // Height already at minimum,
      *width &= ~1;                           // reduction must be to width
    } else {
      // Neither axis at minimum size. Determine which axis yields a
      // result closer to 4:3. Always rounds one axis down, never up.
      float test_w = fabs(((float)*height / (float)(*width & ~1)) - 0.75);
      float test_h = fabs(((float)(*height & ~1) / (float)*width) - 0.75);
      if(test_w <= test_h) {
        *width &= ~1;
      } else {
        *height &= ~1;
      }
    }
    // Make another pass through the axis-sizing functions. Dimensions
    // won't change now, but we need revised downsample & zoom values.
    size_axis(*width, OV7670_CIF_WIDTH, OV7670_VGA_WIDTH, &ds_x, &ratio_x);
    size_axis(*height, OV7670_CIF_HEIGHT, OV7670_VGA_HEIGHT, &ds_y, &ratio_y);
  }
#endif // __SAMD51__

  // DO MAGIC HERE, issue stuff to camera
  // Will also require reallocing buffer
  // Maybe that's done in the calling code
}



NOTES:
Although the camera can output data in various formats,
this lib will ALWAYS use RGB565 format.
COM7[2] = 1, COM7[0] = 0, COM15[5] = 0, COM15[4] = 1
In RGB565 format, it supports VGA and any resolution below CIF.
If manually setting a resolution (COM14[3] and SCALING_PCLK_DELAY[7] must
ne set), the output window settings must be adjusted (HSTART, HSTOP, HREF,
VSTRT, VSTOP, VREF. Register TSLB[0] must be 0 for this to happen.
Sensor array always outputs VGA, anything below that is scaled.
Mirroring is supported via MVFP[5] and MVFP[4].

Input clock is multipled by a PLL multiplier (DBLV[7:6]), then divided by a
pre-scaler (CLKRC[5:0]) to produce the internal clock frequency.
Frame rate is adjusted by the clock pre-scaler. Max VGA frame rate is 30 fps.
PCLK is the pixel clock.
Frame rate can also be adjusted by inserting dummy pixels in the
horizontal and vertical blanking periods, keeping PCLK the same?
There's an automatic frame rate setting as well (sets up dummy rows/pixels).

Exposure time is a function of row interval. Longer exposures are generally
good, so running at max frame rate might not always be the best. Might want
to offer other rates.

PIXEL CLOCK DIVIDER - this ONLY applies to HORIZONTAL scaling factor
1 to 1/2x   (640 to 321)  1 pixel clock/byte   SCALING_PCLK_DIV[3:0] = 0
1/2 to 1/4  (320 to 161)  2 pixel clocks/byte  SCALING_PCLK_DIV[3:0] = 1
1/4 to 1/8  (160 to 81)   4 pixel clocks/byte  SCALING_PCLK_DIV[3:0] = 2
1/8 to 1/16 (80 to 40)    8 pixel clocks/byte  SCALING_PCLK_DIV[3:0] = 3
Since this mentions 1/16x, this implies the combination of both
downsampling and digital zoom. But I think just the downsample
factor is enough to identify what's needed for the DIV, right?

Currently at 160, I'm right at the limit where either 2 or 4 works.

#endif // 0
