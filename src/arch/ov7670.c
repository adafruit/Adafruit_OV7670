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
// I2C calls)..
void OV7670_write_list(void *platform, OV7670_command *cmd) {
  for (int i = 0; cmd[i].reg <= OV7670_REG_LAST; i++) {
#if 0
    char buf[50];
    sprintf(buf, "Write reg %02X = %02X\n", cmd[i].reg, cmd[i].value);
    OV7670_print(buf);
#endif
    OV7670_write_register(platform, cmd[i].reg, cmd[i].value);
    OV7670_delay_ms(1); // Required, else lockup on init
  }
}

// CAMERA STARTUP ----------------------------------------------------------

static const OV7670_command OV7670_init[] = {
    {OV7670_REG_TSLB, OV7670_TSLB_YLAST},
    {OV7670_REG_COM15, OV7670_COM15_RGB565}, // Use 565 (not 555)
    {OV7670_REG_COM7, OV7670_COM7_SIZE_QVGA | OV7670_COM7_RGB},
    //    {OV7670_REG_COM7, OV7670_COM7_SIZE_QVGA | OV7670_COM7_RGB |
    //    OV7670_COM7_COLORBAR },
    {OV7670_REG_HREF, 0x80},
    {OV7670_REG_HSTART, 0x16},
    {OV7670_REG_COM10, OV7670_COM10_VS_NEG}, // Neg VSYNC (req by SAMD51 PCC)
    {OV7670_REG_HSTOP, 0x04},
    {OV7670_REG_VSTART, 0x03},
    {OV7670_REG_VSTOP, 0x7B},
    {OV7670_REG_VREF, 0x08},
    {OV7670_REG_COM3, OV7670_COM3_SCALEEN | OV7670_COM3_DCWEN},
    //    {OV7670_REG_COM3, OV7670_COM3_SCALEEN | OV7670_COM3_DCWEN |
    //    OV7670_COM3_SWAP},
    {OV7670_REG_COM14, 0x00},
    {OV7670_REG_SCALING_XSC, 0x00},
    {OV7670_REG_SCALING_YSC, 0x01},
    {OV7670_REG_SCALINGDCW, 0x11}, // Default scaling = 1:2
    {OV7670_REG_SCALINGPCLK, 0x09},
    {OV7670_REG_SPD, 0x02},
    {OV7670_REG_CLKRC, 0x01},
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
    {OV7670_REG_DBLV, 0x40},
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
    {OV7670_REG_CONTRAS_CTR, 0x80}, // 0x40?
    {OV7670_REG_LAST + 1, 0x00}, // End-of-data marker
};

OV7670_status OV7670_begin(OV7670_host_t *host) {
  OV7670_status status;

  // I2C must already be set up and running (@ 100 KHz) in calling code

  // Do device-specific (but platform-agnostic) setup. e.g. on SAMD this
  // function will fiddle registers to start a timer for XCLK output and
  // enable the parallel capture peripheral.
  status = OV7670_arch_begin(host);
  if(status != OV7670_STATUS_OK) {
    return status;
  }

  // Unsure of camera startup time from beginning of input clock.
  // Let's guess it's similar to tS:REG (300 ms) from datasheet.
  OV7670_delay_ms(300);

  // ENABLE AND/OR RESET CAMERA --------------------------------------------

  if (host->pin[OV7670_PIN_ENABLE] >= 0) { // Enable pin defined?
    OV7670_pin_output(host->pin[OV7670_PIN_ENABLE])
    OV7670_pin_write(host->pin[OV7670_PIN_ENABLE], 0); // PWDN low (enable)
    OV7670_delay_ms(300);
  }

  if (host->pin[OV7670_PIN_RESET] >= 0) { // Hard reset pin defined?
    OV7670_pin_output(host->pin[OV7670_PIN_RESET]);
    OV7670_pin_write(host->pin[OV7670_PIN_RESET], 0);
    OV7670_delay_ms(1);
    OV7670_pin_write(host->pin[OV7670_PIN_RESET], 1);
  } else { // Soft reset, doesn't seem reliable, might just need more delay?
    OV7670_write_register(host->platform, OV7670_REG_COM7, OV7670_COM7_RESET);
  }
  OV7670_delay_ms(1); // Datasheet: tS:RESET = 1 ms

  // Send initial configuration to camera
  OV7670_write_list(host->platform, OV7670_init);

  OV7670_delay_ms(300); // tS:REG = 300 ms (settling time = 10 frames)
}

