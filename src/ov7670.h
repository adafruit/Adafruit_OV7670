#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#define OV7670_delay_ms(x) delay(x)
#define OV7670_pin_output(pin) pinMode(pin, OUTPUT);
#define OV7670_pin_write(pin, hi) digitalWrite(pin, hi ? 1 : 0)
#define OV7670_disable_interrupts() noInterrupts()
#define OV7670_enable_interrupts() interrupts()
#else
#include <stdint.h>
// If platform provides device-agnostic functions for millisecond delay,
// set-pin-to-output, pin-write and/or interrupts on/off, those can be
// #defined here as with Arduino above. If platform does NOT provide some
// or all of these, they should go in the device-specific .c file with a
// !defined(ARDUINO) around them.
#endif // end platforms

// IMPORTANT: #include ALL of the arch-specific .h files here.
// They have #ifdef checks to only take effect on the active architecture.
#include "arch/samd51.h"
#include "arch/rp2040.h"

/** Status codes returned by some functions */
typedef enum {
  OV7670_STATUS_OK = 0,         ///< Success
  OV7670_STATUS_ERR_MALLOC,     ///< malloc() call failed
  OV7670_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} OV7670_status;

/** Supported color formats */
typedef enum {
  OV7670_COLOR_RGB = 0, ///< RGB565 big-endian
  OV7670_COLOR_YUV,     ///< YUV/YCbCr 4:2:2 big-endian
} OV7670_colorspace;

/** Supported sizes (VGA division factor) for OV7670_set_size() */
typedef enum {
  OV7670_SIZE_DIV1 = 0, ///< 640 x 480
  OV7670_SIZE_DIV2,     ///< 320 x 240
  OV7670_SIZE_DIV4,     ///< 160 x 120
  OV7670_SIZE_DIV8,     ///< 80 x 60
  OV7670_SIZE_DIV16,    ///< 40 x 30
} OV7670_size;

typedef enum {
  OV7670_TEST_PATTERN_NONE = 0,       ///< Disable test pattern
  OV7670_TEST_PATTERN_SHIFTING_1,     ///< "Shifting 1" pattern
  OV7670_TEST_PATTERN_COLOR_BAR,      ///< 8 color bars
  OV7670_TEST_PATTERN_COLOR_BAR_FADE, ///< Color bars w/fade to white
} OV7670_pattern;

typedef enum {
  OV7670_NIGHT_MODE_OFF = 0, ///< Disable night mode
  OV7670_NIGHT_MODE_2,       ///< Night mode 1/2 frame rate
  OV7670_NIGHT_MODE_4,       ///< Night mode 1/4 frame rate
  OV7670_NIGHT_MODE_8,       ///< Night mode 1/8 frame rate
} OV7670_night_mode;

/**
Defines physical connection to OV7670 camera, passed to constructor.
On certain architectures, some of these pins are fixed in hardware and
cannot be changed, e.g. SAMD51 has its Parallel Capture Controller in
one specific place. In such cases, you can usually avoid declaring those
values when setting up the structure. However, elements DO need to be
declared in-order matching the structure and without gaps, and struct
elements have been sorted with this in mind. You can cut off a declaration
early, but if middle elements aren't needed must still be assigned some
unused value (e.g. 0). On Arduino platform, SDA/SCL are dictated by the
Wire instance and don't need to be set here. See example code.
*/
typedef struct {
  OV7670_pin enable;  ///< Also called PWDN, or set to -1 and tie to GND
  OV7670_pin reset;   ///< Cam reset, or set to -1 and tie to 3.3V
  OV7670_pin xclk;    ///< MCU clock out / cam clock in
  OV7670_pin pclk;    ///< Cam clock out / MCU clock in
  OV7670_pin vsync;   ///< Also called DEN1
  OV7670_pin hsync;   ///< Also called DEN2
  OV7670_pin data[8]; ///< Camera parallel data out
  OV7670_pin sda;     ///< I2C data
  OV7670_pin scl;     ///< I2C clock
} OV7670_pins;

/** Address/value combo for OV7670 camera commands. */
typedef struct {
  uint8_t reg;   ///< Register address
  uint8_t value; ///< Value to store
} OV7670_command;

/** Architecture+platform combination structure. */
typedef struct {
  OV7670_arch *arch; ///< Architecture-specific config data
  OV7670_pins *pins; ///< Physical connection to camera
  void *platform;    ///< Platform-specific data (e.g. Arduino C++ object)
} OV7670_host;

#define OV7670_ADDR 0x21 //< Default I2C address if unspecified

// OV7670 registers
#define OV7670_REG_GAIN 0x00               //< AGC gain bits 7:0 (9:8 in VREF)
#define OV7670_REG_BLUE 0x01               //< AWB blue channel gain
#define OV7670_REG_RED 0x02                //< AWB red channel gain
#define OV7670_REG_VREF 0x03               //< Vert frame control bits
#define OV7670_REG_COM1 0x04               //< Common control 1
#define OV7670_COM1_R656 0x40              //< COM1 enable R656 format
#define OV7670_REG_BAVE 0x05               //< U/B average level
#define OV7670_REG_GbAVE 0x06              //< Y/Gb average level
#define OV7670_REG_AECHH 0x07              //< Exposure value - AEC 15:10 bits
#define OV7670_REG_RAVE 0x08               //< V/R average level
#define OV7670_REG_COM2 0x09               //< Common control 2
#define OV7670_COM2_SSLEEP 0x10            //< COM2 soft sleep mode
#define OV7670_REG_PID 0x0A                //< Product ID MSB (read-only)
#define OV7670_REG_VER 0x0B                //< Product ID LSB (read-only)
#define OV7670_REG_COM3 0x0C               //< Common control 3
#define OV7670_COM3_SWAP 0x40              //< COM3 output data MSB/LSB swap
#define OV7670_COM3_SCALEEN 0x08           //< COM3 scale enable
#define OV7670_COM3_DCWEN 0x04             //< COM3 DCW enable
#define OV7670_REG_COM4 0x0D               //< Common control 4
#define OV7670_REG_COM5 0x0E               //< Common control 5
#define OV7670_REG_COM6 0x0F               //< Common control 6
#define OV7670_REG_AECH 0x10               //< Exposure value 9:2
#define OV7670_REG_CLKRC 0x11              //< Internal clock
#define OV7670_CLK_EXT 0x40                //< CLKRC Use ext clock directly
#define OV7670_CLK_SCALE 0x3F              //< CLKRC Int clock prescale mask
#define OV7670_REG_COM7 0x12               //< Common control 7
#define OV7670_COM7_RESET 0x80             //< COM7 SCCB register reset
#define OV7670_COM7_SIZE_MASK 0x38         //< COM7 output size mask
#define OV7670_COM7_PIXEL_MASK 0x05        //< COM7 output pixel format mask
#define OV7670_COM7_SIZE_VGA 0x00          //< COM7 output size VGA
#define OV7670_COM7_SIZE_CIF 0x20          //< COM7 output size CIF
#define OV7670_COM7_SIZE_QVGA 0x10         //< COM7 output size QVGA
#define OV7670_COM7_SIZE_QCIF 0x08         //< COM7 output size QCIF
#define OV7670_COM7_RGB 0x04               //< COM7 pixel format RGB
#define OV7670_COM7_YUV 0x00               //< COM7 pixel format YUV
#define OV7670_COM7_BAYER 0x01             //< COM7 pixel format Bayer RAW
#define OV7670_COM7_PBAYER 0x05            //< COM7 pixel fmt proc Bayer RAW
#define OV7670_COM7_COLORBAR 0x02          //< COM7 color bar enable
#define OV7670_REG_COM8 0x13               //< Common control 8
#define OV7670_COM8_FASTAEC 0x80           //< COM8 Enable fast AGC/AEC algo,
#define OV7670_COM8_AECSTEP 0x40           //< COM8 AEC step size unlimited
#define OV7670_COM8_BANDING 0x20           //< COM8 Banding filter enable
#define OV7670_COM8_AGC 0x04               //< COM8 AGC (auto gain) enable
#define OV7670_COM8_AWB 0x02               //< COM8 AWB (auto white balance)
#define OV7670_COM8_AEC 0x01               //< COM8 AEC (auto exposure) enable
#define OV7670_REG_COM9 0x14               //< Common control 9 - max AGC value
#define OV7670_REG_COM10 0x15              //< Common control 10
#define OV7670_COM10_HSYNC 0x40            //< COM10 HREF changes to HSYNC
#define OV7670_COM10_PCLK_HB 0x20          //< COM10 Suppress PCLK on hblank
#define OV7670_COM10_PCLK_REV 0x10         //< COM10 PCLK reverse
#define OV7670_COM10_HREF_REV 0x08         //< COM10 HREF reverse
#define OV7670_COM10_VS_EDGE 0x04          //< COM10 VSYNC chg on PCLK rising
#define OV7670_COM10_VS_NEG 0x02           //< COM10 VSYNC negative
#define OV7670_COM10_HS_NEG 0x01           //< COM10 HSYNC negative
#define OV7670_REG_HSTART 0x17             //< Horiz frame start high bits
#define OV7670_REG_HSTOP 0x18              //< Horiz frame end high bits
#define OV7670_REG_VSTART 0x19             //< Vert frame start high bits
#define OV7670_REG_VSTOP 0x1A              //< Vert frame end high bits
#define OV7670_REG_PSHFT 0x1B              //< Pixel delay select
#define OV7670_REG_MIDH 0x1C               //< Manufacturer ID high byte
#define OV7670_REG_MIDL 0x1D               //< Manufacturer ID low byte
#define OV7670_REG_MVFP 0x1E               //< Mirror / vert-flip enable
#define OV7670_MVFP_MIRROR 0x20            //< MVFP Mirror image
#define OV7670_MVFP_VFLIP 0x10             //< MVFP Vertical flip
#define OV7670_REG_LAEC 0x1F               //< Reserved
#define OV7670_REG_ADCCTR0 0x20            //< ADC control
#define OV7670_REG_ADCCTR1 0x21            //< Reserved
#define OV7670_REG_ADCCTR2 0x22            //< Reserved
#define OV7670_REG_ADCCTR3 0x23            //< Reserved
#define OV7670_REG_AEW 0x24                //< AGC/AEC upper limit
#define OV7670_REG_AEB 0x25                //< AGC/AEC lower limit
#define OV7670_REG_VPT 0x26                //< AGC/AEC fast mode op region
#define OV7670_REG_BBIAS 0x27              //< B channel signal output bias
#define OV7670_REG_GbBIAS 0x28             //< Gb channel signal output bias
#define OV7670_REG_EXHCH 0x2A              //< Dummy pixel insert MSB
#define OV7670_REG_EXHCL 0x2B              //< Dummy pixel insert LSB
#define OV7670_REG_RBIAS 0x2C              //< R channel signal output bias
#define OV7670_REG_ADVFL 0x2D              //< Insert dummy lines MSB
#define OV7670_REG_ADVFH 0x2E              //< Insert dummy lines LSB
#define OV7670_REG_YAVE 0x2F               //< Y/G channel average value
#define OV7670_REG_HSYST 0x30              //< HSYNC rising edge delay
#define OV7670_REG_HSYEN 0x31              //< HSYNC falling edge delay
#define OV7670_REG_HREF 0x32               //< HREF control
#define OV7670_REG_CHLF 0x33               //< Array current control
#define OV7670_REG_ARBLM 0x34              //< Array ref control - reserved
#define OV7670_REG_ADC 0x37                //< ADC control - reserved
#define OV7670_REG_ACOM 0x38               //< ADC & analog common - reserved
#define OV7670_REG_OFON 0x39               //< ADC offset control - reserved
#define OV7670_REG_TSLB 0x3A               //< Line buffer test option
#define OV7670_TSLB_NEG 0x20               //< TSLB Negative image enable
#define OV7670_TSLB_YLAST 0x04             //< TSLB UYVY or VYUY, see COM13
#define OV7670_TSLB_AOW 0x01               //< TSLB Auto output window
#define OV7670_REG_COM11 0x3B              //< Common control 11
#define OV7670_COM11_NIGHT 0x80            //< COM11 Night mode
#define OV7670_COM11_NMFR 0x60             //< COM11 Night mode frame rate mask
#define OV7670_COM11_HZAUTO 0x10           //< COM11 Auto detect 50/60 Hz
#define OV7670_COM11_BAND 0x08             //< COM11 Banding filter val select
#define OV7670_COM11_EXP 0x02              //< COM11 Exposure timing control
#define OV7670_REG_COM12 0x3C              //< Common control 12
#define OV7670_COM12_HREF 0x80             //< COM12 Always has HREF
#define OV7670_REG_COM13 0x3D              //< Common control 13
#define OV7670_COM13_GAMMA 0x80            //< COM13 Gamma enable
#define OV7670_COM13_UVSAT 0x40            //< COM13 UV saturation auto adj
#define OV7670_COM13_UVSWAP 0x01           //< COM13 UV swap, use w TSLB[3]
#define OV7670_REG_COM14 0x3E              //< Common control 14
#define OV7670_COM14_DCWEN 0x10            //< COM14 DCW & scaling PCLK enable
#define OV7670_REG_EDGE 0x3F               //< Edge enhancement adjustment
#define OV7670_REG_COM15 0x40              //< Common control 15
#define OV7670_COM15_RMASK 0xC0            //< COM15 Output range mask
#define OV7670_COM15_R10F0 0x00            //< COM15 Output range 10 to F0
#define OV7670_COM15_R01FE 0x80            //< COM15              01 to FE
#define OV7670_COM15_R00FF 0xC0            //< COM15              00 to FF
#define OV7670_COM15_RGBMASK 0x30          //< COM15 RGB 555/565 option mask
#define OV7670_COM15_RGB 0x00              //< COM15 Normal RGB out
#define OV7670_COM15_RGB565 0x10           //< COM15 RGB 565 output
#define OV7670_COM15_RGB555 0x30           //< COM15 RGB 555 output
#define OV7670_REG_COM16 0x41              //< Common control 16
#define OV7670_COM16_AWBGAIN 0x08          //< COM16 AWB gain enable
#define OV7670_REG_COM17 0x42              //< Common control 17
#define OV7670_COM17_AECWIN 0xC0           //< COM17 AEC window must match COM4
#define OV7670_COM17_CBAR 0x08             //< COM17 DSP Color bar enable
#define OV7670_REG_AWBC1 0x43              //< Reserved
#define OV7670_REG_AWBC2 0x44              //< Reserved
#define OV7670_REG_AWBC3 0x45              //< Reserved
#define OV7670_REG_AWBC4 0x46              //< Reserved
#define OV7670_REG_AWBC5 0x47              //< Reserved
#define OV7670_REG_AWBC6 0x48              //< Reserved
#define OV7670_REG_REG4B 0x4B              //< UV average enable
#define OV7670_REG_DNSTH 0x4C              //< De-noise strength
#define OV7670_REG_MTX1 0x4F               //< Matrix coefficient 1
#define OV7670_REG_MTX2 0x50               //< Matrix coefficient 2
#define OV7670_REG_MTX3 0x51               //< Matrix coefficient 3
#define OV7670_REG_MTX4 0x52               //< Matrix coefficient 4
#define OV7670_REG_MTX5 0x53               //< Matrix coefficient 5
#define OV7670_REG_MTX6 0x54               //< Matrix coefficient 6
#define OV7670_REG_BRIGHT 0x55             //< Brightness control
#define OV7670_REG_CONTRAS 0x56            //< Contrast control
#define OV7670_REG_CONTRAS_CENTER 0x57     //< Contrast center
#define OV7670_REG_MTXS 0x58               //< Matrix coefficient sign
#define OV7670_REG_LCC1 0x62               //< Lens correction option 1
#define OV7670_REG_LCC2 0x63               //< Lens correction option 2
#define OV7670_REG_LCC3 0x64               //< Lens correction option 3
#define OV7670_REG_LCC4 0x65               //< Lens correction option 4
#define OV7670_REG_LCC5 0x66               //< Lens correction option 5
#define OV7670_REG_MANU 0x67               //< Manual U value
#define OV7670_REG_MANV 0x68               //< Manual V value
#define OV7670_REG_GFIX 0x69               //< Fix gain control
#define OV7670_REG_GGAIN 0x6A              //< G channel AWB gain
#define OV7670_REG_DBLV 0x6B               //< PLL & regulator control
#define OV7670_REG_AWBCTR3 0x6C            //< AWB control 3
#define OV7670_REG_AWBCTR2 0x6D            //< AWB control 2
#define OV7670_REG_AWBCTR1 0x6E            //< AWB control 1
#define OV7670_REG_AWBCTR0 0x6F            //< AWB control 0
#define OV7670_REG_SCALING_XSC 0x70        //< Test pattern X scaling
#define OV7670_REG_SCALING_YSC 0x71        //< Test pattern Y scaling
#define OV7670_REG_SCALING_DCWCTR 0x72     //< DCW control
#define OV7670_REG_SCALING_PCLK_DIV 0x73   //< DSP scale control clock divide
#define OV7670_REG_REG74 0x74              //< Digital gain control
#define OV7670_REG_REG76 0x76              //< Pixel correction
#define OV7670_REG_SLOP 0x7A               //< Gamma curve highest seg slope
#define OV7670_REG_GAM_BASE 0x7B           //< Gamma register base (1 of 15)
#define OV7670_GAM_LEN 15                  //< Number of gamma registers
#define OV7670_R76_BLKPCOR 0x80            //< REG76 black pixel corr enable
#define OV7670_R76_WHTPCOR 0x40            //< REG76 white pixel corr enable
#define OV7670_REG_RGB444 0x8C             //< RGB 444 control
#define OV7670_R444_ENABLE 0x02            //< RGB444 enable
#define OV7670_R444_RGBX 0x01              //< RGB444 word format
#define OV7670_REG_DM_LNL 0x92             //< Dummy line LSB
#define OV7670_REG_LCC6 0x94               //< Lens correction option 6
#define OV7670_REG_LCC7 0x95               //< Lens correction option 7
#define OV7670_REG_HAECC1 0x9F             //< Histogram-based AEC/AGC ctrl 1
#define OV7670_REG_HAECC2 0xA0             //< Histogram-based AEC/AGC ctrl 2
#define OV7670_REG_SCALING_PCLK_DELAY 0xA2 //< Scaling pixel clock delay
#define OV7670_REG_BD50MAX 0xA5            //< 50 Hz banding step limit
#define OV7670_REG_HAECC3 0xA6             //< Histogram-based AEC/AGC ctrl 3
#define OV7670_REG_HAECC4 0xA7             //< Histogram-based AEC/AGC ctrl 4
#define OV7670_REG_HAECC5 0xA8             //< Histogram-based AEC/AGC ctrl 5
#define OV7670_REG_HAECC6 0xA9             //< Histogram-based AEC/AGC ctrl 6
#define OV7670_REG_HAECC7 0xAA             //< Histogram-based AEC/AGC ctrl 7
#define OV7670_REG_BD60MAX 0xAB            //< 60 Hz banding step limit
#define OV7670_REG_ABLC1 0xB1              //< ABLC enable
#define OV7670_REG_THL_ST 0xB3             //< ABLC target
#define OV7670_REG_SATCTR 0xC9             //< Saturation control

// 2640 stuff will be split into separate file, but for now...

#define OV2640_ADDR 0x30 //< Default I2C address if unspecified

#define OV2640_REG_RA_DLMT 0xFF            //< Register bank select
#define OV2640_RA_DLMT_DSP 0x00            //< Bank 0 - DSP address
#define OV2640_RA_DLMT_SENSOR 0x01         //< Bank 1 - Sensor address

// OV2640 register bank 0 -- DSP address
// These register names are preceded by 'REG0' as a bank-select reminder
#define OV2640_REG0_R_BYPASS 0x05          //< Bypass DSP
#define OV2640_R_BYPASS_MASK 0x01          //< R_BYPASS bypass DSP mask
#define OV2640_REG0_QS 0x44                //< Quantization scale factor
#define OV2640_REG0_CTRLI 0x50             //< ?
#define OV2640_CTRLI_LP_DP_MASK 0x80       //< LP_DP mask
#define OV2640_CTRLI_V_DIV_MASK 0x38       //< V_DIVIDER mask
#define OV2640_CTRLI_H_DIV_MASK 0x07       //< H_DIVIDER mask
#define OV2640_REG0_HSIZE 0x51             //< H_SIZE[7:0] (real/4)
#define OV2640_REG0_VSIZE 0x52             //< V_SIZE[7:0] (real/4)
#define OV2640_REG0_XOFFL 0x53             //< OFFSET_X[7:0]
#define OV2640_REG0_YOFFL 0x54             //< OFFSET_Y[7:0]
#define OV2640_REG0_VHYX 0x55              //< V/H/X/Y size/offset high bits
#define OV2640_VHYX_V_SIZE_MASK 0x80       //< V_SIZE[8] mask
#define OV2640_VHYX_OFFSET_Y_MASK 0x70     //< OFFSET_Y[10:8] mask
#define OV2640_VHYX_H_SIZE_MASK 0x08       //< H_SIZE[8] masl
#define OV2640_VHYX_OFFSET_X_MASK 0x7      //< OFFSET_X[10:8] mask
#define OV2640_REG0_DPRP 0x56              //< ?
#define OV2640_DPRP_DP_SELY_MASK 0xF0      //< DP_SELY mask
#define OV2640_DPRP_DP_SELX_MASK 0x0F      //< DP_SELX mask
#define OV2640_REG0_TEST 0x57              //< ?
#define OV2640_TEST_H_SIZE_MASK 0x80       //< H_SIZE[9] mask
#define OV2640_REG0_ZMOW 0x5A              //< OUTW[7:0] (real/4)
#define OV2640_REG0_ZMOH 0x5B              //< OUTH[7:0] (real/4)
#define OV2640_REG0_ZMHH 0x5C              //< Zoom speed and more
#define OV2640_ZMHH_ZMSPD_MASK 0xF0        //< ZMSPD (zoom speed)
#define OV2640_ZMHH_OUTH_MASK 0x40         //< OUTH[8]
#define OV2640_ZMHH_OUTW_MASK 0x03         //< OUTW[9:8]
#define OV2640_REG0_BPADDR 0x7C            //< SDE indirect reg access: addr
#define OV2640_REG0_BPDATA 0x7D            //< SDE indirect reg access: data
#define OV2640_REG0_CTRL2 0x86             //< Module enable
#define OV2640_CTRL2_DCW_MASK 0x20         //< DCW mask
#define OV2640_CTRL2_SDE_MASK 0x10         //< SDE mask
#define OV2640_CTRL2_UV_ADJ_MASK 0x08      //< UV_ADJ mask
#define OV2640_CTRL2_UV_AVG_MASK 0x04      //< UV_AVG mask
#define OV2640_CTRL2_CMX_MASK 0x01         //< CMX mask
#define OV2640_REG0_CTRL3 0x87             //< Module enable, continued
#define OV2640_CTRL3_BPC_MASK 0x80         //< BPC mask
#define OV2640_CTRL3_WPC_MASK 0x40         //< WPC mask
#define OV2640_REG0_SIZEL 0x87             //< HSIZE, VSIZE bits
#define OV2640_REG0_HSIZE8 0xC0            //< Image horiz size HSIZE[10:3]
#define OV2640_REG0_VSIZE8 0xC1            //< Image vert size VSIZE[10:3]
#define OV2640_REG0_CTRL0 0xC2             //< Module enable, continued
#define OV2640_CTRL0_AEC_EN_MASK 0x80      //< AEC_EN mask
#define OV2640_CTRL0_AEC_SEL_MASK 0x40     //< AEC_SEL mask
#define OV2640_CTRL0_STAT_SEL_MASK 0x20    //< STAT_SEL mask
#define OV2640_CTRL0_VFIRST_MASK 0x10      //< VFIRST mask
#define OV2640_CTRL0_YUV422_MASK 0x08      //< YUV922 mask
#define OV2640_CTRL0_YUV_EN_MASK 0x04      //< YUV_EN mask
#define OV2640_CTRL0_RGB_EN_MASK 0x02      //< RGB_EN mask
#define OV2640_CTRL0_RAW_EN_MASK 0x01      //< RAW_EN mask
#define OV2640_REG0_CTRL1 0xC3             //< Module enable, continued
#define OV2640_CTRL1_CIP_MASK 0x80         //< CIP mask
#define OV2640_CTRL1_DMY_MASK 0x40         //< DMY mask
#define OV2640_CTRL1_RAW_GMA_MASK 0x20     //< RAW_GMA mask
#define OV2640_CTRL1_DG_MASK 0x10          //< DG mask
#define OV2640_CTRL1_AWB_MASK 0x08         //< AWB mask
#define OV2640_CTRL1_AWB_GAIN_MASK 0x04    //< AWB_GAIN mask
#define OV2640_CTRL1_LENC_MASK 0x02        //< LENC mask
#define OV2640_CTRL1_PRE_MASK 0x01         //< PRE mask
#define OV2640_REG0_R_DVP_SP 0xD3          //< DVP selections
#define OV2640_R_DVP_SP_AUTO_MASK 0x80     //< Auto mode mask
#define OV2640_R_DVP_SP_PCLK_MASK 0x7F     //< DVP PCLK mask
#define OV2640_REG0_IMAGE_MODE 0xDA        //< Image output format select
#define OV2640_IMAGE_MODE_Y8_MASK 0x40     //< Y8 enable for DVP
#define OV2640_IMAGE_MODE_JPEG_MASK 0x10   //< JPEG output enable mask
#define OV2640_IMAGE_MODE_DVP_MASK 0x0C    //< DVP output format mask
#define OV2640_IMAGE_MODE_DVP_YUV 0x00     //< YUV422
#define OV2640_IMAGE_MODE_DVP_RAW10 0x04   //< RAW10 (DVP)
#define OV2640_IMAGE_MODE_DVP_RGB565 0x08  //< RGB565
#define OV2640_IMAGE_MODE_HREF_MASK 0x02   //< HREF timing select in JPEG mode
#define OV2640_IMAGE_MODE_SWAP_MASK 0x01   //< Byte swap enable for DVP
#define OV2640_REG0_RESET 0xE0             //< Reset
#define OV2640_RESET_MCU_MASK 0x40         //< Microcontroller reset mask
#define OV2640_RESET_SCCB_MASK 0x20        //< SCCB reset mask
#define OV2640_RESET_JPEG_MASK 0x10        //< JPEG reset mask
#define OV2640_RESET_DVP_MASK 0x04         //< DVP reset mask
#define OV2640_RESET_IPU_MASK 0x02         //< IPU reset mask
#define OV2640_RESET_CIF_MASK 0x01         //< CIF reset mask
#define OV2640_REG0_MS_SP 0xF0             //< SCCB host speed
#define OV2640_REG0_SS_ID 0xF7             //< SCCB periph ID
#define OV2640_REG0_SS_CTRL 0xF8           //< SCCB periph control 1
#define OV2640_SS_CTRL_ADDR_MASK 0x20      //< Address auto-increment mask
#define OV2640_SS_CTRL_SCCB_MASK 0x08      //< SCCB enable mask
#define OV2640_SS_CTRL_DELAY_MASK 0x04     //< Delay SCCB main clock mask
#define OV2640_SS_CTRL_ACCESS_MASK 0x02    //< Enable SCCB host access mask
#define OV2640_SS_CTRL_SENSOR_MASK 0x01    //< Enable sensor pass-through mask
#define OV2640_REG0_MC_BIST 0xF9           //< ?
#define OV2640_MC_BIST_RESET_MASK 0x80     //< MCU reset mask
#define OV2640_MC_BIST_BOOTROM_MASK 0x40   //< Boot ROM select mask
#define OV2640_MC_BIST_12K_1_MASK 0x20     //< R/W 1 error for 12KB mem mask
#define OV2640_MC_BIST_12K_0_MASK 0x10     //< R/W 0 error for 12KB mem mask
#define OV2640_MC_BIST_512_1_MASK 0x08     //< R/W 1 error for 512B mem mask
#define OV2640_MC_BIST_512_0_MASK 0x04     //< R/W 0 error for 512B mem mask
#define OV2640_MC_BIST_BUSY_MASK 0x02      //< R=BISY busy, W=MCU reset mask
#define OV2640_MC_BIST_LAUNCH_MASK 0x01    //< Launch BIST mask
#define OV2640_REG0_MC_AL 0xFA             //< Program mem ptr addr low byte
#define OV2640_REG0_MC_AH 0xFB             //< Program mem ptr addr high byte
#define OV2640_REG0_MC_D 0xFC              //< Program mem ptr access address
#define OV2640_REG0_P_CMD 0xFD             //< SCCB protocol command register
#define OV2640_REG0_P_STATUS 0xFE          //< SCCB protocol status register

// OV2640 register bank 1 -- Sensor address
// These register names are preceded by 'REG1' as a bank-select reminder
#define OV2640_REG1_GAIN 0x00              //< AGC gain control LSBs
#define OV2640_REG1_COM1 0x03              //< Common control 1
#define OV2640_COM1_DFRAME_MASK 0xC0       //< Dummy frame control mask
#define OV2640_COM1_DFRAME_1 0x40          //< Allow 1 dummy frame
#define OV2640_COM1_DFRAME_4 0x80          //< Allow 4 dummy frames
#define OV2640_COM1_DFRAME_7 0xC0          //< Allow 7 dummy frames
#define OV2640_COM1_VEND_MASK 0x0C         //< Vert window end line LSBs
#define OV2640_COM1_VSTRT_MASK 0x03        //< Vert window start line LSBs
#define OV2640_REG1_REG04 0x04             //< Register 04
#define OV2640_REG04_HFLIP_MASK 0x80       //< Horizontal mirror
#define OV2640_REG04_VFLIP_MASK 0x40       //< Vertical mirror
#define OV2640_REG04_VREF_MASK 0x10        //< VREF[0]
#define OV2640_REG04_HREF_MASK 0x08        //< HREF[0]
#define OV2640_REG04_AEC_MASK 0x03         //< AEC[1:0]
#define OV2640_REG1_REG08 0x08             //< Register 08 (frame exposure)
#define OV2640_REG1_COM2 0x09              //< Common control 2
#define OV2640_COM2_STANDBY_MASK 0x10      //< Standby mode mask
#define OV2640_COM2_PINUSE_MASK 0x04       //< PWDN/RESETB as SLVS/SLHS mask
#define OV2640_COM2_DRIVE_MASK 0x03        //< Output drive select mask
#define OV2640_COM2_DRIVE_1X 0x00          //< 1x
#define OV2640_COM2_DRIVE_3X 0x01          //< 3x (sic)
#define OV2640_COM2_DRIVE_2X 0x02          //< 2x (sic)
#define OV2640_COM2_DRIVE_4X 0x03          //< 4x
#define OV2640_REG1_PIDH 0x0A              //< Product ID MSB (read only)
#define OV2640_REG1_PIDL 0x0B              //< Product ID LSB (read only)
#define OV2640_REG1_COM3 0x0C              //< Common control 3
#define OV2640_COM3_BANDING_MASK 0x04      //< Set banding manually mask
#define OV2640_COM3_BANDING_60HZ 0x00      //< 60 Hz
#define OV2640_COM3_BANDING_50HZ 0x04      //< 50 Hz
#define OV2640_COM3_AUTO_BAND_MASK 0x02    //< Auto-set banding mask
#define OV2640_COM3_SNAPSHOT_MASK 0x01     //< Snapshot option
#define OV2640_REG1_COM4 0x0D              //< Common control 4
#define OV2640_COM4_CLOCK_MASK 0x04        //< Clock output power pin status
#define OV2640_REG1_AEC 0x10               //< AEC[9:2] auto exposure ctrl
#define OV2640_REG1_CLKRC 0x11             //< Clock rate control
#define OV2640_CLKRC_DOUBLE_MASK 0x80      //< Internal freq doubler mask
#define OV2640_CLKRC_DOUBLE_OFF 0x00       //< Internal freq doubler off
#define OV2640_CLKRC_DOUBLE_ON 0x80        //< Internal freq doubler on
#define OV2640_CLKRC_DIV_MASK 0x3F         //< Clock divider mask
#define OV2640_REG1_COM7 0x12              //< Common control 7
#define OV2640_COM7_SRST_MASK 0x80         //< System reset mask
#define OV2640_COM7_RES_MASK 0x70          //< Resolution mask
#define OV2640_COM7_RES_UXGA 0x00          //< UXGA (full size) mode
#define OV2640_COM7_RES_CIF 0x10           //< CIF mode
#define OV2640_COM7_RES_SVGA 0x40          //< SVGA mode
#define OV2640_COM7_ZOOM_MASK 0x04         //< Zoom mode
#define OV2640_COM7_COLORBAR_MASK 0x02     //< Color bar test pattern enable
#define OV2640_REG1_COM8 0x13              //< Common control 8
#define OV2640_COM8_BANDING_MASK 0x20      //< Banding filter select mask
#define OV2640_COM8_BANDING_OFF 0x00       //< Banding filter off
#define OV2640_COM8_BANDING_ON 0x20        //< Banding filter on
#define OV2640_COM8_AGC_MASK 0x04          //< AGC auto/manual select mask
#define OV2640_COM8_AGC_MANUAL 0x00        //< Manual gain
#define OV2640_COM8_AGC_AUTO 0x04          //< Auto gain
#define OV2640_COM8_EXP_MASK 0x01          //< Exposure control mask
#define OV2640_COM8_EXP_MANUAL 0x00        //< Manual exposure
#define OV2640_COM8_EXP_AUTO 0x01          //< Auto exposure
#define OV2640_REG1_COM9 0x14              //< Common control 9
#define OV2640_COM9_AGC_GAIN_MASK 0xE0     //< AGC gain ceiling mask, GH[2:0]
#define OV2640_COM9_AGC_GAIN_2X 0x00       //< 2x
#define OV2640_COM9_AGC_GAIN_4X 0x20       //< 4x
#define OV2640_COM9_AGC_GAIN_8X 0x40       //< 8x
#define OV2640_COM9_AGC_GAIN_16X 0x60      //< 16x
#define OV2640_COM9_AGC_GAIN_32X 0x80      //< 32x
#define OV2640_COM9_AGC_GAIN_64X 0xA0      //< 64x
#define OV2640_COM9_AGC_GAIN_128X 0xC0     //< 128x
#define OV2640_REG1_COM10 0x15             //< Common control 10
#define OV2640_COM10_CHSYNC_SWAP_MASK 0x80 //< CHSYNC pin output swap mask
#define OV2640_COM10_CHSYNC_CHSYNC 0x00    //< CHSYNC
#define OV2640_COM10_CHSYNC_HREF 0x80      //< HREF
#define OV2640_COM10_HREF_SWAP_MASK 0x40   //< HREF pin output swap mask
#define OV2640_COM10_HREF_HREF 0x00        //< HREF
#define OV2640_COM10_HREF_CHSYNC 0x40      //< CHSYNC
#define OV2640_COM10_PCLK_MASK 0x20        //< PCLK output selection mask
#define OV2640_COM10_PCLK_ALWAYS 0x00      //< PCLK always output
#define OV2640_COM10_PCLK_HREF 0x20        //< PCLK qualified by HREF
#define OV2640_COM10_PCLK_EDGE_MASK 0x10   //< PCLK edge selection mask
#define OV2640_COM10_PCLK_FALLING 0x00     //< Data updated on falling PCLK
#define OV2640_COM10_PCLK_RISING 0x10      //< Data updated on rising PCLK
#define OV2640_COM10_HREF_MASK 0x08        //< HREF polarity mask
#define OV2640_COM10_HREF_POSITIVE 0x00    //< Positive HREF
#define OV2640_COM10_HREF_NEGATIVE 0x08    //< Negative HREF for data valid
#define OV2640_COM10_VSYNC_MASK 0x02       //< VSYNC polarity mask
#define OV2640_COM10_VSYNC_POSITIVE 0x00   //< Positive VSYNC
#define OV2640_COM10_VSYNC_NEGATIVE 0x02   //< Negative VSYNC
#define OV2640_COM10_HSYNC_MASK 0x01       //< HSYNC polarity mask
#define OV2640_COM10_HSYNC_POSITIVE 0x00   //< Positive HSYNC
#define OV2640_COM10_HSYNC_NEGATIVE 0x01   //< Negative HSYNC
#define OV2640_REG1_HREFST 0x17            //< Horizontal window start MSB
#define OV2640_REG1_HREFEND 0x18           //< Horizontal window end MSB
#define OV2640_REG1_VSTRT 0x19             //< Vertical window line start MSB
#define OV2640_REG1_VEND 0x1A              //< Vertical window line end MSB
#define OV2640_REG1_MIDH 0x1C              //< Manufacturer ID MSB (RO=0x7F)
#define OV2640_REG1_MIDL 0x1D              //< Manufacturer ID LSB (RO=0xA2)
#define OV2640_REG1_AEW 0x24               //< Luminance signal high range
#define OV2640_REG1_AEB 0x25               //< Luminance signal low range
#define OV2640_REG1_VV 0x26                //< Fast mode large step threshold
#define OV2640_VV_HIGH_MASK 0xF0           //< High threshold mask
#define OV2640_VV_LOW_MASK 0x0F            //< Low threshold mask
#define OV2640_REG1_REG2A 0x2A             //< Register 2A
#define OV2640_REG2A_LINE_MASK 0xF0        //< Line interval adjust MSBs
#define OV2640_REG2A_HSYNC_END_MASK 0x0C   //< HSYNC timing end point MSBs
#define OV2640_REG2A_HSYNC_START_MASK 0x03 //< HSYNC timing start point MSBs
#define OV2640_REG1_FRARL 0x2B             //< Line interval adjust LSB
#define OV2640_REG1_ADDVSL 0x2D            //< VSYNC pulse width LSB
#define OV2640_REG1_ADDVSH 0x2E            //< VSYNC pulse width MSB
#define OV2640_REG1_YAVG 0x2F              //< Luminance average
#define OV2640_REG1_HSDY 0x30              //< HSYNC pos+width start LSB
#define OV2640_REG1_HEDY 0x31              //< HSYNC pos+width end LSB
#define OV2640_REG1_REG32 0x32             //< Common control 32
#define OV2640_REG32_PCLK_MASK 0xC0        //< Pixel clock divide option mask
#define OV2640_REG32_PCLK_DIV1 0x00        //< No effect on PCLK
#define OV2640_REG32_PCLK_DIV2 0x80        //< PCLK frequency / 2
#define OV2640_REG32_PCLK_DIV4 0xC0        //< PCLK frequency / 4
#define OV2640_REG32_HREFEND_MASK 0x38     //< HREFEND LSBs
#define OV2640_REG32_HREFST_MASK 0x07      //< HREFST LSBs
#define OV2640_REG1_ARCOM2 0x34            //< ?
#define OV2640_ARCOM2_ZOOM_MASK 0x04       //< Zoom window horiz start point
#define OV2640_REG1_REG45 0x45             //< Register 45
#define OV2640_REG45_AGC_MASK 0xC0         //< AGC[9:8] highest gain control
#define OV2640_REG45_AEC_MASK 0x3F         //< AEC[15:10] AEC MSBs
#define OV2640_REG1_FLL 0x46               //< Frame length adjustment LSBs
#define OV2640_REG1_FLH 0x47               //< Frame length adjustment MSBs
#define OV2640_REG1_COM19 0x48             //< Frame length adjustment MSBs
#define OV2640_COM19_ZOOM_MASK 0x03        //< Zoom mode vert window LSBs
#define OV2640_REG1_ZOOMS 0x49             //< Zoom mode vert window MSB
#define OV2640_REG1_COM22 0x4B             //< Common control 22 (flash)
#define OV2640_REG1_COM25 0x4E             //< Common control 25
#define OV2640_COM25_50HZ_MASK 0xC0        //< 50 Hz banding AEC MSBs
#define OV2640_COM25_60HZ_MASK 0x30        //< 60 Hz banding AEC MSBs
#define OV2640_REG1_BD50 0x4F              //< 50 Hz banding AEC LSBs
#define OV2640_REG1_BD60 0x50              //< 60 Hz banding AEC LSBs
#define OV2640_REG1_REG5D 0x5D             //< AVGsel[7:0] 16-zone avg weight
#define OV2640_REG1_REG5E 0x5E             //< AVGsel[15:8]
#define OV2640_REG1_REG5F 0x5F             //< AVGsel[23:16]
#define OV2640_REG1_REG60 0x60             //< AVGsel[31:24]
#define OV2640_REG1_HISTO_LOW 0x61         //< Histogram low level
#define OV2640_REG1_HISTO_HIGH 0x62        //< Histogram high level

extern OV7670_status OV7670_arch_begin(OV7670_host *host);

// C++ ACCESSIBLE FUNCTIONS ------------------------------------------------

// These are declared in an extern "C" so Arduino platform C++ code can
// access them.

#ifdef __cplusplus
extern "C" {
#endif

// Architecture- and platform-neutral initialization function.
// Called by the platform init function, this in turn may call an
// architecture-specific init function.
OV7670_status OV7670_begin(OV7670_host *host, OV7670_colorspace colorspace,
                           OV7670_size size, float fps);

// Configure camera frame rate. Actual resulting frame rate (returned) may
// be different depending on available clock frequencies. Result will only
// exceed input if necessary for minimum supported rate, but this is very
// rare, typically below 1 fps. In all other cases, result will be equal
// or less than the requested rate, up to a maximum of 30 fps (the "or less"
// is because requested fps may be based on other host hardware timing
// constraints (e.g. screen) and rounding up to a closer-but-higher frame
// rate would be problematic). There is no hardcoded set of fixed frame
// rates because it varies with architecture, depending on OV7670_XCLK_HZ.
float OV7670_set_fps(void *platform, float fps);

// Configure camera resolution to one of the supported frame sizes
// (powers-of-two divisions of VGA -- 640x480 down to 40x30).
void OV7670_set_size(void *platform, OV7670_size size);

// Lower-level resolution register fiddling function, exposed so dev code
// can test variations for OV7670_set_size() windowing defaults.
void OV7670_frame_control(void *platform, uint8_t size, uint8_t vstart,
                          uint16_t hstart, uint8_t edge_offset,
                          uint8_t pclk_delay);

// Select one of the camera's night modes (or disable).
// Trades off frame rate for less grainy images in low light.
void OV7670_night(void *platform, OV7670_night_mode night);

// Flips camera output on horizontal and/or vertical axes.
void OV7670_flip(void *platform, bool flip_x, bool flip_y);

// Selects one of the camera's test patterns (or disable).
// See Adafruit_OV7670.h for notes about minor visual bug here.
void OV7670_test_pattern(void *platform, OV7670_pattern pattern);

// Convert Y (brightness) component YUV image in RAM to RGB565 big-
// endian format for preview on TFT display. Data is overwritten in-place,
// Y is truncated and UV elements are lost. No practical use outside TFT
// preview. If you need actual grayscale 0-255 data, just access the low
// byte of each 16-bit YUV pixel.
void OV7670_Y2RGB565(uint16_t *ptr, uint32_t len);

#ifdef __cplusplus
};
#endif
