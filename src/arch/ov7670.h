#pragma once

// IMPORTANT: #include ALL of the arch-specific .h files here.
// They have #ifdef checks to only take effect on the active architecture.
#include "samd51.h"

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

typedef enum {
  OV7670_STATUS_OK = 0,         ///< Success
  OV7670_STATUS_ERR_MALLOC,     ///< malloc() call failed
  OV7670_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} OV7670_status;

// Indices into pin list used by OV7670_begin()
typedef enum {
  OV7670_PIN_XCLK = 0,
  OV7670_PIN_PCLK,
  OV7670_PIN_VSYNC,
  OV7670_PIN_HSYNC,
  OV7670_PIN_D0,
  OV7670_PIN_D1,
  OV7670_PIN_D2,
  OV7670_PIN_D3,
  OV7670_PIN_D4,
  OV7670_PIN_D5,
  OV7670_PIN_D6,
  OV7670_PIN_D7,
  OV7670_PIN_SDA,
  OV7670_PIN_SCL,
  OV7670_PIN_RESET,
  OV7670_PIN_ENABLE,
  OV7670_NUM_PINS,
} OV7670_PIN_INDEX;

typedef struct {
  uint8_t reg;
  uint8_t value;
} OV7670_command;

typedef struct {
  OV7670_arch arch;     ///< Architecture-specific config data
  void       *platform; ///< Platform-specific data (e.g. Arduino C++ object)
  int16_t     pin[OV7670_NUM_PINS]; ///< List of pin numbers (see below)
} OV7670_host;

#define OV7670_ADDR 0x21 //< Default I2C address if unspecified

#define OV7670_REG_GAIN 0x00        //< AGC gain bits 7:0 (9:8 in VREF)
#define OV7670_REG_BLUE 0x01        //< AWB blue channel gain
#define OV7670_REG_RED 0x02         //< AWB red channel gain
#define OV7670_REG_VREF 0x03        //< Vert frame control bits
#define OV7670_REG_COM1 0x04        //< Common control 1
#define OV7670_COM1_R656 0x40       //< COM1 enable R656 format
#define OV7670_REG_BAVE 0x05        //< U/B average level
#define OV7670_REG_GbAVE 0x06       //< Y/Gb average level
#define OV7670_REG_AECHH 0x07       //< Exposure value - AEC 15:10 bits
#define OV7670_REG_RAVE 0x08        //< V/R average level
#define OV7670_REG_COM2 0x09        //< Common control 2
#define OV7670_COM2_SSLEEP 0x10     //< COM2 soft sleep mode
#define OV7670_REG_PID 0x0A         //< Product ID MSB (read-only)
#define OV7670_REG_VER 0x0B         //< Product ID LSB (read-only)
#define OV7670_REG_COM3 0x0C        //< Common control 3
#define OV7670_COM3_SWAP 0x40       //< COM3 output data MSB/LSB swap
#define OV7670_COM3_SCALEEN 0x08    //< COM3 scale enable
#define OV7670_COM3_DCWEN 0x04      //< COM3 DCW enable (dnsample,crop,window)
#define OV7670_REG_COM4 0x0D        //< Common control 4
#define OV7670_REG_COM5 0x0E        //< Common control 5
#define OV7670_REG_COM6 0x0F        //< Common control 6
#define OV7670_REG_AECH 0x10        //< Exposure value 9:2
#define OV7670_REG_CLKRC 0x11       //< Internal clock
#define OV7670_CLK_EXT 0x40         //< CLKRC Use external clock directly
#define OV7670_CLK_SCALE 0x3F       //< CLKRC Internal clock prescale mask
#define OV7670_REG_COM7 0x12        //< Common control 7
#define OV7670_COM7_RESET 0x80      //< COM7 SCCB register reset
#define OV7670_COM7_SIZE_MASK 0x38  //< COM7 output size mask
#define OV7670_COM7_PIXEL_MASK 0x05 //< COM7 output pixel format mask
#define OV7670_COM7_SIZE_VGA 0x00   //< COM7 output size VGA
#define OV7670_COM7_SIZE_CIF 0x20   //< COM7 output size CIF
#define OV7670_COM7_SIZE_QVGA 0x10  //< COM7 output size QVGA
#define OV7670_COM7_SIZE_QCIF 0x08  //< COM7 output size QCIF
#define OV7670_COM7_RGB 0x04        //< COM7 pixel format RGB
#define OV7670_COM7_YUV 0x00        //< COM7 pixel format YUV
#define OV7670_COM7_BAYER 0x01      //< COM7 pixel format Bayer RAW
#define OV7670_COM7_PBAYER 0x05     //< COM7 pixel format processed Bayer RAW
#define OV7670_COM7_COLORBAR 0x02   //< COM7 color bar enable
#define OV7670_REG_COM8 0x13        //< Common control 8
#define OV7670_COM8_FASTAEC 0x80    //< COM8 Enable fast AGC/AEC algorith,
#define OV7670_COM8_AECSTEP 0x40    //< COM8 AEC step size unlimited
#define OV7670_COM8_BANDING 0x20    //< COM8 Banding filter enable
#define OV7670_COM8_AGC 0x04        //< COM8 AGC (auto gain) enable
#define OV7670_COM8_AWB 0x02        //< COM8 AWB (auto white balance) enable
#define OV7670_COM8_AEC 0x01        //< COM8 AEC (auto exposure) enable
#define OV7670_REG_COM9 0x14        //< Common control 9 - max AGC value
#define OV7670_REG_COM10 0x15       //< Common control 10
#define OV7670_COM10_HSYNC 0x40     //< COM10 HREF changes to HSYNC
#define OV7670_COM10_PCLK_HB 0x20   //< COM10 Suppress PCLK on horiz blank
#define OV7670_COM10_HREF_REV 0x08  //< COM10 HREF reverse
#define OV7670_COM10_VS_EDGE 0x04   //< COM10 VSYNC change on PCLK rising edge
#define OV7670_COM10_VS_NEG 0x02    //< COM10 VSYNC negative
#define OV7670_COM10_HS_NEG 0x01    //< COM10 HSYNC negative
#define OV7670_REG_HSTART 0x17      //< Horiz frame start high bits
#define OV7670_REG_HSTOP 0x18       //< Horiz frame end high bits
#define OV7670_REG_VSTART 0x19      //< Vert frame start high bits
#define OV7670_REG_VSTOP 0x1A       //< Vert frame end high bits
#define OV7670_REG_PSHFT 0x1B       //< Pixel delay select
#define OV7670_REG_MIDH 0x1C        //< Manufacturer ID high byte
#define OV7670_REG_MIDL 0x1D        //< Manufacturer ID low byte
#define OV7670_REG_MVFP 0x1E        //< Mirror / vert-flip enable
#define OV7670_MVFP_MIRROR 0x20     //< MVFP Mirror image
#define OV7670_MVFP_VFLIP 0x10      //< MVFP Vertical flip
#define OV7670_REG_LAEC 0x1F        //< Reserved
#define OV7670_REG_ADCCTR0 0x20     //< ADC control
#define OV7670_REG_ADCCTR1 0x21     //< Reserved
#define OV7670_REG_ADCCTR2 0x22     //< Reserved
#define OV7670_REG_ADCCTR3 0x23     //< Reserved
#define OV7670_REG_AEW 0x24         //< AGC/AEC upper limit
#define OV7670_REG_AEB 0x25         //< AGC/AEC lower limit
#define OV7670_REG_VPT 0x26         //< AGC/AEC fast mode operating region
#define OV7670_REG_BBIAS 0x27       //< B channel signal output bias
#define OV7670_REG_GbBIAS 0x28      //< Gb channel signal output bias
#define OV7670_REG_EXHCH 0x2A       //< Dummy pixel insert MSB
#define OV7670_REG_EXHCL 0x2B       //< Dummy pixel insert LSB
#define OV7670_REG_RBIAS 0x2C       //< R channel signal output bias
#define OV7670_REG_ADVFL 0x2D       //< Insert dummy lines MSB
#define OV7670_REG_ADVFH 0x2E       //< Insert dummy lines LSB
#define OV7670_REG_YAVE 0x2F        //< Y/G channel average value
#define OV7670_REG_HSYST 0x30       //< HSYNC rising edge delay
#define OV7670_REG_HSYEN 0x31       //< HSYNC falling edge delay
#define OV7670_REG_HREF 0x32        //< HREF control
#define OV7670_REG_CHLF 0x33        //< Array current control
#define OV7670_REG_ARBLM 0x34       //< Array reference control - reserved
#define OV7670_REG_ADC 0x37         //< ADC control - reserved
#define OV7670_REG_ACOM 0x38        //< ADC & analog common control - reserved
#define OV7670_REG_OFON 0x39        //< ADC offset control - reserved
#define OV7670_REG_TSLB 0x3A        //< Line buffer test option
#define OV7670_TSLB_NEG 0x20        //< TSLB Negative image enable
#define OV7670_TSLB_YLAST 0x04      //< TSLB UYVY or VYUY, see COM13
#define OV7670_TSLB_AOW 0x01        //< TSLB Auto output window
#define OV7670_REG_COM11 0x3B       //< Common control 11
#define OV7670_COM11_NIGHT 0x80     //< COM11 Night mode
#define OV7670_COM11_NMFR 0x60      //< COM11 Night mode frame rate mask
#define OV7670_COM11_HZAUTO 0x10    //< COM11 Auto detect 50/60 Hz
#define OV7670_COM11_BAND 0x08      //< COM11 Banding filter value select
#define OV7670_COM11_EXP 0x02       //< COM11 Exposure timing control
#define OV7670_REG_COM12 0x3C       //< Common control 12
#define OV7670_COM12_HREF 0x80      //< COM12 Always has HREF
#define OV7670_REG_COM13 0x3D       //< Common control 13
#define OV7670_COM13_GAMMA 0x80     //< COM13 Gamma enable
#define OV7670_COM13_UVSAT 0x40     //< COM13 UV saturation auto adjustment
#define OV7670_COM13_UVSWAP 0x01    //< COM13 UV swap, use w TSLB[3]
#define OV7670_REG_COM14 0x3E       //< Common control 14
#define OV7670_COM14_DCWEN 0x10     //< COM14 DCW and scaling PCLK enable
#define OV7670_REG_EDGE 0x3F        //< Edge enhancement adjustment
#define OV7670_REG_COM15 0x40       //< Common control 15
#define OV7670_COM15_RMASK 0xC0     //< COM15 Output range mask
#define OV7670_COM15_R10F0 0x00     //< COM15 Output range 10 to F0
#define OV7670_COM15_R01FE 0x80     //< COM15              01 to FE
#define OV7670_COM15_R00FF 0xC0     //< COM15              00 to FF
#define OV7670_COM15_RGBMASK 0x30   //< COM15 RGB 555/565 option mask
#define OV7670_COM15_RGB 0x00       //< COM15 Normal RGB out
#define OV7670_COM15_RGB565 0x10    //< COM15 RGB 565 output
#define OV7670_COM15_RGB555 0x30    //< COM15 RGB 555 output
#define OV7670_REG_COM16 0x41       //< Common control 16
#define OV7670_COM16_AWBGAIN 0x08   //< COM16 AWB gain enable
#define OV7670_REG_COM17 0x42       //< Common control 17
#define OV7670_COM17_AECWIN 0xC0    //< COM17 AEC window must match COM4
#define OV7670_COM17_CBAR 0x08      //< COM17 DSP Color bar enable
#define OV7670_REG_AWBC1 0x43       //< Reserved
#define OV7670_REG_AWBC2 0x44       //< Reserved
#define OV7670_REG_AWBC3 0x45       //< Reserved
#define OV7670_REG_AWBC4 0x46       //< Reserved
#define OV7670_REG_AWBC5 0x47       //< Reserved
#define OV7670_REG_AWBC6 0x48       //< Reserved
#define OV7670_REG_REG4B 0x4B       //< UV average enable
#define OV7670_REG_DNSTH 0x4C       //< De-noise strength
#define OV7670_REG_MTX1 0x4F        //< Matrix coefficient 1
#define OV7670_REG_MTX2 0x50        //< Matrix coefficient 2
#define OV7670_REG_MTX3 0x51        //< Matrix coefficient 3
#define OV7670_REG_MTX4 0x52        //< Matrix coefficient 4
#define OV7670_REG_MTX5 0x53        //< Matrix coefficient 5
#define OV7670_REG_MTX6 0x54        //< Matrix coefficient 6
#define OV7670_REG_BRIGHT 0x55      //< Brightness control
#define OV7670_REG_CONTRAS 0x56     //< Contrast control
#define OV7670_REG_CONTRAS_CTR 0x57 //< Contrast center
#define OV7670_REG_MTXS 0x58        //< Matrix coefficient sign
#define OV7670_REG_LCC1 0x62        //< Lens correction option 1
#define OV7670_REG_LCC2 0x63        //< Lens correction option 2
#define OV7670_REG_LCC3 0x64        //< Lens correction option 3
#define OV7670_REG_LCC4 0x65        //< Lens correction option 4
#define OV7670_REG_LCC5 0x66        //< Lens correction option 5
#define OV7670_REG_MANU 0x67        //< Manual U value
#define OV7670_REG_MANV 0x68        //< Manual V value
#define OV7670_REG_GFIX 0x69        //< Fix gain control
#define OV7670_REG_GGAIN 0x6A       //< G channel AWB gain
#define OV7670_REG_DBLV 0x6B        //< PLL & regulator control
#define OV7670_REG_AWBCTR3 0x6C     //< AWB control 3
#define OV7670_REG_AWBCTR2 0x6D     //< AWB control 2
#define OV7670_REG_AWBCTR1 0x6E     //< AWB control 1
#define OV7670_REG_AWBCTR0 0x6F     //< AWB control 0
#define OV7670_REG_SCALING_XSC 0x70 //< Test pattern X scaling
#define OV7670_REG_SCALING_YSC 0x71 //< Test pattern Y scaling
#define OV7670_REG_SCALINGDCW 0x72  //< DCW control
#define OV7670_REG_SCALINGPCLK 0x73 //< DSP scale control clock divide
#define OV7670_REG_REG74 0x74       //< Digital gain control
#define OV7670_REG_REG76 0x76       //< Pixel correction
#define OV7670_REG_SLOP 0x7A        //< Gamma curve highest segment slope
#define OV7670_REG_GAM_BASE 0x7B    //< Gamma register base (1 of 15)
#define OV7670_GAM_LEN 15           //< Number of gamma registers
#define OV7670_R76_BLKPCOR 0x80     //< REG76 black pixel correction enable
#define OV7670_R76_WHTPCOR 0x40     //< REG76 white pixel correction enable
#define OV7670_REG_RGB444 0x8C      //< RGB 444 control
#define OV7670_R444_ENABLE 0x02     //< RGB444 enable
#define OV7670_R444_RGBX 0x01       //< RGB444 word format
#define OV7670_REG_DM_LNL 0x92      //< Dummy line LSB
#define OV7670_REG_LCC6 0x94        //< Lens correction option 6
#define OV7670_REG_LCC7 0x95        //< Lens correction option 7
#define OV7670_REG_HAECC1 0x9F      //< Histogram-based AEC/AGC control 1
#define OV7670_REG_HAECC2 0xA0      //< Histogram-based AEC/AGC control 2
#define OV7670_REG_SPD 0xA2         //< Scaling pixel clock delay
#define OV7670_REG_BD50MAX 0xA5     //< 50 Hz banding step limit
#define OV7670_REG_HAECC3 0xA6      //< Histogram-based AEC/AGC control 3
#define OV7670_REG_HAECC4 0xA7      //< Histogram-based AEC/AGC control 4
#define OV7670_REG_HAECC5 0xA8      //< Histogram-based AEC/AGC control 5
#define OV7670_REG_HAECC6 0xA9      //< Histogram-based AEC/AGC control 6
#define OV7670_REG_HAECC7 0xAA      //< Histogram-based AEC/AGC control 7
#define OV7670_REG_BD60MAX 0xAB     //< 60 Hz banding step limit
#define OV7670_REG_ABLC1 0xB1       //< ABLC enable
#define OV7670_REG_THL_ST 0xB3      //< ABLC target
#define OV7670_REG_SATCTR 0xC9      //< Saturation control

#define OV7670_REG_LAST OV7670_REG_SATCTR //< Maximum register address

#ifdef __cplusplus
extern "C" {
#endif

extern OV7670_status OV7670_begin(OV7670_host *host);

#ifdef __cplusplus
};
#endif
