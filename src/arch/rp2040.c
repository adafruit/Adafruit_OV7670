// This is the RP2040-specific parts of OV7670 camera interfacing (device-
// agnostic parts are in ov7670.c). It configures and accesses hardware-
// specific peripherals (PWM, PIO, etc.). DMA is NOT configured here
// because it needs platform-specific information (i.e. the camera buffer
// and image dimensions that are part of the Arduino lib class).

// This will need to change since ARDUINO_ARCH_RP2040 is Arduino-platform-
// specific (possibly Philhower specific), while this file is supposed to
// be platform neutral and RP2040-specific.
#if defined(ARDUINO_ARCH_RP2040)
#include "ov7670.h"

// PIO code in this table is modified at runtime so that PCLK and VSYNC
// pins are configurable (rather than fixed GP##s). Data pins must be
// contiguous.
static uint16_t ov7670_pio_opcodes[] = {
#if 0
  0b0010000010000000, // WAIT 1 GPIO 0 (mask in VSYNC pin before use)
  0b0010000010000000, // WAIT 1 GPIO 0 (mask in PCLK pin before use)
  0b0100000000001000, // IN PINS 8
  0b0010000000000000, // WAIT 0 GPIO 0 (mask in PCLK pin before use)
#else
  // Since PCLK is masked through HSYNC, shouldn't need any checks,
  // just process PCLK directly.
  0b0010000010000000, // WAIT 1 GPIO 0 (mask in PCLK pin before use)
  0b0100000000001000, // IN PINS 8
  0b0010000000000000, // WAIT 0 GPIO 0 (mask in PCLK pin before use)
#endif
};

// On PCLK rising edge, delay before sampling IN pins.
// Determined empirically; might need to be a func of CPU or XCLK.
#define PIO_DELAY 16
// This doesn't make sense to me to delay this many cycles,
// but here we are.

struct pio_program ov7670_pio_program = {
  .instructions = ov7670_pio_opcodes,
  .length = sizeof ov7670_pio_opcodes / sizeof ov7670_pio_opcodes[0],
  .origin = -1,
};

// Each supported architecture MUST provide this function with this name,
// arguments and return type. It receives a pointer to a structure with
// at least a list of pins, and usually additional device-specific data
// attached to the 'arch' element.
OV7670_status OV7670_arch_begin(OV7670_host *host) {

  // LOOK UP PWM SLICE & CHANNEL BASED ON XCLK PIN -------------------------

  uint8_t slice = pwm_gpio_to_slice_num(host->pins->xclk);
  uint8_t channel = pwm_gpio_to_channel(host->pins->xclk);

  // CONFIGURE PWM FOR XCLK OUT --------------------------------------------
  // XCLK to camera is required for it to communicate over I2C!

  pwm_config config = pwm_get_default_config();
#if 0
  pwm_config_set_clkdiv(&config, (float)F_CPU / (float)(OV7670_XCLK_HZ * 2));
  pwm_config_set_wrap(&config, 1); // 2 clocks/cycle
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, 1); // 50% duty cycle
#else
  pwm_config_set_clkdiv(&config, 1); // PWM clock = F_CPU
  pwm_config_set_wrap(&config, F_CPU / OV7670_XCLK_HZ);
  pwm_init(slice, &config, true);
  pwm_set_chan_level(slice, channel, F_CPU / OV7670_XCLK_HZ / 2); // 50% duty
#endif
  gpio_set_function(host->pins->xclk, GPIO_FUNC_PWM);

  // SET UP PIO ------------------------------------------------------------

  // TO DO: verify the DATA pins are contiguous.

  // Set up GPIO pins (other than I2C, done in the Wire lib)
  gpio_init(host->pins->pclk); gpio_set_dir(host->pins->pclk, GPIO_IN);
  gpio_init(host->pins->vsync); gpio_set_dir(host->pins->vsync, GPIO_IN);
  gpio_init(host->pins->hsync); gpio_set_dir(host->pins->hsync, GPIO_IN);
  for(uint8_t i=0; i<8; i++) {
    gpio_init(host->pins->data[i]);
    gpio_set_dir(host->pins->data[i], GPIO_IN);
  }

  // Initially by default this always uses pio0, but I suppose this could be
  // written to use whatever pio has resources.
  host->arch->pio = pio0;

  // Mask the GPIO pins used for VSYNC and PCLK into the PIO opcodes
#if 0
  ov7670_pio_opcodes[0] |= (host->pins->vsync & 31);
  ov7670_pio_opcodes[1] |= (host->pins->pclk & 31);
  // Opcode 2 is unmodified
  ov7670_pio_opcodes[3] |= (host->pins->pclk & 31);
#else
  // See notes at top with the PIO code
  ov7670_pio_opcodes[0] |= (host->pins->pclk & 31);
  ov7670_pio_opcodes[0] |= PIO_DELAY << 8;
  // Opcode 1 is unmodified
  ov7670_pio_opcodes[2] |= (host->pins->pclk & 31);
#endif

  // Here's where resource check & switch to pio1 might go
  uint offset = pio_add_program(host->arch->pio, &ov7670_pio_program);
  host->arch->sm = pio_claim_unused_sm(host->arch->pio, true); // 0-3

  // host->pins->data[0] is data bit 0. PIO code requires all 8 data be
  // contiguous.
  pio_sm_set_consecutive_pindirs(host->arch->pio, host->arch->sm,
                                 host->pins->data[0], 8, false);

  pio_sm_config c = pio_get_default_sm_config();
  c.pinctrl = 0; // SDK fails to set this
  sm_config_set_wrap(&c, offset, offset + ov7670_pio_program.length - 1);

  sm_config_set_in_pins(&c, host->pins->data[0]);
  sm_config_set_in_shift(&c, false, true, 8); // 8 data bits
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);

  pio_sm_init(host->arch->pio, host->arch->sm, offset, &c);
  pio_sm_set_enabled(host->arch->pio, host->arch->sm, true);

  // This can improve GPIO responsiveness but is less noise-immune.
  uint32_t mask = (0xFF << host->pins->data[0]) | (1 << host->pins->pclk) |
                  (1 << host->pins->vsync);
  // Maybe not. Disabled for now.
  //host->arch->pio->input_sync_bypass = mask;

  // PIO read from camera also requires DMA and interrupts, which are NOT
  // set up here! These are done in the platform arch_begin(), as they
  // require platform-specific information such as the camera buffer
  // address and dimensions).

  // OTHER -----------------------------------------------------------------

  host->arch->pclk_mask = 1UL << host->pins->pclk;
  host->arch->vsync_mask = 1UL << host->pins->vsync;
  host->arch->hsync_mask = 1UL << host->pins->hsync;

  return OV7670_STATUS_OK;
}

// Non-DMA capture function using previously-initialized peripherals.
void OV7670_capture(uint16_t *dest, uint16_t width, uint16_t height,
                    volatile uint32_t *vsync_reg, uint32_t vsync_bit,
                    volatile uint32_t *hsync_reg, uint32_t hsync_bit) {

#if 0
  while (*vsync_reg & vsync_bit)
    ; // Wait for VSYNC low (frame end)
  OV7670_disable_interrupts();
  while (!*vsync_reg & vsync_bit)
    ; // Wait for VSYNC high (frame start)

  width /= 2;                             // PCC receives 2 pixels at a time
  for (uint16_t y = 0; y < height; y++) { // For each row...
    while (*hsync_reg & hsync_bit)
      ; //  Wait for HSYNC low (row end)
    while (!*hsync_reg & hsync_bit)
      ;                               //  Wait for HSYNC high (row start)
    for (int x = 0; x < width; x++) { //   For each column pair...
      while (!PCC->ISR.bit.DRDY)
        ;                     //    Wait for PCC data ready
      *dest++ = PCC->RHR.reg; //    Store 2 pixels
    }
  }

  OV7670_enable_interrupts();
#endif
}

// DEVICE-SPECIFIC FUNCTIONS FOR NON-ARDUINO PLATFORMS ---------------------

// If a platform doesn't offer a device-agnostic function for certain
// operations (i.e. can't be trivially remapped in ov7670.h as is done for
// Arduino), those device-specific functions can be declared here, with a
// platform-specific #ifdef around them. These functions may include:
// OV7670_delay_ms(x)
// OV7670_pin_output(pin)
// OV7670_pin_write(pin, hi)
// OV7670_disable_interrupts()
// OV7670_enable_interrupts()
// Also see notes at top regarding pin MUXing in this file.

#endif // end ARDUINO_ARCH_RP2040
