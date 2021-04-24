/*
This is the RP2040- and Arduino-specific parts of OV7670 camera interfacing.
Unlike the RP2040-specific (but platform-agnostic) code in rp2040.c, this
adds a DMA layer. Camera data is continually read and then paused when
needed, reducing capture latency (rather than waiting for a VSYNC before
starting a capture of the next frame, VSYNC instead marks the end of the
prior frame and data is ready). Code for non-DMA use of the SAMD51 PCC
exists in samd51.c, but isn't really used right now, unless we add a
DMA/non-DMA compile- or run-time capability. It's all DMA for now.

Each architecture will have a .cpp file like this, to tie into the other
Arduino code up a level and provide a couple of missing functions there.
An intentional choice was made NOT to make a subclass for each architecture,
overloaded or virtual functions, etc. even if that might be more Proper
Computer Science(tm). It avoids more per-arch ifdefs in the examples,
avoids an extra .h file to accompany each per-arch .cpp, etc. Instead,
arch-specific constructs are handled through the 'arch' struct which gets
passed around, and needed by the middle-layer C code anyway. Just provide
arch_begin() and capture() in each .cpp and call it done.
*/

#if defined(ARDUINO_ARCH_RP2040)
#include "Adafruit_OV7670.h"
#include "hardware/irq.h"
#include "wiring_private.h" // pinPeripheral() function
#include <Arduino.h>

// Because interrupts exist outside the class context, but our interrupt
// needs to access to object- and arch-specific data like the camera buffer
// and DMA settings, pointers are kept (initialized in the begin() function).
// This does mean that only a single OV7670 can be active.
Adafruit_OV7670 *platformptr = NULL; // Camera buffer, size, etc.
OV7670_arch *archptr = NULL;         // DMA settings

// INTERRUPT HANDLING AND RELATED CODE -------------------------------------

static volatile bool frameReady = false; // true at end-of-frame
static volatile bool suspended = false;

// To do: use the arch state variable to detect when a VSYNC IRQ occurs
// before the last DMA transfer is complete (suggesting one or more pixels
// dropped, likely bad PCLK signal). If that happens, abort the DMA
// transfer and return, skip frames and  don't begin next DMA until abort
// has completed (this is what state var is for, to detect if in this
// holding pattern).

// Pin interrupt on VSYNC calls this to start DMA transfer (unless suspended).
static void ov7670_vsync_irq(uint gpio, uint32_t events) {
  if (!suspended) {
    frameReady = false;
    // Clear PIO FIFOs and start DMA transfer
    pio_sm_clear_fifos(archptr->pio, archptr->sm);
    dma_channel_start(archptr->dma_channel);
  }
}

static void ov7670_dma_finish_irq() {
  // DMA transfer completed. Set up (but do not trigger) next one.
  frameReady = true;
  // Channel MUST be reconfigured each time (to reset the dest address).
  dma_channel_set_write_addr(archptr->dma_channel,
                             (uint8_t *)(platformptr->getBuffer()), false);
  dma_hw->ints0 = 1u << archptr->dma_channel; // Clear IRQ
}

// This is NOT a sleep function, it just pauses background DMA.

void Adafruit_OV7670::suspend(void) {
  while (!frameReady)
    ;               // Wait for current frame to finish loading
  suspended = true; // Don't load next frame (camera runs, DMA stops)
}

// NOT a wake function, just resumes background DMA.

void Adafruit_OV7670::resume(void) {
  frameReady = false;
  suspended = false; // Resume DMA transfers
}

OV7670_status Adafruit_OV7670::arch_begin(OV7670_colorspace colorspace,
                                          OV7670_size size, float fps) {

  // BASE INITIALIZATION (PLATFORM-AGNOSTIC) -------------------------------
  // This calls the device-neutral C init function OV7670_begin(), which in
  // turn calls the device-specific C init OV7670_arch_begin() in samd51.c.
  // So many layers. It's like an ogre.

  // I'm not sure what Past Me was thinking with the host struct here as
  // a local variable passed to OV7670_begin() and then just goes away.
  // Maybe it was supposed to be a static global in this file (replacing
  // the separate archptr and platformptr pointers)?
  OV7670_host host;
  host.arch = &arch; // Point to struct in Adafruit_OV7670 class
  host.pins = &pins; // Point to struct in Adafruit_OV7670 class
  if (arch_defaults) {
    // See SAMD code for what could go here
  }
  host.platform = this; // Pointer back to Arduino_OV7670 object

  OV7670_status status;
  status = OV7670_begin(&host, colorspace, size, fps);
  if (status != OV7670_STATUS_OK) {
    return status;
  }

// See notes in rp2040.c -- a change to the PIO (waiting for HSYNC)
// seems to fix this, can then keep same init sequence as SAMD.
#if 0
  // PIO code requires masking PCLK through the HSYNC signal
  OV7670_write_register(this, OV7670_REG_COM10,
    OV7670_COM10_VS_NEG | OV7670_COM10_PCLK_HB);
#endif

  // ARDUINO-SPECIFIC EXTRA INITIALIZATION ---------------------------------

  // See notes at top re: pointers
  archptr = &arch;    // For access to DMA settings
  platformptr = this; // For access to buffer & dimensions

  // SET UP DMA ------------------------------------------------------------

  arch.dma_channel = dma_claim_unused_channel(false); // don't panic

  arch.dma_config = dma_channel_get_default_config(arch.dma_channel);
  channel_config_set_transfer_data_size(&arch.dma_config, DMA_SIZE_16);
  channel_config_set_read_increment(&arch.dma_config, false);
  channel_config_set_write_increment(&arch.dma_config, true);
  // Set PIO RX as DMA trigger. Input shift register saturates at 16 bits
  // (1 pixel), configured in data size above and in PIO setup elsewhere.
  channel_config_set_dreq(&arch.dma_config,
                          pio_get_dreq(arch.pio, arch.sm, false));
  // Set up initial DMA xfer, but don't trigger (that's done in interrupt)
  dma_channel_configure(arch.dma_channel, &arch.dma_config, getBuffer(),
                        &arch.pio->rxf[arch.sm], width() * height(), false);

  // Set up end-of-DMA interrupt
  dma_channel_set_irq0_enabled(arch.dma_channel, true);
  irq_set_exclusive_handler(DMA_IRQ_0, ov7670_dma_finish_irq);
  irq_set_enabled(DMA_IRQ_0, true);

  // SET UP VSYNC INTERRUPT ------------------------------------------------

  gpio_set_irq_enabled_with_callback(pins.vsync, GPIO_IRQ_EDGE_RISE, true,
                                     &ov7670_vsync_irq);

  return status;
}

void Adafruit_OV7670::capture(void) {
  volatile uint32_t *vsync_reg, *hsync_reg;
  uint32_t vsync_bit, hsync_bit;

  // Note to future self: might add DMA pause-and-return-immediately
  // into this function, so it's usable the same for both DMA and
  // non-DMA situations. Calling code would still need to resume DMA
  // when it's done with the data, wouldn't be completely transparent.

#if 0
  vsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN1].ulPort].IN.reg;
  vsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN1].ulPin;
  hsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN2].ulPort].IN.reg;
  hsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN2].ulPin;

  OV7670_capture((uint32_t *)buffer, _width, _height, vsync_reg, vsync_bit,
                 hsync_reg, hsync_bit);
#endif
}

#endif // ARDUINO_ARCH_RP2040
