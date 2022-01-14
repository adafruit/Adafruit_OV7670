// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

/*
This is the SAMD51- and Arduino-specific parts of OV7670 camera interfacing.
Unlike the SAMD51-specific (but platform-agnostic) code in samd51.c, this
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

#if defined(__SAMD51__) && defined(ARDUINO)
#include "Adafruit_OV7670.h"
#include "wiring_private.h" // pinPeripheral() function
#include <Adafruit_ZeroDMA.h>
#include <Arduino.h>

// Because interrupts exist outside the class context, but our interrupt
// needs to access to an active ZeroDMA object, a separate ZeroDMA pointer
// is kept (initialized in the begin() function). This does mean that only
// a single OV7670 can be active (probably no big deal, as there's only a
// single parallel capture peripheral).

static Adafruit_ZeroDMA dma;
static DmacDescriptor *descriptor;       ///< DMA descriptor
static volatile bool frameReady = false; // true at end-of-frame
static volatile bool suspended = false;

// INTERRUPT HANDLING AND RELATED CODE -------------------------------------

// Pin interrupt on VSYNC calls this to start DMA transfer (unless suspended).
static void startFrame(void) {
  if (!suspended) {
    frameReady = false;
    (void)dma.startJob();
  }
}

// End-of-DMA-transfer callback
static void dmaCallback(Adafruit_ZeroDMA *dma) { frameReady = true; }

// Since ZeroDMA suspend/resume functions don't yet work, these functions
// use static vars to indicate whether to trigger DMA transfers or hold off
// (camera keeps running, data is simply ignored without a DMA transfer).

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

  OV7670_host host;
  host.arch = &arch; // Point to struct in Adafruit_OV7670 class
  host.pins = &pins; // Point to struct in Adafruit_OV7670 class
  if (arch_defaults) {
    arch.timer = TCC1;      // Use default timer
    arch.xclk_pdec = false; // and default pin MUX
  }
  host.platform = this; // Pointer back to Arduino_OV7670 object

  OV7670_status status;
  status = OV7670_begin(&host, colorspace, size, fps);
  if (status != OV7670_STATUS_OK) {
    return status;
  }

  // ARDUINO-SPECIFIC EXTRA INITIALIZATION ---------------------------------
  // Sets up DMA for the parallel capture controller.

  ZeroDMAstatus dma_status = dma.allocate();
  // To do: handle dma_status here, map to OV7670_status on error
  dma.setAction(DMA_TRIGGER_ACTON_BEAT);
  dma.setTrigger(PCC_DMAC_ID_RX);
  dma.setCallback(dmaCallback);
  dma.setPriority(DMA_PRIORITY_3);

  // Use 32-bit PCC transfers (4 bytes accumulate in RHR.reg)
  descriptor = dma.addDescriptor((void *)(&PCC->RHR.reg), // Move from here
                                 (void *)buffer,          // to here
                                 _width * _height / 2,    // this many
                                 DMA_BEAT_SIZE_WORD,      // 32-bit words
                                 false,                   // Don't src++
                                 true);                   // Do dest++

  // A pin FALLING interrupt is used to detect the start of a new frame.
  // Seems like the PCC RXBUFF and/or ENDRX interrupts could take care
  // of this, but in practice that didn't seem to work.
  // DEN1 is the PCC VSYNC pin.
  attachInterrupt(PIN_PCC_DEN1, startFrame, FALLING);

  return status;
}

void Adafruit_OV7670::capture(void) {
  volatile uint32_t *vsync_reg, *hsync_reg;
  uint32_t vsync_bit, hsync_bit;

  // Note to future self: might add DMA pause-and-return-immediately
  // into this function, so it's usable the same for both DMA and
  // non-DMA situations. Calling code would still need to resume DMA
  // when it's done with the data, wouldn't be completely transparent.

  vsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN1].ulPort].IN.reg;
  vsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN1].ulPin;
  hsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN2].ulPort].IN.reg;
  hsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN2].ulPin;

  OV7670_capture((uint32_t *)buffer, _width, _height, vsync_reg, vsync_bit,
                 hsync_reg, hsync_bit);
}

#endif // __SAMD51__ && ARDUINO
