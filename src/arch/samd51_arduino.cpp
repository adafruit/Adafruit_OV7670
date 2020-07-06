// This is the SAMD51- and Arduino-specific parts of OV7670 camera
// interfacing. Unlike the SAMD51-specific (but platform-agnostic) code
// in samd51.c, this adds a DMA layer. Camera data is continually read
// and then paused when needed, reducing capture latency (rather than
// waiting for a VSYNC before starting a capture of the next frame,
// VSYNC instead marks the end of the prior frame and data is ready).

#if defined(__SAMD51__) && defined(ARDUINO)
#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h" // pinPeripheral() function
#include "Adafruit_OV7670.h"

// Because interrupts exist outside the class context, but our interrupt
// needs to access to an active ZeroDMA object, a separate ZeroDMA pointer
// is kept (initialized in the begin() function). This does mean that only
// a single OV7670 can be active (probably no big deal, as there's only a
// single parallel capture peripheral).

static Adafruit_ZeroDMA *_dmaPtr = NULL;
static volatile bool _frameReady = false; // true at end-of-frame
static volatile bool _suspended = false;

// INTERRUPT HANDLING AND RELATED CODE -------------------------------------

// Pin interrupt on VSYNC calls this to start DMA transfer (unless suspended).
static void _startFrame(void) {
  if (_dmaPtr && !_suspended) {
    _frameReady = false;
    (void)_dmaPtr->startJob();
  }
}

// End-of-DMA-transfer callback
static void _dmaCallback(Adafruit_ZeroDMA *dma) { _frameReady = true; }

// Since ZeroDMA suspend/resume functions don't yet work, these functions
// use static vars to indicate whether to trigger DMA transfers or hold off
// (camera keeps running, data is simply ignored without a DMA transfer).

void Adafruit_OV7670::suspend(void) {
  while (!_frameReady)
    ;                // Wait for current frame to finish loading
  _suspended = true; // Don't load next frame (camera runs, DMA stops)
}

void Adafruit_OV7670::resume(void) {
  _frameReady = false;
  _suspended = false; // Resume DMA transfers
}


#endif // __SAMD51__ && ARDUINO









OV7670_status Adafruit_OV7670::arch_begin(void) {

  // BASE INITIALIZATION (PLATFORM-AGNOSTIC) -------------------------------
  // This calls the device-neutral C init function OV7670_begin(), which in
  // turn calls the device-specific C init OV7670_arch_begin() in samd51.c.

  OV7670_host_t host;
  // host.arch = arch
  // host.value = value;
  // etc.

  OV7670_status status;
  status = OV7670_begin(&host);
  if(status != OV7670_STATUS_OK) {
    return status;
  }

  // ARDUINO-SPECIFIC EXTRA INITIALIZATION ---------------------------------
  // Sets up DMA for the parallel capture controller.

  _dmaPtr = &dma; // Required global for interrupt function

  ZeroDMAstatus dma_status = dma.allocate();
  // To do: handle dma_status here, map to OV7670_status on error
  dma.setAction(DMA_TRIGGER_ACTON_BEAT);
  dma.setTrigger(PCC_DMAC_ID_RX);
  dma.setCallback(_dmaCallback);
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
  attachInterrupt(PIN_PCC_DEN1, _startFrame, FALLING);

  return status;
}

void Adafruit_OV7670::capture(void) {
  volatile uint32_t *vsync_reg, *hsync_reg;
  uint32_t vsync_bit, hsync_bit;

  vsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN1].ulPort].IN.reg;
  vsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN1].ulPin;
  hsync_reg = &PORT->Group[g_APinDescription[PIN_PCC_DEN2].ulPort].IN.reg;
  hsync_bit = 1ul << g_APinDescription[PIN_PCC_DEN2].ulPin;

  OV7670_capture((uint32_t *)buffer, _width, _height, vsync_reg, vsync_bit,
    hsync_reg, hsync_bit);
}
