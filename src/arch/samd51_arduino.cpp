// This is the SAMD51- and Arduino-specific parts of OV7670 camera
// interfacing. Unlike the SAMD51-specific (but platform-agnostic) code
// in samd51.c, this adds a DMA layer. Camera data is continually read
// and then paused when needed, reducing capture latency (rather than
// waiting for a VSYNC before starting a capture of the next frame,
// VSYNC instead marks the end of the prior frame and data is ready).

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

void Adafruit_OV7670::suspend(void) {
  while (!frameReady)
    ;               // Wait for current frame to finish loading
  suspended = true; // Don't load next frame (camera runs, DMA stops)
}

void Adafruit_OV7670::resume(void) {
  frameReady = false;
  suspended = false; // Resume DMA transfers
}

OV7670_status Adafruit_OV7670::arch_begin(OV7670_size size, float fps) {

  // BASE INITIALIZATION (PLATFORM-AGNOSTIC) -------------------------------
  // This calls the device-neutral C init function OV7670_begin(), which in
  // turn calls the device-specific C init OV7670_arch_begin() in samd51.c.

  OV7670_host host;
  host.arch = &arch; // Point to struct in Adafruit_OV7670 class
  host.pins = &pins; // Point to struct in Adafruit_OV7670 class
  if (arch_defaults) {
    arch.timer = TCC1;      // Use default timer
    arch.xclk_pdec = false; // and default pin MUX
  }
  host.platform = this; // Pointer back to Arduino_OV7670 object

  OV7670_status status;
  status = OV7670_begin(&host, size, fps);
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

// Host-specific .cpp must provide this function with same name, arguments
// and return type. Not a virtual function because it's not a subclass.
// Though wondering now if it should be. But I don't like the idea of each
// architecture requiring its own constructor name/call. Is that really
// any worse than setting up the arch struct though?

/*

Okay decided: will stick with arch struct rather than subclasses.
The middle-layer C code needs that struct to pass down to the arch-specific
C code anyway. With per-device constructors, the Arduino sketches will
need ifdefs anyway. Subclasses would mean a third place where per-device
ifdefs would need to happen.

Only arguments in favor of the subclass thing are
- Can put the DMA and descriptor instances in the subclass, allowing
multiple OV7670 instances. But since there's only one PCC anyway, there
can be only one instance.
Unless...having both PCC and bitbang instances present is an option.
So basically, multiple cameras. I think that's all it comes down to.
But we could do that with the arch structure anyway I think.

Update: addition of the OV7670_pins struct means that all constructors
could accept identical args now. Maybe that's more in the subclass' favor,

Just keeping this function though, avoids having to make a separate .h
file for each device. The .cpp is enough. It just has the missing funcs.

*/

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

#endif // __SAMD51__ && ARDUINO
