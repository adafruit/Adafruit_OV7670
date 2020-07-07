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

static Adafruit_ZeroDMA dma;
static DmacDescriptor *descriptor;     ///< DMA descriptor
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
static void _dmaCallback(Adafruit_ZeroDMA *dma) { frameReady = true; }

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


#endif // __SAMD51__ && ARDUINO









OV7670_status Adafruit_OV7670::arch_begin(void) {

  // BASE INITIALIZATION (PLATFORM-AGNOSTIC) -------------------------------
  // This calls the device-neutral C init function OV7670_begin(), which in
  // turn calls the device-specific C init OV7670_arch_begin() in samd51.c.

  OV7670_host host;
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
  attachInterrupt(PIN_PCC_DEN1, startFrame, FALLING);

  return status;
}

// Host-specific .cpp must provide this function with same name, arguments
// and return type. Not a virtual function because it's not a subclass.
// Though wondering now if it should be. But I don't like the idea of each
// architecture requiring its own constructor name/call. Is that really
// any worse than setting up the arch struct though?

/*
ifdef device
  arch = { ... }
endif
constructor( ..., arch)

vs
if device
  constructor ( ..., ... )
endif

I suppose it's fewer lines that way

There would then be
Adafruit_OV7670_SAMD51
Adafruit_OV7670_iMX
Adafruit_OV7670_ESP32
and so forth
but never simply
Adafruit_OV7670

OH WAIT.
I think the reason this is NOT done
is that sketches would need to include a specific .h for each arch,
i.e. #include <Adafruit_OV7670_SAMD51>
and THEN the special declaration also
and I don't really like that.

I suppose it's possible for a main .h to include any arch-specific .h,
and then always name the subclass the same thing, but with different
constructor args.
So there'd be like Adafruit_OV7670_core that the user never sees,
and Adafruit_OV7670 (a subclass of core) that they do see, and which
has different arguments depending on arch.
Adafruit_OV7670.h would include the core class and then
all the subclass .h files.
Seems a mess. But so is the arch struct.
Also, having the virtual func would make it easier to enforce that
architectures implement specific functions by name.

Well...the arch struct goes down to the C code anyway, and is going
to exist regardles. So maybe just stick with it? It inflicts a lot
on the user, but that info needs to go somewhere anyway, whether in
the constructor or in a special struct.

Okay decided: will stick with arch struct rather than subclasses.
The middle-layer C code needs that struct to pass down to the arch-specific
C code anyway. With per-device constructors, the Arduino sketches will
need ifdefs anyway. Subclasses would mean a third place where per-device
ifdefs would need to happen.

Only arguments in favor of the subclass thing are
- can use defaults for those weird arguments in constructor, and if they're
good defaults, the user will never need to see them and the constructor
looks the same on most devices.
- Can put the DMA and descriptor instances in the subclass, allowing
multiple OV7670 instances. But since there's only one PCC anyway, there
can be only one instance.
Unless...having both PCC and bitbang instances present is an option.
So basically, multiple cameras. I think that's all it comes down to.
But we could do that with the arch structure anyway I think.


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
