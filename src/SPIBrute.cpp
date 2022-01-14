// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// This class handles fast-as-possible writes to SPI, using direct register
// accesses. It's used for TFT updates in the camera examples when
// USE_SPI_DMA is not enabled in Adafruit_SPITFT.h (part of Adafruit_GFX).
// DMA is only enabled by default on certain boards, mostly ones with
// built-in screens (e.g. PyPortal), so it's worth editing if testing on
// Grand Central, etc. The mentions of SERCOMs and registers in this code
// are entirely SAMD-specific.

#if defined(__SAMD51__) // SAMD51 only

#include "SPIBrute.h"

// Constructor. Pass SPIClass pointer (e.g. &SPI).
SPIBrute::SPIBrute(SPIClass *s) : spi(s) {}

// Call this function after the SPI peripheral has been started
// (can't do this in the constructor).
void SPIBrute::begin(void) {
  // List of SERCOM base addresses by SERCOM #
  static Sercom *sercomBase[] = {
    SERCOM0,
    SERCOM1,
    SERCOM2,
    SERCOM3,
#if defined(SERCOM4)
    SERCOM4,
#endif
#if defined(SERCOM5)
    SERCOM5,
#endif
#if defined(SERCOM6)
    SERCOM6,
#endif
#if defined(SERCOM7)
    SERCOM7,
#endif
  };

  // There's no direct route from an SPI peripheral instance to a
  // SERCOM base pointer. There IS a function to get the INDEX of
  // a SERCOM associated with an SPI instance, so we use that, plus
  // the list above, to get the screen's SERCOM base address.
  sercom = sercomBase[spi->getSercomIndex()];
  // We're done with the value of spi at this point. spi and sercom
  // are a union, so this overwrites one with the other.
}

// Brute force (non-DMA) fast SPI writing function, accesses SPI registers
// directly. This is "mostly blocking" except for the last byte out. Call
// wait() before performing another write() or any action on the SPI bus.
void SPIBrute::write(uint8_t *addr, uint32_t len) {
  // Manual byte-by-byte SPI transfer. Have tested & confirmed on scope
  // that there is no inter-byte gap when running this loop...it's as fast
  // as can be. Tried 32-bit SPI transfers...wasn't any faster, in fact
  // required small delays when switching between 32- and 8-bit for TFT
  // setAddrWindow() calls. So keep this as-is...
  while (len--) {
    while (sercom->SPI.INTFLAG.bit.DRE == 0)
      ;                             // Wait for Data Register Empty
    sercom->SPI.DATA.reg = *addr++; // Write next byte
  }
}

// write() is not entirely blocking. Call wait() before more writes.
void SPIBrute::wait(void) {
  while (sercom->SPI.INTFLAG.bit.DRE == 0)
    ; // Wait for Data Register Empty
}

#endif // __SAMD51__
