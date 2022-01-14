// SPDX-FileCopyrightText: 2020 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once
#if defined(__SAMD51__) // SAMD51 only
#include <SPI.h>
#define USE_SPI_BRUTE

class SPIBrute {
public:
  SPIBrute(SPIClass *s);
  void begin(void);
  void wait(void);
  void write(uint8_t *addr, uint32_t len);

private:
  union {
    SPIClass *spi;  ///< Before begin(), holds SPI class pointer
    Sercom *sercom; ///< After begin(), holds SERCOM base address
  };
};

#endif // __SAMD51__
