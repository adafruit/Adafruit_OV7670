# Adafruit_OV7670

Arduino library for OV7670 cameras, built over a C foundation common to
both Arduino and CircuitPython.

These notes are primarily to assist with porting to new devices and in
creating a CircuitPython library from the lower-level code. For using the
library, see the included Arduino example(s).

## Terminology

In order to understand, discuss and adapt the code, a basic vernacular
must be established. The terms chosen here are probably not optimal, but
applied consistently will suffice in allowing different elements of the
code to be discussed relative to one another. These are the terms used
in the comments and in naming of structures and types.

An "architecture" refers to a related family of microcontroller devices
where certain peripherals are implemented in-common. For example, SAMD51
is an architecture distinct from SAMD21 (though both ARM-based), because
these have a different range of peripherals, and even when there's overlap
in functionality, the registers for configuring these peripherals may be
different. (Note that SAMD51 is initially the only supported architecture,
and the mention of SAMD21 is purely hypothetical. Others might be added in
the future -- STM32, i.MX, etc. -- the code is written with that in mind
even though there's currently just one architecture.)

The source files make a reasonable attempt to keep architecture-dependent
and device-neutral C or C++ code in separate files. There might be a few
exceptions, always documented in the source.

"Platform" as used in this code refers to a runtime environment, such
as Arduino or CircuitPython. Could just as easily been "environment" or
"language," but we'll use "platform" despite its slight ambiguity (in
other settings it might be applied the way we use architecture, e.g. "the
ARM platform" or similar). But here we are. It's fine.

"Host" refers to a specific combination of architecture and platform (along
with a list of pins to which an OV7670 camera is connected). Picture a table
with architectures along one axis and platforms along the other. "SAMD51
Arduino" is one host, "SAMD51 CircuitPython" is another.

## SPIBrute Arduino class

An extra class in here called "SPIBrute" is NOT RELATED to the OV7670
camera itself. This SAMD51-specific class can be used to issue data to
SPI-connected displays when DMA is not enabled in Adafruit_SPITFT (part
of Adafruit_GFX). It's just a constructor and one function. These display
writes are still blocking operations, but the timing is particularly tight
and avoids inter-byte delays sometimes seen with SPI.transfer(), that's all.

## Files

`examples/`

`  cameratest/`

`    cameratest.ino`  Grand Central demo, OV7670 to ILI9341 TFT shield

`src/`

`  Adafruit_OV7670.cpp`  Arduino class functions

`  Adafruit_OV7670.h`    Arduino class header

`  SPIBrute.cpp`         SAMD51-specific class, see notes above

`  SPIBrute.h`           same

`src/arch/`

`  ov7670.c`             Architecture- and platform-neutral functions in C

`  ov7670.h`             Header for ov7670.c

`  samd51.c`             SAMD51 arch-specific, platform-neutral functions in C

`  samd51.h`             Header for samd51.c

`  samd51_arduino.cpp`   SAMD51 arch- and Arduino-specific functions in C++

Architecture- and/or platform-specific files contain #ifdef checks since some
platforms (e.g. Arduino) will compile all source files in the directory and
would otherwise break.

When adding support for a new device at the C level, you MUST #include the
corresponding header file in ov7670.h.

Finer details are in comments peppered throughout the source.
