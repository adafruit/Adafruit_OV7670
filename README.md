# Adafruit OV7670 Library [![Build Status](https://github.com/adafruit/Adafruit_OV7670/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_OV7670/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_OV7670/html/index.html)

Arduino library for OV7670 cameras, built over a C foundation common to
both Arduino and (later) CircuitPython.

These notes are primarily to assist with porting to new devices and in
creating a CircuitPython library from the lower-level code. For using the
library, see the included Arduino example(s).

## Terminology

To understand, discuss and adapt the code, a basic vernacular should be
established. The terms chosen here are probably not optimal, but applied
consistently will suffice in allowing different elements of the code to
be discussed relative to one another. These are the terms used in the
comments and in naming of structures and types.

An "architecture" refers to a related family of microcontroller devices
where certain peripherals are implemented in-common. For example, SAMD51
is an architecture distinct from SAMD21 (though both ARM-based), because
these have a different range of peripherals, and even when there's
functional overlap, the registers for configuring these peripherals are
often a bit different. (Note that SAMD51 is initially the only supported
architecture, and the mention of SAMD21 is purely hypothetical and might
never happen. Others might be added in the future -- STM32, i.MX, etc. --
the code is written with that in mind even though there's currently just
one architecture.)

"Platform" as used in this code refers to a runtime environment, such
as Arduino or CircuitPython. Could just as easily been "environment" or
"language" or something else, but we'll use "platform," it's good enough.

"Host" refers to a specific combination of architecture and platform (along
with a list of pins to which an OV7670 camera is connected). Picture a table
with architectures along one axis and platforms along the other. "SAMD51
Arduino" is one host, "SAMD51 CircuitPython" is another.

## Code structure

The project is written in a combination of C and C++. The C++ parts are
currently intended for the Arduino platform. CircuitPython development
requires C at present, so most lower-level functionality is implemented
in that language. The C part itself has two layers...one "mid layer" set of
functions that are common to all architectures, and a "lower" layer that
does architecture-specific things such as accessing peripheral registers.
CircuitPython support, when implemented, will probably have its own topmost
C layer, akin to the Arduino C++ layer.

The code makes a reasonable attempt to keep architecture and platform
-neutral vs -dependent C or C++ code in separate files. There might be a
few exceptions with #ifdefs around them, always documented in the source.
Also in comments, the terms "neutral" or "agnostic" might be used in
different places, but same intent: non-preferential.

## SPIBrute Arduino class

An extra class in here called "SPIBrute" is NOT RELATED to the OV7670
camera itself. This SAMD51-specific class can be used to issue data to
SPI-connected displays when DMA is not enabled in Adafruit_SPITFT.h (part
of Adafruit_GFX). It's just a constructor and a couple functions. These
display writes are still blocking operations, but the timing is particularly
tight and avoids small inter-byte delays sometimes seen with SPI.transfer(),
that's all.

## Files

    examples/
        cameratest/
            cameratest.ino   Grand Central demo, OV7670 to ILI9341 TFT shield
    src/
        Adafruit_OV7670.cpp  Arduino C++ class functions
        Adafruit_OV7670.h    Arduino C++ class header
        SPIBrute.cpp         SAMD51-specific C++ class, see notes above
        SPIBrute.h           C++ header for SPIBrute.cpp
    src/arch/                Mid-layer C, plus arch-specific code
        ov7670.c             Architecture- and platform-neutral functions in C
        ov7670.h             C header for ov7670.c
        samd51.c             SAMD51 arch-specific, platform-neutral C functions
        samd51.h             Header for samd51.c
        samd51_arduino.cpp   SAMD51 arch- and Arduino-specific C++ functions

Architecture- and/or platform-specific files should contain #ifdef checks
since some platforms (e.g. Arduino) will wanton compile all source files in
the directory and would otherwise break.

**When adding support for a new device at the C level, you MUST #include
the corresponding header file in ov7670.h, so the C and C++ code that build
atop this will pick up the specifics for the device.**

Finer details are in comments peppered throughout the source.

The Arduino library and examples make use of an architecture-specific "arch"
structure. This is kind of gross, but after much deliberation it seemed just
slightly less gross than the alternative. There's some notes in the .cpp.

## OV7670 notes

The library currently provides five resolution settings from full VGA
(640x480 pixels, RAM permitting, which it isn't), and powers-of-two
divisions of this, down to 1:16 (40x30 pixels).

An intentional choice was made to ignore the camera's CIF resolution options
for the sake of getting things done. CIF is an antiquated throwback to
analog PAL video and doesn't really get us much, being just slightly larger
than the nearest corresponding VGA division.

IN THEORY the camera can provide nearly any resolution from CIF down to
40x30, with independent scaling on each axis. In practice this has proven
difficult to implement, as it's not well explained in documentation or
example code. Maybe this will be revisited in the future if it's needed,
at which point CIF (and any other resolution) will happen implicitly.
But for now, just the VGA powers-of-two.

At the smallest size (40x30), there are artifacts in the first row and
column that I've not been able to eliminate. Any software using this
setting should try to mask out or otherwise ignore those pixels.

Additionally, only RGB565 color is currently supported.
