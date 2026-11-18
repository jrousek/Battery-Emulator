#ifndef PYLON_DEBUG_CONSOLE_H
#define PYLON_DEBUG_CONSOLE_H

#include <Arduino.h>

// Text console for bench-testing the Pylon relay commands (see battery/PYLON-DEBUG.h).
// The same commands work from the web route /pylondbg?c=... and from the USB serial port.
String pylon_debug_command(const String& line);

// Reads command lines from the USB serial port. Call from the Arduino loop().
void pylon_debug_poll_serial();

#endif
