#ifndef SOLAVOLT_BRANDING_H
#define SOLAVOLT_BRANDING_H

#include <stddef.h>
#include <stdint.h>

// SOLAVOLT edition of Battery Emulator: the logo and the legal notices the GPL asks a modified
// version to carry (section 5: who modified it and when; section 5d: the notices shown in an
// interactive interface; section 6: where the corresponding source is).

// Where the source code of this edition is published.
#define SOLAVOLT_SOURCE_URL "https://github.com/jrousek/Battery-Emulator"

// /logo.png, from web_data/solavolt_logo.png (solavolt_logo.cpp is generated from it).
extern const uint8_t solavolt_logo_png[];
extern const size_t solavolt_logo_png_len;

// One line at the bottom of every page, linking to /license.
#define SOLAVOLT_FOOTER_HTML                                                                     \
  "<p style='font:12px Arial,sans-serif;color:#888;margin:30px 0 10px'>SOLAVOLT edition of "     \
  "Battery Emulator &middot; free software under the GNU GPL v3, without any warranty &middot; " \
  "<a href='/license' style='color:#888'>license and source code</a></p>"

// The logo on a white plate, as the logo itself is drawn for a light background.
#define SOLAVOLT_LOGO_HTML                                                                                 \
  "<div style='display:inline-block;background:#fff;padding:8px 14px;border-radius:10px;margin-top:10px'>" \
  "<img src='/logo.png' alt='SOLAVOLT' width='209' height='33' style='display:block'></div>"

#define SOLAVOLT_LICENSE_PAGE                                                                                          \
  "<!doctype html><html><head><meta charset='utf-8'><meta name=viewport content='width=device-width'>"                 \
  "<title>License - Battery Emulator SOLAVOLT edition</title>"                                                         \
  "<style>body{background:#000;color:#ddd;font:15px/1.5 Arial,sans-serif;max-width:760px;margin:0 auto;"               \
  "padding:16px}a{color:#7fb3ff}h2,h3{color:#fff}</style></head><body>" SOLAVOLT_LOGO_HTML                             \
  "<h2>Battery Emulator &ndash; SOLAVOLT edition</h2>"                                                                 \
  "<p>This firmware is a modified version of <a href='https://github.com/dalathegreat/Battery-Emulator'>"              \
  "Battery Emulator</a>, Copyright &copy; dalathegreat and the Battery Emulator contributors.</p>"                     \
  "<p>Modified by SOLAVOLT s.r.o., 2026: Pylon HV relay control and diagnostics for JBD BMS, a CAN debug "             \
  "console for bench testing, MQTT over TLS with a publish-only option, and SOLAVOLT branding.</p>"                    \
  "<h3>License</h3>"                                                                                                   \
  "<p>This program is free software: you can redistribute it and/or modify it under the terms of the GNU "             \
  "General Public License version 3 as published by the Free Software Foundation.</p>"                                 \
  "<p>This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even "         \
  "the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the "                              \
  "<a href='https://www.gnu.org/licenses/gpl-3.0.html'>GNU General Public License</a> for more details.</p>"           \
  "<h3>Source code</h3>"                                                                                               \
  "<p>The complete source code of this edition is available at <a href='" SOLAVOLT_SOURCE_URL "'>" SOLAVOLT_SOURCE_URL \
  "</a>.</p>"                                                                                                          \
  "<h3>Third-party components</h3><ul>"                                                                                \
  "<li>ElegantOTA &ndash; AGPL-3.0</li>"                                                                               \
  "<li>ESPAsyncWebServer, AsyncTCP, Adafruit NeoPixel &ndash; LGPL-3.0</li>"                                           \
  "<li>ArduinoJson, eModbus, acan-esp32 &ndash; MIT</li>"                                                              \
  "<li>Arduino core for ESP32, ESP-IDF &ndash; LGPL-2.1 / Apache-2.0</li></ul>"                                        \
  "<p><a href='/'>Back to main page</a></p></body></html>"

#endif  // SOLAVOLT_BRANDING_H
