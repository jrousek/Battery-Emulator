# Battery Emulator – SOLAVOLT edition

This is a modified version of [Battery Emulator](https://github.com/dalathegreat/Battery-Emulator)
by dalathegreat and the Battery Emulator contributors, distributed under the GNU General Public
License version 3 (see `LICENSE`). It comes WITHOUT ANY WARRANTY.

Modified by SOLAVOLT s.r.o., based on upstream `main` commit `7a290035` (16 November 2026).

## Modifications

| Date | Change |
|---|---|
| 2026-11-17 | Pylon HV: selectable BMS address, V1.27+ frames 0x42B0/0x42C0/0x42D0, relay control modes (force relay 0x822x, sleep/wake 0x820x) with close delay and comm-loss handling, diagnostics on the battery page |
| 2026-11-18 | Bench debug console for the Pylon relay commands (`/pylondbg`, USB serial) |
| 2026-11-18 | MQTT over TLS with a configured CA certificate, publish-only option |
| 2026-11-18 | SOLAVOLT branding: logo, edition name, license page `/license` and a notice line on every page |

The git history of this repository lists every change in detail.
