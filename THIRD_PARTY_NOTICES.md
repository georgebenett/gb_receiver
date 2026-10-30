# Third-party notices

| Component | Where | License |
|---|---|---|
| VESC `buffer`, `crc`, `datatypes.h` (Benjamin Vedder) and the CAN interface built on them | `components/vesc/` | GPL-3.0-or-later |
| ESP-IDF BLE SPP server example (basis of `main/ble.c`) | Espressif Systems | Unlicense OR CC0-1.0 |
| miniz (inflate for BLE updates) | ESP-IDF ROM / components | MIT |
| ESP-IDF | build toolchain | Apache-2.0 |

Because it links the GPL VESC code, gb_receiver as a whole is GPL-3.0-or-later
(see LICENSE). `main/gb_protocol.h` is MIT so the remotes, which are licensed
separately, can share it.
