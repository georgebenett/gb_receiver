# GB Receiver

Firmware for the receiver side of the GB Remote system: an ESP32-C3 on the
board that takes throttle from a [GB Remote](https://github.com/georgebenett/gb_remote)
over BLE and drives a VESC over CAN, and reports VESC and JBD (Jiabaida)/Kaly
BMS telemetry back to the remote.

> **Safety:** this firmware moves a vehicle and is experimental. Test every
> build with the wheels off the ground. When the BLE link drops without a
> shutdown command from the remote, the failsafe brakes the board.

## Features
- Throttle/brake from the remote, with smart reverse, reverse lockout,
  assistive push, speed limit and ride-profile current caps (set from the remote)
- VESC telemetry over CAN, JBD/Kaly BMS over UART, trip odometer
- Wi-Fi setup page (SoftAP when no remote is connected) for pairing and updates
- Firmware updates from the remote over BLE (the remotes bundle this image)

## Pairing
1. On the Wi-Fi setup page, scan for remotes (they advertise `GS-REM`) and add
   yours. The receiver then advertises only to remotes on that list.
2. The remote bonds with the passkey `CONFIG_GB_BLE_PASSKEY` (same value on
   both sides; the default is public, so set your own for builds you ship).
3. Throttle and commands are accepted only over the encrypted link.

## Build
ESP-IDF 5.5.x, target esp32c3:

```sh
idf.py build
./bundle_to_remotes.sh   # copy the image and gb_protocol.h into ../gb_remote*
```

Host tests: `for t in test/test_*.c; do cc -Wall -Wextra -o /tmp/t $t -lm && /tmp/t; done`

`main/gb_protocol.h` defines the remote ↔ receiver protocol; this repository
owns it and `bundle_to_remotes.sh` syncs it into the remotes.

## License
GPL-3.0-or-later (it includes GPL code from VESC); see [LICENSE](LICENSE) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). `main/gb_protocol.h` is MIT.
Report security issues as described in [SECURITY.md](SECURITY.md).
