<!--
SPDX-FileCopyrightText: 2026 Cyril Brunet

SPDX-License-Identifier: MIT
-->

# aj_sr04m — get-started example

Minimal ESP-IDF project that exercises the [`aj_sr04m`](../../) driver: initialize the sensor, trigger a measurement every couple of seconds, and log the distance (or the error status) to the console.

## What it does

1. `aj_sr04m_init()` configures the GPIO + RMT capture (modes 1 and 2) or the UART (modes 3, 4 and 5), depending on the `AJ_SR04M_MODE` selected in `menuconfig`.
2. In a loop: `aj_sr04m_trigger()` starts a measurement, `aj_sr04m_read_duration()` reads the result, and `ESP_LOGI` / `ESP_LOGW` print either the distance in mm or the failure status.
3. Polling cadence is 1 ms in mode 3 (the module emits frames autonomously) and 2 s in every other mode.

## Hardware

| Module pin | ESP32 default GPIO | Role |
| ---------- | ------------------ | ---- |
| `5V` / `VCC` | 3.3 V or 5 V supply | Power (both are supported by the AJ-SR04M). |
| `GND` | `GND` | |
| `TRIG` (modes 1, 2) or `RX` (modes 3, 4, 5) | GPIO 17 | ESP32 TX in UART modes — configurable via menuconfig. |
| `ECHO` (modes 1, 2) or `TX` (modes 3, 4, 5) | GPIO 16 | ESP32 RX in UART modes — configurable via menuconfig. |

If you power the module from 5 V, level-shift the module's TX line with a 10 kΩ series + 20 kΩ to-ground divider before connecting to the ESP32 RX pin. See the component [`README.md`](../../README.md) for the full wiring guide.

## Build, flash, monitor

```bash
idf.py set-target esp32        # or esp32s3, esp32c3, ...
idf.py menuconfig              # AJ-SR04M Configuration -> Sensor operating mode, pins, UART
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

All sensor parameters are surfaced in `idf.py menuconfig` under **AJ-SR04M Configuration**:

- Sensor variant (AJ-SR04M / JSN-SR04T) — controls the UART trigger byte (0x01 / 0x55).
- Operating mode (1-5) — must match the R19 resistor on the physical module.
- Trigger / UART TX GPIO (default 17).
- Echo / UART RX GPIO (default 16).
- UART port number (modes 3-5 only, default 2 on ESP32 / ESP32-S3).

The bundled `sdkconfig.defaults` pins the C++ standard to GNU20 because SonarCloud's CFamily analyzer rejects the C++26 default of recent IDF / GCC 14 toolchains. Drop the file if you don't ship to SonarCloud.

## Expected output

```
I (...) MAIN: Distance: 1500
I (...) MAIN: Distance: 1503
W (...) MAIN: Status: 1          # AJ_SR04M_DIST_NO_ECHO — target out of range
I (...) MAIN: Distance: 850
```

The status codes are the enum members of `aj_sr04m_dist_status_t` (`OK=0`, `NO_ECHO=1`, `BAD_CHECKSUM=2`, `BAD_FRAME=3`).

## License

MIT — see the component's [`LICENSE`](../../LICENSE).
