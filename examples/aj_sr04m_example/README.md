<!--
SPDX-FileCopyrightText: 2026 Cyril Brunet

SPDX-License-Identifier: MIT
-->

# aj_sr04m — get-started example

Minimal ESP-IDF project that exercises the [`aj_sr04m`](../../) driver: initialize the driver, trigger a measurement on every configured sensor every couple of seconds, and log each distance (or the error status) to the console.

## What it does

1. `aj_sr04m_init()` creates every sensor described in `menuconfig` and configures its GPIO + RMT capture (modes 1 and 2) or its UART (modes 3, 4 and 5), depending on the `AJ_SR04M_MODE` selected there. `aj_sr04m_get_sensor_count()` then reports how many instances were created.
2. In a loop: `aj_sr04m_trigger_all()` starts a measurement on every sensor, a 100 ms delay leaves the modules time to answer, `aj_sr04m_read_all()` fills the caller-provided `distances` and `statuses` arrays (sized `AJ_SR04M_MAX_SENSORS`) and reports how many sensors were read, then `ESP_LOGI` / `ESP_LOGW` print one line per sensor with either the distance in mm or the failure status.
3. The loop then sleeps 1 s in mode 3 (the module emits frames autonomously) and 2 s in every other mode.

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
- Maximum number of simultaneous sensor instances (default 1) — raising it reveals a `Sensor Configuration` submenu with the pins and UART port of sensors 2 to 4.
- Trigger / UART TX GPIO (default 17, sensor 1).
- Echo / UART RX GPIO (default 16, sensor 1).
- UART port number (modes 3-5 only, default 2 on ESP32 / ESP32-S3, sensor 1).
- Delay between consecutive triggers (multi-sensor only, default 150 ms) — spaces the triggers issued by `aj_sr04m_trigger_all()` so that co-located modules do not hear each other's burst.

The bundled `sdkconfig.defaults` pins the C++ standard to GNU20 because SonarCloud's CFamily analyzer rejects the C++26 default of recent IDF / GCC 14 toolchains. Drop the file if you don't ship to SonarCloud.

## Expected output

```
I (...) MAIN: Driver initialized, 2 sensor(s) configured
I (...) MAIN: Sensor 0: 1500 mm
I (...) MAIN: Sensor 1: 850 mm
I (...) MAIN: Sensor 0: 1503 mm
W (...) MAIN: Sensor 1: status 1     # AJ_SR04M_DIST_NO_ECHO — target out of range
```

The status codes are the enum members of `aj_sr04m_dist_status_t` (`OK=0`, `NO_ECHO=1`, `BAD_CHECKSUM=2`, `BAD_FRAME=3`).

## License

MIT — see the component's [`LICENSE`](../../LICENSE).
