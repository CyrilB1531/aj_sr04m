<!--
SPDX-FileCopyrightText: 2026 Cyril Brunet

SPDX-License-Identifier: MIT
-->

# aj_sr04m — ESP-IDF driver for the AJ-SR04M ultrasonic sensor

ESP-IDF v5.4+ component for the AJ-SR04M waterproof ultrasonic distance sensor. All five operating modes selected by the R19 resistor on the module are supported.

> **Compatibility with JSN-SR04T**: only **mode 1** (HC-SR04 trigger/echo) is fully compatible as-is. The JSN-SR04T uses **R27** (not R19) to select modes, only documents **3 modes** in its datasheet, and uses **0x55** as the UART trigger byte (not 0x01). For UART modes on a JSN-SR04T, the trigger byte and frame layout in this driver need adjustments.

## Features

- All five operating modes: GPIO trigger/echo via RMT capture (modes 1 and 2), UART autonomous (mode 3), UART low-power binary (mode 4), UART low-power ASCII (mode 5).
- Mode 1/2 echo capture uses the **RMT peripheral** for hardware-timed pulse measurement — no GPIO ISR, no `esp_timer` polling.
- Mode, GPIO pins and UART port configurable at compile time via `idf.py menuconfig`.
- **Multiple sensors** on one board: up to four instances configured from Kconfig, each with its own pins and peripheral resources, driven together through `aj_sr04m_trigger_all()` / `aj_sr04m_read_all()`.
- Built-in frame parsers (binary 4-byte and ASCII `Gap=XXXX mm`) exposed as part of the public API for offline use or external sniffing.
- Out-of-range and bad-frame detection through dedicated status codes.
- **Software UART backend** for UART modes 3-5, so the number of UART sensors is no longer capped by the hardware UART controller count.

## Software UART backend (more sensors than hardware UARTs)

Classic ESP32 exposes only 3 hardware UART controllers (UART0 is usually the console), which caps the number of hardware-backed UART-mode sensors. To drive more, each modes 3-5 sensor can run on a software UART instead: the trigger byte is bit-banged on the TX GPIO and the reply frame is captured with an RMT RX channel on the RX GPIO, then decoded as 9600 8N1 and fed to the same binary/ASCII frame parsers.

The backend is chosen per sensor from its configured UART port number: a value in `[0, SOC_UART_NUM)` (0-2 on ESP32) uses that hardware controller, while any larger value selects the software backend. Up to about 8 software sensors fit on classic ESP32 (one RMT RX channel each), though only four can be described in Kconfig — beyond that, create the extra instances with `aj_sr04m_new()`. Transmitting the trigger byte holds a short critical section (~1 ms at 9600 baud), so account for that if Wi-Fi or other latency-sensitive work runs concurrently.

## Hardware

| Module pin | ESP32 default GPIO | Notes |
| ---------- | ------------------ | ----- |
| `5V` / `VCC` | external 3.3 V or 5 V supply | The AJ-SR04M and JSN-SR04T-2.0/3.0 work on **either 3.3 V or 5 V** — see the section below for level-shifting if you use 5 V. |
| `GND` | `GND` | |
| `TRIG` (modes 1, 2) or `RX` (modes 3, 4, 5) | GPIO 17 | ESP32 TX line in UART modes. Configurable. |
| `ECHO` (modes 1, 2) or `TX` (modes 3, 4, 5) | GPIO 16 | ESP32 RX line in UART modes. Configurable. |

### TX/RX voltage level

If the AJ-SR04M is used with a 5 V power supply, the TX/RX of the AJ-SR04M is at 5 V level — check the IO datasheet of your variant to confirm it accepts this voltage before connecting. If not, use a voltage divider on the TX module pin branch (20 kΩ / 10 kΩ) to bring the level down to ~3.3 V.

```drawing
                     AJ-SR04M TX (5 V)
                            │
                        [ 10 kΩ ]
                            │
                            ├─────── ESP32 RX (≈ 3.3 V)
                            │
                        [ 20 kΩ ]
                            │
                           GND
```

Output voltage = 5 V × 20 kΩ / (10 kΩ + 20 kΩ) ≈ 3.33 V — within the ESP32 input range. The TRIG / ESP32-TX direction does not need a divider, since 3.3 V is already above the AJ-SR04M's logic-high threshold.

If you supply the module from 3.3 V, no divider is needed on either line.

## Mode selection (R19 resistor on the module)

| Mode | R19 value | Behavior |
| ---- | --------- | -------- |
| 1 | open | HC-SR04 compatible: 10-15 µs trigger pulse, echo pin pulse-width = round-trip time. |
| 2 | 300 kΩ | Same protocol as mode 1 but with a 1100 µs trigger pulse and lower idle power. |
| 3 | 120 kΩ | Module measures continuously every ~120 ms and pushes a 4-byte binary frame on UART. |
| 4 | 47 kΩ | Module sleeps; sends a 4-byte binary frame after receiving a trigger byte (0x01) on UART. |
| 5 | 0 Ω (short) | Module sleeps; sends an ASCII frame `Gap=XXXX mm\r\n` after receiving a trigger byte. Lowest power. |

Values are those of the AJ-SR04M user manual (mode table and startup flow chart). Use the exact value: the module reads R19 with an ADC, and an undocumented value such as 470 kΩ lands between two thresholds and silently falls back to mode 1.

The mode selected in `menuconfig` **must** match the R19 selection on the physical module — there is no auto-detect.

## Configuration

Run `idf.py menuconfig` and open `AJ-SR04M Configuration`:

- `Sensor variant` — AJ-SR04M (trigger byte 0x01) or JSN-SR04T (0x55).
- `Sensor operating mode` (1-5) — must match the module's R19. The mode is global: **every sensor must be wired to a module set to the same mode.**
- `Maximum number of simultaneous sensor instances` — sensors beyond the first only appear in the menu once this is raised. The menu accepts 1 to 8, but only sensors 1 to 4 have configuration entries; a higher value just allocates unused pool slots.
- `Trigger / UART TX GPIO` and `Echo / UART RX GPIO` — sensor 1, defaults to GPIO 17 and 16.
- `UART port number` (modes 3-5 only) — sensor 1, defaults to UART2. UART0 is normally the console — avoid it.
- `Delay between consecutive triggers` (multi-sensor only) — milliseconds inserted between two sensors by `aj_sr04m_trigger_all()`, 150 ms by default. See [Acoustic crosstalk](#acoustic-crosstalk).
- A `Sensor Configuration` submenu exposes the same three settings for sensors 2 to 4.

Default pins per sensor:

| Sensor | Trigger / TX | Echo / RX | UART port (modes 3-5) |
| ------ | ------------ | --------- | --------------------- |
| 1 | GPIO 17 | GPIO 16 | 2 (hardware) |
| 2 | GPIO 19 | GPIO 18 | 1 (hardware) |
| 3 | GPIO 21 | GPIO 20 | 3 (software) |
| 4 | GPIO 23 | GPIO 22 | 4 (software) |

`aj_sr04m_init()` reads this configuration and creates every sensor instance itself — the application does not call `aj_sr04m_new()` unless it needs pins that are not known at build time.

## Usage

```c
#include "aj_sr04m.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "APP";

void app_main(void) {
    ESP_ERROR_CHECK(aj_sr04m_init());

    int16_t distances[CONFIG_AJ_SR04M_MAX_SENSORS];
    aj_sr04m_dist_status_t statuses[CONFIG_AJ_SR04M_MAX_SENSORS];
    int count = 0;

    while (true) {
        ESP_ERROR_CHECK(aj_sr04m_trigger_all());
        vTaskDelay(pdMS_TO_TICKS(100)); /* let the echo / frame arrive */

        ESP_ERROR_CHECK(aj_sr04m_read_all(distances, statuses,
                                          CONFIG_AJ_SR04M_MAX_SENSORS, &count));

        for (int i = 0; i < count; i++) {
            if (statuses[i] == AJ_SR04M_DIST_OK) {
                ESP_LOGI(TAG, "Sensor %d: %d mm", i, distances[i]);
            } else {
                ESP_LOGW(TAG, "Sensor %d: status %d", i, statuses[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

## Public API

Driver lifecycle and bulk access — the usual entry points:

- `aj_sr04m_init(void)` — create every sensor described by Kconfig and initialize its GPIO+RMT or UART resources.
- `aj_sr04m_deinit(void)` — delete all instances and return to the pre-initialized state.
- `aj_sr04m_get_sensor_count(void)` — number of instances actually created.
- `aj_sr04m_trigger_all(void)` — trigger every sensor.
- `aj_sr04m_read_all(int16_t *distances, aj_sr04m_dist_status_t *statuses, int max_sensors, int *out_count)` — read every sensor into caller-provided arrays.

Per-instance control, for pins decided at runtime:

- `aj_sr04m_new(int trigger_pin, int echo_pin, uint8_t trigger_byte, int uart_num)` — create one instance; returns `NULL` if resources are exhausted. `uart_num` in `[0, SOC_UART_NUM)` picks a hardware UART, any other value selects the software backend.
- `aj_sr04m_delete(aj_sr04m_handle_t handle)` — release that instance.
- `aj_sr04m_trigger(aj_sr04m_handle_t handle)` — arm RMT and pulse TRIG (modes 1-2), arm RMT on a software UART port then send the trigger byte (modes 4-5), or arm RMT alone (mode 3, autonomous — a no-op on a hardware UART port). Returns `ESP_OK`, or an error when the capture could not be armed, in which case nothing was triggered.
- `aj_sr04m_read_distance(aj_sr04m_handle_t handle, int16_t *distance)` — return a measurement status; on success `*distance` holds the value in millimeters.

Frame parsers, usable without hardware:

- `aj_sr04m_parse_binary_frame(const uint8_t *data, int len, int16_t *distance)` — parse a 4-byte binary frame (modes 3 and 4). Useful when sniffing the UART externally or unit-testing.
- `aj_sr04m_parse_ascii_frame(const char *data, int16_t *distance)` — parse an ASCII frame (mode 5).

## Multiple sensors

Each instance owns its pins, buffers and peripheral resources, so sensors run independently. What limits their number is the peripheral they consume:

| Mode | Resource per sensor | Practical limit on classic ESP32 |
| ---- | ------------------- | -------------------------------- |
| 1-2 | one RMT RX channel | 4-8, depending on the chip |
| 3-5, hardware UART | one UART controller | 2 (ports 1 and 2; port 0 is the console) |
| 3-5, software UART | one RMT RX channel | as many as there are free RMT channels |

All sensors share the mode selected in menuconfig. Mixing a module wired for mode 1 with one wired for a UART mode does not work: whichever mode is compiled in, the other module reports `AJ_SR04M_DIST_NO_ECHO` or `AJ_SR04M_DIST_BAD_FRAME` on every read. Check R19 on **every** module before wiring more than one.

### Acoustic crosstalk

Modules mounted side by side hear each other's 40 kHz burst. Fired at the same instant, the neighbour's burst competes with the real echo and the affected module answers with its out-of-range sentinel — surfacing as `AJ_SR04M_DIST_NO_ECHO` on a third of the readings or more, usually on one sensor while the other stays clean.

`aj_sr04m_trigger_all()` therefore spaces its triggers by `Delay between consecutive triggers` (`CONFIG_AJ_SR04M_TRIGGER_STAGGER_MS`, 150 ms by default). The cost is that the call blocks for that delay times the number of sensors minus one, so a four-sensor board spends 450 ms in `aj_sr04m_trigger_all()`. Lower it toward 0 only when the modules face away from each other or are acoustically isolated.

## Status codes

| Code | Meaning |
| ---- | ------- |
| `AJ_SR04M_DIST_OK` | Valid measurement, `*distance` holds the value in mm. |
| `AJ_SR04M_DIST_NO_ECHO` | No echo detected: target too close, too far, or absorbing material. |
| `AJ_SR04M_DIST_BAD_CHECKSUM` | Binary frame received but the checksum is invalid (modes 3-4 only). |
| `AJ_SR04M_DIST_BAD_FRAME` | Frame missing, malformed or unrecognized. |

## Sensor range

The driver validates measurements within a **200 mm – 4500 mm** window (`AJ_SR04M_DIST_MIN_VALID_MM` / `AJ_SR04M_DIST_MAX_VALID_MM` in `src/aj_sr04m.c`). The AJ-SR04M datasheet advertises up to 8 m on some revisions and the JSN-SR04T up to 6 m, but accuracy degrades significantly past 4.5 m on the modules tested — adjust the upper bound if your module proves reliable further out.

Out-of-range conditions are reported as `AJ_SR04M_DIST_NO_ECHO`. Different revisions emit different "no echo" sentinels: some AJ-SR04M units return a value greater than 4500 mm (e.g. `6016`), the JSN-SR04T datasheet specifies plain `0`. Both are caught by the [200, 4500] window check.

## Tests

Test cases live in `tests/cases/` and run on the ESP-IDF Unity runner. They are deterministic and need no connected sensor: the peripherals are replaced by mocks (`tests/cases/mocks.c`) and by the GPIO/RMT shims under `tests/linux/shims/`.

| File | Covers |
| ---- | ------ |
| `test_parser.c` | binary and ASCII frame parsers, including bad checksum, malformed frame, out-of-range distance and leading garbage |
| `test_mode1_2_gpio_rmt.c` | GPIO trigger and RMT echo capture, modes 1 and 2 |
| `test_mode3_autonomous.c` | autonomous UART streaming, mode 3 |
| `test_mode4_uart_binary.c` | triggered binary frame, mode 4 |
| `test_mode5_uart_ascii.c` | triggered ASCII frame, mode 5 |
| `test_sw_uart.c` | software UART bit-banging and 9600 8N1 decoding |

Two runners share those cases. `tests/linux/` builds for the `linux` target and is the fastest loop; `tests/qemu/` runs the same cases on emulated ESP32. Since the operating mode is a compile-time choice, each mode is a separate build selected through `SDKCONFIG_DEFAULTS`:

```bash
cd tests/linux
SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.mode4" \
  idf.py --preview set-target linux
idf.py build && ./build/*.elf
```

Presets exist for `mode1` to `mode5` plus `jsn_sr04t`, which appends the JSN-SR04T variant on top of a mode. CI builds every combination on IDF v5.4.3 and v6.0.1.

## License

MIT — see [`LICENSE`](LICENSE).
