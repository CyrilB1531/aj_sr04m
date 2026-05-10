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
- Built-in frame parsers (binary 4-byte and ASCII `Gap=XXXX mm`) exposed as part of the public API for offline use or external sniffing.
- Out-of-range and bad-frame detection through dedicated status codes.

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

| Mode | R19 value (typical) | Behavior |
| ---- | ------------------- | -------- |
| 1 | open | HC-SR04 compatible: 10-15 µs trigger pulse, echo pin pulse-width = round-trip time. |
| 2 | 47 kΩ | Same protocol as mode 1 but with a 1100 µs trigger pulse and lower idle power. |
| 3 | 120 kΩ | Module measures continuously every ~100 ms and pushes a 4-byte binary frame on UART. |
| 4 | 360 kΩ | Module sleeps; sends a 4-byte binary frame after receiving a trigger byte (0x01) on UART. |
| 5 | 0 Ω (short) | Module sleeps; sends an ASCII frame `Gap=XXXX mm\r\n` after receiving a trigger byte. Lowest power. |

The mode selected in `menuconfig` **must** match the R19 selection on the physical module — there is no auto-detect.

## Configuration

Run `idf.py menuconfig` and open `AJ-SR04M Configuration`:

- `Sensor operating mode` (1-5) — must match the module's R19.
- `Trigger / UART TX GPIO` — defaults to GPIO 17.
- `Echo / UART RX GPIO` — defaults to GPIO 16.
- `UART port number` (modes 3-5 only) — defaults to UART2. UART0 is normally the console — avoid it.

## Usage

```c
#include "aj_sr04m.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "APP";

void app_main(void) {
    int16_t distance = 0;
    ESP_ERROR_CHECK(aj_sr04m_init());

    while (true) {
        aj_sr04m_trigger();
        switch (aj_sr04m_read_duration(&distance)) {
            case AJ_SR04M_DIST_OK:
                ESP_LOGI(TAG, "Distance: %d mm", distance);
                break;
            case AJ_SR04M_DIST_NO_ECHO:
                ESP_LOGW(TAG, "No echo (too close, too far, or absorbing target)");
                break;
            case AJ_SR04M_DIST_BAD_FRAME:
            case AJ_SR04M_DIST_BAD_CHECKSUM:
                ESP_LOGW(TAG, "Sensor frame rejected");
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
```

## Public API

- `aj_sr04m_init(void)` — initialize GPIO+RMT or UART according to the configured mode.
- `aj_sr04m_trigger(void)` — start a measurement: arm RMT and pulse TRIG (modes 1-2), send a 0x01 byte (modes 4-5), or no-op (mode 3, autonomous).
- `aj_sr04m_read_duration(int16_t* distance)` — return a measurement status; on success, `*distance` holds the distance in millimeters.
- `aj_sr04m_parse_binary_frame(const uint8_t* data, int len, int16_t* distance)` — parse a 4-byte binary frame (modes 3 and 4). Useful when sniffing the UART externally or unit-testing.
- `aj_sr04m_parse_ascii_frame(const char* data, int16_t* distance)` — parse an ASCII frame (mode 5).

## Status codes

| Code | Meaning |
| ---- | ------- |
| `AJ_SR04M_DIST_OK` | Valid measurement, `*distance` holds the value in mm. |
| `AJ_SR04M_DIST_NO_ECHO` | No echo detected: target too close, too far, or absorbing material. |
| `AJ_SR04M_DIST_BAD_CHECKSUM` | Binary frame received but the checksum is invalid (modes 3-4 only). |
| `AJ_SR04M_DIST_BAD_FRAME` | Frame missing, malformed or unrecognized. |

## Sensor range

The driver validates measurements within a **200 mm – 4500 mm** window (`AJ_SR04M_DIST_MIN_VALID_MM` / `AJ_SR04M_DIST_MAX_VALID_MM` in `aj_sr04m.c`). The AJ-SR04M datasheet advertises up to 8 m on some revisions and the JSN-SR04T up to 6 m, but accuracy degrades significantly past 4.5 m on the modules tested — adjust the upper bound if your module proves reliable further out.

Out-of-range conditions are reported as `AJ_SR04M_DIST_NO_ECHO`. Different revisions emit different "no echo" sentinels: some AJ-SR04M units return a value greater than 4500 mm (e.g. `6016`), the JSN-SR04T datasheet specifies plain `0`. Both are caught by the [200, 4500] window check.

## Tests

Unit tests covering both frame parsers live in `test/` and run on the ESP-IDF Unity test runner. They are deterministic and do not require a connected sensor — the parser API can be exercised on synthetic frames.

## License

MIT — see [`LICENSE`](LICENSE).
