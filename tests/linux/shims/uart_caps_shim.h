/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Shim for ESP-IDF linux target test compilation.
 *
 * ESP-IDF's `components/soc/linux/include/soc/soc_caps.h` deliberately
 * omits UART caps because the canonical host-test path uses CMock-
 * generated UART mocks, not the real `driver/uart.h` headers. We instead
 * --wrap the UART entry points, which requires the real headers to compile.
 * This shim is force-included (via `idf_build_set_property(COMPILE_OPTIONS
 * "-include;<this file>")`) before any TU is built, giving us enough soc
 * caps and a stand-in for `soc_periph_uart_clk_src_legacy_t` to satisfy
 * `hal/uart_types.h` and `driver/uart.h`.
 *
 * Values are dummy: nothing on the linux target reads them at runtime — the
 * tests intercept the UART calls before they hit any HAL code.
 */
#pragma once

#ifdef __linux__

#ifndef SOC_UART_NUM
#define SOC_UART_NUM 3
#endif
#ifndef SOC_UART_HP_NUM
#define SOC_UART_HP_NUM 3
#endif
#ifndef SOC_UART_LP_NUM
#define SOC_UART_LP_NUM 0
#endif
#ifndef SOC_UART_FIFO_LEN
#define SOC_UART_FIFO_LEN 128
#endif
#ifndef SOC_UART_BITRATE_MAX
#define SOC_UART_BITRATE_MAX 5000000
#endif
#ifndef SOC_UART_SUPPORT_WAKEUP_INT
#define SOC_UART_SUPPORT_WAKEUP_INT 0
#endif

#ifndef SOC_PERIPH_UART_CLK_SRC_LEGACY_T_DEFINED
#define SOC_PERIPH_UART_CLK_SRC_LEGACY_T_DEFINED 1
typedef enum {
  UART_SCLK_DEFAULT = 0,
  UART_SCLK_APB = 1,
  UART_SCLK_XTAL = 2,
  UART_SCLK_RTC = 3,
} soc_periph_uart_clk_src_legacy_t;
#endif

/* RMT clock source default. On real targets this is per-chip enum in
 * soc/<chip>/clk_tree_defs.h (e.g. SOC_MOD_CLK_APB on esp32s3); on linux
 * the soc port omits it and `hal/rmt_types.h` falls back to
 * `typedef int rmt_clock_source_t`. The value here is irrelevant —
 * mocks.c intercepts every RMT call so the clock source is never used. */
#ifndef RMT_CLK_SRC_DEFAULT
#define RMT_CLK_SRC_DEFAULT 0
#endif

#endif /* __linux__ */
