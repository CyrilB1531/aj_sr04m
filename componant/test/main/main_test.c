/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include <stdio.h>

/* Coverage instrumentation can be enabled via -D-DDISABLE_COVERAGE=OFF at the
 * project level. The .gcno files end up in build/, but extracting .gcda from
 * the running program in QEMU is non-trivial:
 *   - apptrace gcov needs OpenOCD over JTAG, which QEMU does not expose;
 *   - a custom UART dump conflicts with esptool's 64 KB segment layout rule;
 *   - semihosting works in principle but requires path remapping setup.
 * For now CI only validates the test results; coverage extraction is a
 * follow-up. */

void app_main(void) {
  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();

  printf("\n=== TESTS_DONE ===\n");
  fflush(stdout);

  /* Idle so QEMU stays alive long enough for CI to capture the full output. */
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
