/*
 * SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "unity.h"
#include <stdio.h>
#include <stdlib.h>

/* Coverage instrumentation can be enabled via -D-DDISABLE_COVERAGE=OFF at the
 * project level. On the ESP32/QEMU target the .gcda data is reconstructed
 * from a UART dump; on the linux target gcov writes .gcda files directly
 * to the build tree on process exit (so the linux app_main must `exit()`
 * rather than spin, otherwise the SIGTERM that kills it never runs the
 * gcov destructors). */

void app_main(void) {
  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();

  printf("\n=== TESTS_DONE ===\n");
  fflush(stdout);

#if CONFIG_IDF_TARGET_LINUX
  /* Clean exit so the gcov runtime flushes .gcda files. */
  exit(0);
#else
  /* Idle so QEMU stays alive long enough for CI to capture the full output. */
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
#endif
}
