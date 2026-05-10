#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* GCC gcov runtime exposes the linker-provided info table boundaries and the
 * dump function below. With -fprofile-arcs -ftest-coverage these are populated
 * automatically at link time. Requires GCC >= 10. */
struct gcov_info;
extern const struct gcov_info *const __gcov_info_start[];
extern const struct gcov_info *const __gcov_info_end[];
extern void __gcov_info_to_gcda(const struct gcov_info *info,
                                void (*filename_fn)(const char *, void *),
                                void (*dump_fn)(const void *, unsigned, void *),
                                void *(*allocate_fn)(unsigned, void *),
                                void *arg);

static void gcov_filename(const char *fn, void *arg) {
  (void)arg;
  printf("\n--GCDA-START %s--\n", fn);
}

static void gcov_dump(const void *data, unsigned len, void *arg) {
  (void)arg;
  const uint8_t *bytes = (const uint8_t *)data;
  for (unsigned i = 0; i < len; i++) {
    printf("%02x", bytes[i]);
  }
  putchar('\n');
}

static void *gcov_alloc(unsigned size, void *arg) {
  (void)arg;
  return malloc(size);
}

/* Walk all instrumented translation units and dump their .gcda payload as hex,
 * framed by GCDA-START/GCDA-END markers, on the default UART. CI parses these
 * markers and reconstructs the .gcda files alongside the matching .gcno. */
static void dump_all_gcov(void) {
  fflush(stdout);
  for (const struct gcov_info *const *info = __gcov_info_start;
       info < __gcov_info_end; info++) {
    __gcov_info_to_gcda(*info, gcov_filename, gcov_dump, gcov_alloc, NULL);
    printf("\n--GCDA-END--\n");
  }
  fflush(stdout);
}

void app_main(void) {
  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();

  dump_all_gcov();
  printf("\n=== TESTS_AND_GCOV_DONE ===\n");
  fflush(stdout);

  /* Idle so QEMU stays alive long enough for CI to capture the full output. */
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
