#include "unity.h"
#include <stdio.h>

void app_main(void) {
  // Run test groups (we have only 1 in this example)
  UNITY_BEGIN();
  unity_run_all_tests();
  UNITY_END();
}
