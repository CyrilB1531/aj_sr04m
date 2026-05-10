/* SPDX-FileCopyrightText: 2026 Cyril Brunet
 *
 * SPDX-License-Identifier: MIT
 *
 * Linker fragment that collects the per-TU gcov_info pointers emitted by
 * objects compiled with --coverage, and exposes the markers that
 * __gcov_info_to_gcda() in main_test.c iterates over. ESP-IDF's stock
 * linker scripts don't define this section, so without this fragment the
 * link fails with "undefined reference to __gcov_info_start". */
SECTIONS
{
  .dram0.gcov_info :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__gcov_info_start = .);
    KEEP(*(.gcov_info))
    PROVIDE_HIDDEN(__gcov_info_end = .);
  } > dram0_0_seg
}
