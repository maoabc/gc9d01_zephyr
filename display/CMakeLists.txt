# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_GC9D01		display_gc9d01.c)

zephyr_library_include_directories(${ZEPHYR_BASE}/drivers/display)
set_property(TARGET drivers__display PROPERTY ALLOW_EMPTY TRUE)




