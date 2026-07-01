// Creality on-board loader firmware version string
//
// Copyright (C) 2026  minicx <minicx@disroot.org>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_BOARD_INFO_CONFIGURE

#if CONFIG_BOARD_INFO_CONFIGURE
// The linker places this section at a fixed offset (0x200) after the vector
// table.  Creality's factory MCU updater reads the version string here; a
// CRC16 and image length are written into the tail of this field after the
// build (see scripts/gd32_board_crc16.py), so the string must be <= 12 bytes.
static const char software_version[32]
    __attribute__((section("SV_SECTION"))) __attribute__((used))
    = BOARD_FW_VERSION;
#endif
