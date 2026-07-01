#!/usr/bin/env python3
# Write the Creality on-board loader header (CRC16 + image length) into a
# firmware binary so it can be flashed through the factory MCU update path.
#
# Copyright (C) 2026  minicx <minicx@disroot.org>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, struct, binascii

# The 6-byte header (uint16 CRC16 + uint32 length) sits at the tail of the
# 32-byte version string that the linker placed at offset 0x200.
HEADER_OFFSET = 0x20C

def main():
    if len(sys.argv) != 2:
        sys.stderr.write("Usage: gd32_board_crc16.py <firmware.bin>\n")
        return 1
    path = sys.argv[1]
    with open(path, 'rb') as f:
        data = bytearray(f.read())
    if len(data) < HEADER_OFFSET + 6:
        sys.stderr.write("firmware image too small for loader header\n")
        return 1
    # The header slot is zero at this point (padding of the version string).
    # CRC16-CCITT (XMODEM) is computed over the whole image with it zeroed.
    data[HEADER_OFFSET:HEADER_OFFSET + 6] = b'\x00' * 6
    crc = binascii.crc_hqx(bytes(data), 0)
    length = len(data)
    struct.pack_into('<HI', data, HEADER_OFFSET, crc, length)
    with open(path, 'wb') as f:
        f.write(data)
    sys.stdout.write("  Loader header: crc16=0x%04x length=0x%x\n"
                     % (crc, length))
    return 0

if __name__ == '__main__':
    sys.exit(main())
