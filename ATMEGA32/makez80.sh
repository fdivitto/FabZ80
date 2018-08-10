#!/bin/bash

# makes bootcode.h from Z80BOOT.z80 assembler code

set -e

# compile Z80 code
z80asm --cpu=z80 --make-bin --output=Z80BOOT.bin Z80BOOT.z80

# insert compiled in bootcode.h
echo "// Auto generated code from makeboot.sh" > bootcode.h
echo "const uint8_t Z80BOOTCODE[] PROGMEM = {" >> bootcode.h
xxd -i < Z80BOOT.bin >> bootcode.h
echo "};" >> bootcode.h
echo -n "const uint16_t Z80DESTADDR = 0x" >> bootcode.h
pcregrep -i -o1 'org ([0-9a-f]{4}).+' Z80BOOT.z80 | tr -d '\n' >> bootcode.h
echo ";" >> bootcode.h

echo "Size: $(stat -f%z Z80BOOT.bin) bytes"

# remove temporary files
rm Z80BOOT.bin
rm Z80BOOT.o
