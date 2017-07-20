\ core libraries

<<<board>>>
cr compiletoflash
( core start: ) here dup hex.

\ include ../flib/mecrisp/disassembler-m0.fs

include ../flib/stm32l0/eeprom.fs

include ../flib/i2c/bno055.fs
include ../tlib/numprint.fs

( core end, size: ) here dup hex. swap - .
cornerstone <<<core>>>
compiletoram
