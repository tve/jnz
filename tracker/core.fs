\ core libraries

<<<board>>>
cr compiletoflash
( core start: ) here dup hex.

\ replace std emit? by one that does not call pause, this allows tasks to print
\ stuff without it getting garbled
USART1 $1C + constant USART1-ISR \ interrupt status register
USART1 $28 + constant USART1-TDR \ transmit data register
: uart1-emit? ( -- f ) 7 bit USART1-ISR bit@ ;
: uart1-emit ( c -- ) begin uart1-emit? until USART1-TDR ! ;
: init init
  ['] uart1-emit? hook-emit? !
  ['] uart1-emit hook-emit !
  ;

\ include ../flib/mecrisp/disassembler-m0.fs

\ include ../flib/mecrisp/multi.fs

include ../flib/stm32l0/uart2.fs
include ../flib/any/ring.fs
include ../flib/stm32l0/uart2-irq.fs

include ../flib/any/buffers.fs
include ../flib/spi/lora1276.fs
include ../flib/any/varint.fs
include ../tlib/numprint.fs

include ../flib/uart/gps.fs
include ../flib/i2c/bno055.fs

\ we re-include spi.fs so we get a copy with a different chip-select
PC14 variable ssel \ redefine for smem
include ../flib/stm32l0/spi.fs
include ../flib/spi/smem.fs
include ../flib/any/smem-rec.fs

( core end, size: ) here dup hex. swap - .
cornerstone <<<core>>>
compiletoram
