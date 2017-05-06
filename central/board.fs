\ board definitions

\ eraseflash
compiletoflash
( board start: ) here dup hex.

: -jtag ( -- )  \ disable JTAG on PB3 PB4 PA15
  25 bit AFIO-MAPR bis! ;

: list ( -- )  \ list all words in dictionary, short form
  cr dictionarystart begin
      dup 6 + ctype space
        dictionarynext until drop ;

include ../flib/mecrisp/cond.fs
include ../flib/mecrisp/hexdump.fs
include ../flib/stm32f1/clock.fs
include ../flib/stm32f1/io.fs
include ../flib/pkg/pins36.fs
include ../flib/any/i2c-bb.fs
include ../flib/stm32f1/spi.fs

PA1 constant LED

\ pin connections to the RFM69
PB0 constant RF.DIO0
PB1 constant RF.DIO1
PA8 constant RF.DIO2
PB3 constant RF.DIO3
PB5 constant RF.DIO5
PB4 constant RF.RST
PA4 constant RF.SEL

PA15 constant SMEM.SEL  \ SPI flash memory

1 constant OLED.LARGE  \ display size: 0 = 128x32, 1 = 128x64 (default)

: hello ( -- ) flash-kb . ." KB <tve-central> " hwid hex.
  $10000 compiletoflash here -  flashvar-here compiletoram here -
  ." ram: " . ." free " ." flash: " . ." free " ;

: init ( -- )  \ board initialisation
  init  \ this is essential to start up USB comms!
  -jtag  \ disable JTAG, we only need SWD

  OMODE-PP LED      io-mode!  LED      ios!
  OMODE-PP RF.SEL   io-mode!  RF.SEL   ios!
  OMODE-PP SMEM.SEL io-mode!  SMEM.SEL ios!

  1000 systick-hz

  hello ." ok." cr
;

( board end, size: ) here dup hex. swap - .
cornerstone <<<board>>>
compiletoram
