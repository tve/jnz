\ GPS task
\ assumes that the GPS is connected to usart2 on PA2&PA3

PA11 constant PPS   \ Pulse-Per-Second input from GPS

\ ===== buffers

66 buffer: gpsbuf \ variable-length messages

 0 variable gpslen  \ length of msg in gpsbuf
 0 variable gpslock \ true if gpsbuf is being transmitted

\ ===== varints

: >var.buf ( n addr -- len )  \ add one 32-bit value to addr and return bytes output
  \ shift one position left - if negative, invert all bits (puts sign in bit 0)
  \ this compresses better for *signed* values of small magnitude
  rol dup 1 and 0<> shl xor
  \ output lowest 7 bits
  dup $80 or hold
  \ output higher 7-bit groups
  begin
    7 rshift
  dup while
    dup $7F and hold
  repeat drop ;

\ ===== synchronize internal millis time with GPS pulse-per-second

0 variable p-millis \ value of millis at last pulse

: pps-irq ( -- ) \ handle PPS interrupt
  millis p-millis !          \ fetch current millis and save away as reference
  11 bit EXTI-PR bis!        \ clear interrupt
  ;

: pps-init ( -- ) \ init PPS interrupt
  IMODE-LOW PPS io-mode!
  11 bit                     \ bit for pin
  dup EXTI-IMR bis!          \ unmask interrupt line
      EXTI-RTSR bis!         \ trigger on rising edge
  \ $00 syscfg_exticr3 !     \ select port A (power-on default)
  ['] pps-irq irq-exti4_15 ! \ set interrupt vector
  7 bit NVIC-EN0R bis!       \ enable EXTI4_15 interrupt
  ;

\ ===== init

: init-gps
  uart2-init
  pps-init
  0 gpslen !
  0 gpslock !
  ;

\ ===== GPS task

task: gps-task
: gps-loop& ( -- )
  init-gps
  pps-init
  gps-task activate 1000 ms
  begin
    gps-line ?dup if ( c-addr len )
      2dup rec \ record gps buffer
      begin gpslock c@ while 10 ms repeat \ wait for gps buffer not to be TX'ed
      gpsbuf -rot buffer-cpy ( gpsbuf len ) \ copy into gps buffer
      gpslen c! drop
    then
    depth 0<> if ." GPSstack:" .v then
  again ;
