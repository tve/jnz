\ application setup and main loop
\ assumes that the MCP9808 temp sensor is on i2c

 60 constant rate     \ seconds between readings
 20 constant rate     \ seconds between readings
  1 variable rate-now \ current rate depending on ACK success
  1 variable missed   \ number of consecutive ACKs missed
120 variable blinks   \ how often to blink LED before going quiet

PA1 constant vBatPin  \ battery voltage divider

15 constant margin \ target SNR margin in dB 8dB demod + log10(2*RxBw) + 5dB margin

: low-power-sleep ( n -- ) \ sleeps for n * 100ms
  rf-sleep highz-gpio
  adc-deinit only-msi
  0 do stop100ms loop
  OMODE-PP LED io-mode!
  OMODE-PP PA0 io-mode! \ for debugging
  hsi-on adc-init spi-init i2c-init ;

LED ios!

1 variable Vcellar \ lowest VCC measured
: vbat vBatPin adc-mv shl ;
: v-cellar vbat Vcellar @ min Vcellar ! ;
: v-cellar-init vbat Vcellar ! ;

: n-flash ( n -- ) \ flash LED n times very briefly (100ms)
  0 ?do
    LED ioc! 1 low-power-sleep
    LED ios! 1 low-power-sleep
  loop ;

: empty-stack ( any -- ) \ checks that the stack is empty, if not flash LED and empty it
  depth ?dup if
    rx-connected? if
      ." *** " . ." left on stack: " .v
      depth
    else
      10 n-flash
    then
    0 do drop loop
  then ;

\ : rx-connected? 0 ;

: c>f ( n -- n ) \ convert celsius to farenheit
  9 * 5 / 32 + ;

: cC>F ( n -- f ) \ convert hundredths of degrees celsius to farenheit fixed-point
  0 swap 0,018 f* 0 32 d+ 
  ;

: show-readings ( vprev vcc tint txpow lux humi pres temp -- ) \ print readings on console
  hwid hex. ." = "
  dup cC>F 4 1 f.n.m ." °F, "
  4 pick . ." dBm, "
  5 pick c>f .n ." °F, "
  6 pick .milli ." => "
  7 pick .milli ." V "
  ;

: rf>uart ( len -- len )  \ print reception parameters
  rf.freq @ .n ." Hz "
  rf.rssi @ 2/ negate .n ." dBm "
  rf.afc @ 16 lshift 16 arshift 61 * .n ." Hz "
  dup .n ." b "
  ;

: <pkt ( -- ) pkt.buf pkt.ptr ! ;  \ start collecting values for the packet
: pkt> ( format -- c-addr len )    \ encode the collected values as RF packet
  \ ." PKT> " pkt.ptr @  begin  4 - dup @ . dup pkt.buf u<= until  drop dup . cr
  <v
    pkt.ptr @  begin  4 - dup @ >var  dup pkt.buf u<= until  drop
    hold
  v> ;

: send-packet ( vprev vcc tint txpow lux humi pres temp -- )
  <pkt  hwid 9 0 do +pkt loop  2 pkt>
  PA0 ios!
  $80 rf-send \ request ack
  v-cellar ;

: rf@power ( -- n ) RF:PA rf@ $1F and ;

: rf-adj-power ( snr )
  rx-connected? if ." SNR " dup .  then
  rf@power swap
  margin - 0 > if 2-  0 max  \ SNR > margin
             else 2+ 31 min  \ SNR <= margin
  then
  rx-connected? if ."  POW " dup . cr then
  rf-power ;

: rf-up-power ( -- ) \ increase power one step due to missed ACK
  rf@power 1+ 31 min
  rx-connected? if ." POW++ " dup . cr then
  rf-power ;

: rf-toggle-power ( -- ) \ toggle max/med power so we don't overrun RX by sticking to max
  rf@power $10 xor $F or
  rx-connected? if ." POW<> " dup . cr then
  rf-power ;

: missed++ missed dup @ 1+ swap ! ;
: rate!normal rate rate-now ! ;           \ set normal rate
: rate!fast rate 3 rshift 1+ rate-now ! ; \ set fast rate (8x)
: rate!slow rate 2 lshift rate-now ! ;    \ set slow rate (0.25x)

: no-ack-repeat ( -- ) \ first missed ACK, quickly repeat
  rf-up-power rate!fast missed++ ;
: no-ack-continue ( -- ) \ missed a few ACKs, raise power
  rf-up-power rate!normal missed++ ;
: no-ack-slow ( -- ) \ missed a pile of ACKs, toggle high/med power and go slow
  rf-toggle-power rate!slow missed++ ;

: do-blink
  blinks @ ?dup if
    1- blinks !
    LED ioc! 100 ms LED ios!     \ brief LED blink
  then ;

: process-ack ( n -- )
    rx-connected? if              \ print info if connected
      ." RF69 " rf>uart
      dup 0 do rf.buf i + c@ h.2 loop cr
    then
    if rf.buf c@ ?dup if          \ fetch SNR
      rf-adj-power                \ adjust power
    then then
    rf-correct                    \ correct frequency
    rate!normal
    0 missed !
    do-blink
    ;

: get-ack ( -- ) \ wait a bit to receive an ACK, adjust rate-now accordingly
  40 rf-ack?
  PA0 ioc!
  v-cellar rf-sleep
  ?dup if process-ack
  else missed @ ?dup if
    8 > if no-ack-slow else no-ack-continue then
  else no-ack-repeat
  then then
  PA0 ios! ;

: init-hw
  2.1MHz 1000 systick-hz
  lptim-init i2c-init adc-init

  OMODE-PP PA0 io-mode! \ for debugging
  IMODE-ADC vBatPin io-mode!

  912500000 rf.freq ! 6 rf.group ! \ 61 rf.nodeid !
  rf-init $0F rf-power \ rf. cr

  mcp9808-init drop

  v-cellar-init
  ;

: iter
  Vcellar @                    ( vprev )
  v-cellar-init
  vbat adc-temp                ( vprev vcc tint )
  rf@power 18 -                ( vprev vcc tint txpow )
  PA0 ios!
  0 0 0 \ lux humi pres
  mcp9808-convert 100 / 1+ low-power-sleep mcp9808-data mcp9808-sleep
  ( vprev vcc tint txpow lux humi pres temp )

  rx-connected? if show-readings cr 1 ms then
  PA0 ioc!
  send-packet
  PA0 ios!
  get-ack
  empty-stack
  v-cellar
  ;

: chk-lipo \ check whether Vcellar shows that battery is dead
  Vcellar @
  3300 < if begin stop10s again then
  ;
  
: lpt \ low power test
  ." ... low power test..." cr
  init-hw LED ios!
  rf-sleep
  begin
    PA0 ios! 10 ms PA0 ioc!
    10 low-power-sleep
  again
  ;

: main
  ." ... starting rftemp..." cr
  init-hw LED ios!
  begin
    iter
    chk-lipo
    rate-now @ 10 * low-power-sleep
  key? until ;
