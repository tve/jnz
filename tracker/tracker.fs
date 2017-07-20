\ application setup and main loop
\ assumes that the GPS is connected to usart2 on PA2&PA3

\ ===== init

: init-hw
  16MHz 1000 systick-hz
  lptim-init

  432600 $CB rf-init rf!lora250.7 \ LoRa @423.6Mhz, 11kbps
  17 rf-power
  spi-init \ for smem
  ;

\ ===== Radio task

: prep-gps ( -- addr len ) \ prepare GPS data for tx
    $80 gpsbuf cbis! \ set info bit in format byte
    gpsbuf gpslen c@ ( addr len )
    ." GPSbuf " 2dup swap hex. . cr
    ;

: tx+ack ( c-addr len -- ok ) \ tx packet, get ack, return flag whether got ack
  2dup + rf+info 2+ ( c-addr len ) \ add RF info to the end
  ." TX " 2dup buffer. cr
  LED iox!
  $2B rf-send           ( )                  \ send with ack req as node 11
  rf-ack? ?dup if                            \ wait for ack
    led-on ." LoRa ACK " rf>uart ." : " . cr -1
  else
    led-off ." Lost" cr 0
  then
  depth 1 <> if ." TX+ACKstack:" .v then
  ;

task: radio-task
: radio-loop& ( -- )

  radio-task activate 1000 ms

  begin
    gpslen c@ 0<> if \ got a GPS packet to send?
      -1 gpslock c! \ lock the packet so GPS doesn't overwrite it
      prep-gps ( addr len )
      tx+ack if 0 gpslen c! then \ send and clear if ACK-ed
      0 gpslock c! \ unlock packet buffer
    else imunum c@ if \ got an IMU packet to send?
      imubuf imulen 2- ( addr len ) \ imulen includes rf-info
      tx+ack if free-imu-buffer then
    then then
    depth 0<> if ." RADIOstack:" .v then
    50 ms
  again
  ;

\ ===== MAIN

include info.fs

: info init-hw
  pb-init drop pb-count ." There are " . ." records, next @"
  rec-init drop next-addr @ hex. cr
  ;

: main
  info detail multitask
  init-hw rec-init drop
  gps-loop&
  imu-loop&
  radio-loop&
  tasks ;

: just-imu
  info multitask
  init-hw rec-init drop
  imu-loop&
  tasks ;


256 buffer: buf
0 variable chk
: print-buf ( addr len -- )
  ." PKT>" 0 ?do dup i + c@ dup chk c+! h.2 loop drop ;

: play init-hw
  pb-init drop
  begin
    buf pb dup 0 >= while
    cr 0 chk c!
    buf swap print-buf
    space chk c@ h.2 cr
  repeat drop
  ." -END-" cr
  ;


\ main
