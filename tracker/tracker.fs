\ application setup and main loop
\ assumes that the GPS is connected to usart2 on PA2&PA3

13 constant FMT:IMU \ packet format for IMU info

\ replace std emit? by one that does not call pause
USART1 $1C + constant USART1-ISR \ interrupt status register
USART1 $28 + constant USART1-TDR \ transmit data register
: uart1-emit? ( -- f ) 7 bit USART1-ISR bit@ ;
: uart1-emit ( c -- ) begin uart1-emit? until USART1-TDR ! ;

\ buffers
66 buffer: gpsbuf \ variable-length messages
4 1 + 14 + 2 + 3 + \ format, 24-bit millis, calib, 14 data, 2 info = 21 (+ 3 padding)
   constant imulen \ length of IMU messages
 4 constant imumax \ max number of buffered IMU msgs
imumax imulen *
   buffer: imubuf \ buffer for IMU messages

 \ byte-size variables
 0 variable gpslen              \ length of msg in gpsbuf
 gpslen 1+ dup constant gpslock \ true if gpsbuf is being transmitted
        1+ dup constant imunum  \ number of IMU messages in buffer
        1+ drop                 \ free slot

: init-hw
  16MHz 1000 systick-hz
  uart2-init
  lptim-init
  ['] uart1-emit? hook-emit? !
  ['] uart1-emit hook-emit !

  imubuf imumax imulen * 0 fill \ init IMU message buffer to zero's
  0 gpslen ! \ init gpslen, gpslock, imunum
  ;

task: gps-task
: gps-loop ( -- )
  gps-task activate 1000 ms
  begin
    gps-line ?dup if ( c-addr len )
      begin gpslock c@ while 10 ms repeat \ wait for gps buffer not to be TX'ed
      gpsbuf -rot buffer-cpy ( gpsbuf len ) \ copy into gps buffer
      gpslen c! drop
    then
    depth 0<> if ." GPSstack:" .v then
  again ;

0 variable calib-hook \ points to calib! until calibrated
: calib! ( -- ) \ check whether calibrated and save calib data
  bno-calib? $ff = if
    ." Saving calib" cr
    BNO-EE bno-calib>ee
    ' nop calib-hook ! \ done forever
  then ;

: next-imu-buffer ( -- addr ) \ returns address of IMU buffer to fill
  imunum c@ dup imumax u>= if \ all buffers full?
    drop imumax 1- dup imunum c! \ set imunum to imumax-1, and overwrite that buffer
  then ( imunum )
  imulen * imubuf + \ address of buffer to fill
  ;

: free-imu-buffer ( ) \ free first IMU buffer by moving rest up
  -1 imunum c+! imunum c@ ( imunum ) \ decrement number of buffers in use
  ?dup if \ at least one still in use, need to move
    imubuf dup imulen + swap rot ( imubuf+len imubuf imunum )
    imulen * move ( ) \ move bytes
  then ;

: fill-imu-buffer ( millis calib addr -- ) \ fill IMU buffer at addr, gets data from bno.data
  rot 8 lshift ( calib addr millis<<8 ) \ send only 24 low bits of millis and make space for fmt
  FMT:IMU $80 or or ( calib addr millis+format )
  over ! 4 + ( calib addr ) \ push format and millis into buffer
  bno.data over bno#data move ( calib addr ) \ push IMU data bytes
  bno#data + c! ( ) \ push calib byte
  1 imunum c+! \ one more packet to send...
  ;

: print-imu-buffer ( addr -- )
  ." IMUbuf "
  dup hex. ." : "
  dup c@ h.2 space \ format flag
  dup @ 8 rshift .milli space \ millis
  4 + dup 14 + c@ h.2 ."  : " \ calib flag
  7 0 do dup i shl + h@ 16 lshift 16 arshift . loop drop \ imu data
  cr
  ;

task: imu-task
: imu-loop ( -- )
  bno-init if ." Cannot find/init BNO055" exit then
  cr bno.info

  ['] nop
  BNO-EE bno-calib<ee ( 'nop calib-flag )
  if ." Restored calib" cr else drop ['] calib! then
  calib-hook !

  imu-task activate 1000 ms

  begin
    calib-hook @ execute \ save calibration if we haven't and device is calibrated
    \ fetch data
    millis bno-calib? bno-data 
    next-imu-buffer dup >r fill-imu-buffer
    r> print-imu-buffer
    depth 0<> if ." IMUstack:" .v then
    100 ms
  again ;

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
: radio-loop ( -- )
  432600 $CB rf-init rf!lora250.7 \ LoRa @423.6Mhz, 11kbps
  17 rf-power

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

: main init-hw multitask
  gps-loop
  imu-loop
  radio-loop tasks ;

\ main
