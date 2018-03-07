\ IMU tasks

include ../tlib/tcore.fs
include butter.fs

13 constant FMT:IMU \ packet format for IMU info
PA12 constant RDY   \ LED to show that calibration is done

\ ===== buffers

4 1 + 14 + 2 + 3 + \ format, 24-bit millis, calib, 14 data, 2 info = 21 (+ 3 padding)
   constant imulen \ length of IMU messages
 4 constant imumax \ max number of buffered IMU msgs
imumax imulen *
   buffer: imubuf  \ buffer for IMU messages
 0 variable imunum \ number of IMU messages in buffer

\ ===== IMU buffers

: next-imu-buffer ( -- addr ) \ returns address of IMU buffer to fill
  imunum @ dup imumax u>= if \ all buffers full?
    drop imumax 1- dup imunum ! \ set imunum to imumax-1, and overwrite that buffer
  then ( imunum )
  imulen * imubuf + \ address of buffer to fill
  ;

: free-imu-buffer ( ) \ free first IMU buffer by moving rest up
  -1 imunum +! imunum @ ( imunum ) \ decrement number of buffers in use
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
  1 imunum +! \ one more packet to send...
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

: record-imu-buffer ( addr -- )
  imulen 5 - rec \ imulen includes RF-info and a 3-byte padding, which we're not recording
  ;

\ ===== IMU sampling task

0 variable calib-hook \ points to calib! until calibrated
: calib! ( -- ) \ check whether calibrated and save calib data
  bno-calib? $ff = if
    ." Saving calib " cr
    BNO-EE bno-calib>ee
    ['] nop calib-hook ! \ done forever
  then ;

: calib-restore ( -- ) \ restore calibration and set-up save hook
  [ifdef] not-always-save-calib
    ['] nop
    BNO-EE bno-calib<ee ( 'nop calib-flag )
    if ." Restored fixed calib" cr else drop ['] calib! then
    calib-hook !
  [else]
    ['] calib! calib-hook !
    BNO-EE bno-calib<ee if ." Restored calib" cr then
  [then]
  ;

\ variables to average acceleration x-y-z
0 variable bno.cnt \ number of samples stored
3 cells buffer: bno.accum \ temporary accumulator of 32-bit values

: imu-sample ( -- ) \ add one IMU sample to accumulator
  bno-data \ fetch current data
  bno.data 8 + \ address of accelerometer data
  bno.accum ( d-addr s-addr )
  over hs@ over +!  4 + swap 2+ swap
  over hs@ over +!  4 + swap 2+ swap
  over hs@ over +!  4 + swap 2+ swap
  2drop
  1 bno.cnt +!
  ;

\ The imu-sampler task samples the IMU at its max frequency, which is 100Hz and
\ accumulates the acceleromoter part of the samples. The imu-task then periodically
\ averages the samples, puts them through a butterworth high-pass filter to remove
\ DC components, and saves them for transmission. (The orientations samples are not
\ averaged: the most recent one is taken by imu-task.)
task: imu-sampler
: imu-sloop& ( -- )
  calib-restore
  bno.accum 3 cells 0 fill
  0 bno.cnt !
  imu-sampler activate
  begin
    \ millis .
    imu-sample
    9 ms
  again ;

: imu-average ( -- )
  bno.data 8 + \ dest addr of accelerometer data
  bno.accum bno.cnt @
  3 0 do ( d-addr s-addr cnt )
    over @ over /  ( d-addr s-addr cnt value )
    3 pick h! ( d-addr s-addr cnt )
    rot 2+ rot 4 + rot
  loop 2drop drop
  bno.accum 3 cells 0 fill
  0 bno.cnt ! ;

\ variables to put accelerometer through butterworth filter
3 bw-len * buffer: bno.bw

: imu-data.
  bno.data ." BNO:("
  4 0 do dup h@ 16 lshift 16 arshift . 2+ loop \ print quaternions
  ." ) a("
  3 0 do dup h@ 16 lshift 16 arshift . 2+ loop \ print accel
  drop
  [char] ) emit cr ;

: imu-butter ( -- ) \ run accelerometer data through butterworth filter
  \ imu-data.
  bno.data 8 + \ addr of accelerometer data
  3 0 do ( accel@ )
    0 over hs@ ( accel@[u] accel[f] )
    bno.bw i bw-len * + ( accel@[u] accel[f] state[u] )
    bw-step ( accel@[u] filtered[f] )
    nip over h! \ save filtered value ("integer" part only)
    2+
  loop drop
  \ imu-data.
  ;

\ ===== IMU task

task: imu-task
: imu-loop& ( -- )
  bno-init if ." Cannot find/init BNO055" exit then
  cr bno.info
  imubuf imumax imulen * 0 fill \ init IMU message buffer to zeros
  0 imunum !
  imu-sloop&
  OMODE-PP RDY io-mode!         \ init LED pin
  RDY ios! 100 ms RDY ioc!      \ brief LED flash
  bno.bw 3 bw-len * 0 fill      \ init butterworth filter
  imu-task activate

  begin
    \ fetch data
    millis dup bno-calib? ( millis millis calib )
    bno.cnt @ dup 9 < if . ." IMU samples " cr else drop then
    imu-average
    imu-butter \ apply butterworth filter to data
    next-imu-buffer dup >r fill-imu-buffer ( millis R:addr )
    r@ record-imu-buffer ( millis R:addr )
    r> print-imu-buffer ( millis )
    bno-calib? $ff = RDY io!
    calib-hook @ execute \ save calibration if we haven't and device is calibrated
    millis swap - 100 swap - ( 100-delta-millis )
    dup 0 > if ms else drop pause then \ wait remainder of 100ms
    depth 0<> if ." IMUstack:" .v then
  again ;
