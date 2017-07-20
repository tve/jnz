\ SLIP encapsulation
\ well, not quite yet, just fragments of it...

$c0 constant SLIP-END
$db constant SLIP-ESC
$dc constant SLIP-ESC-END
$dd constant SLIP-ESC-ESC

: emit>slip ( c -- ) \ emit a character with slip-escape to uart1
  $ff and case
    SLIP-END of SLIP-ESC emit SLIP-ESC-END emit endof
    SLIP-ESC of SLIP-ESC emit SLIP-ESC-ESC emit endof
    dup emit
  endcase ;

: pkt>slip ( fei snr rssi addr len -- ) \ emit (uart1) a packet using slip
  SLIP-END emit
  dup emit>slip \ send length
  0 ?do
    dup i + c@ emit>slip \ send pkt character
  loop
  drop
  emit>slip \ send rssi
  emit>slip \ send snr
  dup emit>slip \ send fei LSB
  8 rshift emit>slip \ send fei MSB
  SLIP-END emit ;

: buf>slip ( addr len -- ) \ emit (uart1) a buffer (255 bytes max) using slip
  SLIP-END emit
  dup emit>slip \ send length
  0 ?do
    dup i + c@ emit>slip \ send pkt character
  loop
  drop
  SLIP-END emit ;
