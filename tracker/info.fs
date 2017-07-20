: sect-info ( flash-addr -- ) \ print info about one flash sector
  begin ( flash-addr )
    dup smem.1> dup $ff <> while \ fetch length, check if end
    dup . ( flash-addr len ) \ print length
    + 1+ ( flash-addr ) \ jump past buffer
    dup $fff and 0= if ."  >0" drop exit then
  repeat
  drop $fff and 4096 swap - ." >" .
  ;

: detail ( -- ) \ print info about flash
  init-hw pb-init drop cr
  size-mask @ 0 do
    i smem.1> $ff <> if
      i hex. ." : " i sect-info cr
    then
  4096 +loop 
  ." play @" next-addr @ hex.
  ." rec @" rec-init drop next-addr @ hex. cr ;

: erase ( -- ) \ erase flash
  init-hw pb-init drop cr
  size-mask @ 0 do
    i smem.1> $ff <> if i dup hex. 8 rshift smem-erase then
  4096 +loop ;

