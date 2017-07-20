\ burn the final application in flash for self-contained use

<<<core>>>

cr compiletoflash
( main start: ) here hex.

include depth.fs

: 3blinks 3 0 do led-on 300 ms led-off 300 ms loop ;

: init init unattended 3blinks main ;
( main end, ram free: ) here hex. compiletoram flashvar-here here - .
