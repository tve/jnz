\ BNO055 IMU tests

: init-hw
  bno-init if ." Cannot find/init BNO055" exit then
  cr bno.info 0 page
  ;

