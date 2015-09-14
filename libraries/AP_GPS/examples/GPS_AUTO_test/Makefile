include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : GPS_AUTO_test.bin
	stm32flash -b 115200 -f -v -w GPS_AUTO_test.bin /dev/ttyUSB0
