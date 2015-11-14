include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : AHRS_Test.bin
	stm32flash -b 115200 -f -v -w AHRS_Test.bin /dev/ttyUSB0
