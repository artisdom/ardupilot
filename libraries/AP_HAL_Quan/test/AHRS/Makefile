include ../../../../mk/apm.mk

.PHONY: upload_sp

upload_sp : AHRS.bin
	stm32flash -b 115200 -f -v -w AHRS.bin /dev/ttyUSB0
