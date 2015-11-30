include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : BARO_generic.bin
	stm32flash -b 115200 -f -v -w BARO_generic.bin /dev/ttyUSB0
