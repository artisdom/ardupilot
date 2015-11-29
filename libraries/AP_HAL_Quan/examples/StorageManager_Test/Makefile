#include ../../../../../mk/apm.mk
include /home/andy/cpp/projects/apm_quantracker/ardupilot/mk/apm.mk

 .PHONY: upload_sp
upload_sp : StorageTest.bin
	stm32flash -b 115200 -f -v -w StorageTest.bin /dev/ttyUSB0
