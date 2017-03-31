# top level makefile to build SITL for primary vehicle targets. 
# Useful for static analysis tools

all: sitl

sitl: TARGET=sitl
sitl: plane 

clean: TARGET=clean
clean: plane 

.PHONY: all plane sitl clean

plane:
	$(MAKE) -C ArduPlane $(TARGET)

