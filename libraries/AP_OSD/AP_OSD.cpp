

#include <AP_OSD/AP_OSD.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Osd::var_info[] PROGMEM = {

   AP_GROUPINFO("MAG_PX", 0, AP_Osd, compass_pos[0], 0)
   ,AP_GROUPINFO("MAG_PY", 0, AP_Osd, compass_pos[0], -100)
};




