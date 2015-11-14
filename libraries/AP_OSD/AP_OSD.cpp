

#include <AP_OSD/AP_OSD.h>

extern const AP_HAL::HAL& hal;

AP_OSD::OSD_params::OSD_params()
 : artifical_horizon_pitch_adjustment{quan::angle_<float>::deg{0.f}}
 ,viewing_distance_px{250}
 ,battery_pos{110,-130,-80}
 ,gps_pos{-160,-130,-80}
 ,control_mode_pos{35,-130,-80}
{}






