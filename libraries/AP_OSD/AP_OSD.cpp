

#include <AP_OSD/AP_OSD.h>

extern const AP_HAL::HAL& hal;

AP_OSD::OSD_params::OSD_params()
 : artifical_horizon_pitch_adjustment{quan::angle_<float>::deg{0.f}}
{}






