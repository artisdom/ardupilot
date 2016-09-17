#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Compass.h"

template <> enum Rotation get_compass_orientation<AP_HAL::Tag_BoardType>()
{
   return  ROTATION_NONE;
}

template <> void install_compass_backends<AP_HAL::Tag_BoardType>(Compass& c)
{
    install_compass_backend_hil(c);
}

#endif
