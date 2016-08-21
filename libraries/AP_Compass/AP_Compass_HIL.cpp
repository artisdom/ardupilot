/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_HIL.cpp - HIL backend for AP_Compass
 *
 */


#include <AP_HAL/AP_HAL.h>
#include "AP_Compass_HIL.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_HIL::AP_Compass_HIL(Compass &compass, uint8_t idx):
    AP_Compass_Backend(compass,"HIL::Compass", idx, false)
{
    //memset(_compass_instance, 0, sizeof(_compass_instance));
    _compass._setup_earth_field();
}

// detect the sensor
AP_Compass_Backend *AP_Compass_HIL::detect(Compass &compass, uint8_t idx)
{
    AP_Compass_HIL *sensor = new AP_Compass_HIL(compass, idx);
    if (sensor){
       if (!sensor->init()) {
           delete sensor;
           sensor = nullptr;
       }
    }
    return sensor;
}

bool
AP_Compass_HIL::init(void)
{
   return install();
}

bool install_compass_backend_hil(Compass & c)
{
   bool result = true;
   for ( uint8_t i = 0; i < HIL_NUM_COMPASSES; ++i){
      result &= (AP_Compass_HIL::detect(c, i) != nullptr);
   }
   return result;
}

void AP_Compass_HIL::read()
{
   uint8_t const index = get_index();
   if (_compass._hil.healthy[index]) {
      Vector3f field = _compass._hil.field[index];
      rotate_field(field);
      publish_raw_field(field, AP_HAL::micros());
      correct_field(field);
      uint32_t saved_last_update = _compass.last_update_usec(index);
      publish_filtered_field(field);
      set_last_update_usec(saved_last_update);
   }
}
