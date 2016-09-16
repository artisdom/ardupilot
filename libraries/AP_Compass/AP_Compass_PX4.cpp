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
 *       AP_Compass_PX4.cpp - Arduino Library for PX4 magnetometer
 *
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_Compass_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_device.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;


// Public Methods //////////////////////////////////////////////////////////////

uint8_t AP_Compass_PX4::_num_sensors = 0U;

namespace {

   constexpr const char* px4_device_paths [] ={
      MAG_BASE_DEVICE_PATH"0",
      MAG_BASE_DEVICE_PATH"1",
      MAG_BASE_DEVICE_PATH"2"
   };
}

AP_Compass_PX4::AP_Compass_PX4(Compass &compass, uint8_t index):
    AP_Compass_Backend(compass,px4_device_paths[index],index,false)
  , _mag_fd{-1},_sum{0.f,0.f,0.f},_count{0},_last_timestamp{0ULL}
{
}

template <> enum Rotation get_compass_orientation<AP_HAL::Tag_BoardType>()
{
   return  ROTATION_NONE;
}

template <> void install_compass_backends<AP_HAL::Tag_BoardType>(Compass& c)
{
   AP_Compass_PX4 *sensors [3] = {nullptr,nullptr,nullptr};
   // construct sensors
   for ( int i = 0; i < 3; ++i){
       sensors[i] = new AP_Compass_PX4(c,static_cast<uint8_t>(i));
       if (sensors[i]== nullptr){
         while (i){
            delete sensors [i-1];
            --i;
         }
         return;
       }
   }
   for ( uint8_t i = 0; i < 3; ++i){
     if (! sensors[i]->init() ){
           hal.console->printf("Warning: Compass init failed %s\n",px4_device_paths[sensors[i]->get_index()]);
     }
   }
}

bool AP_Compass_PX4::init(void)
{
   _mag_fd = open(px4_device_paths[get_index()], O_RDONLY);
   if (_mag_fd >= 0) {
      hal.console->printf("opened %s\n",px4_device_paths[get_index()]);
      ++_num_sensors;
   }else{
      hal.console->printf("Unable to open %s\n",px4_device_paths[get_index()]);
      return false;
   }

   // average over up to 20 samples
   if (ioctl(_mag_fd, SENSORIOCSQUEUEDEPTH, 20) != 0) {
      hal.console->printf("Failed to setup compass queue\n");
      return false;                
   }
   set_external( ioctl(_mag_fd, MAGIOCGEXTERNAL, 0) > 0);
   set_dev_id(ioctl(_mag_fd, DEVIOCGDEVICEID, 0));
   return install();

}

void AP_Compass_PX4::read(void)
{
    // try to accumulate one more sample, so we have the latest data
   accumulate();
   if (_count > 0){
      _sum /= _count;
      publish_filtered_field(_sum);
      _sum.zero();
      _count = 0;
   }
}

void AP_Compass_PX4::accumulate(void)
{
   struct mag_report mag_report;

   while (::read(_mag_fd, &mag_report, sizeof(mag_report)) == sizeof(mag_report) &&
         mag_report.timestamp != _last_timestamp) {
      uint32_t time_us = (uint32_t)mag_report.timestamp;

      Vector3f raw_field = Vector3f(mag_report.x, mag_report.y, mag_report.z)*1.0e3f;
      rotate_field(raw_field);
      publish_raw_field(raw_field, time_us);
      correct_field(raw_field);
      _sum += raw_field;
      _count++;
      _last_timestamp = mag_report.timestamp;
   }
}

#endif // CONFIG_HAL_BOARD
