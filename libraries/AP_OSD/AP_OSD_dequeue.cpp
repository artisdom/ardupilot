
#include <cstdio>
#include "AP_OSD_dequeue.h"
#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>
#include <quan/uav/get_bearing_and_distance.hpp>

namespace{

   void get_attitude(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      typedef quan::angle_<float>::deg deg;
      info.attitude 
         = quan::uav::osd::attitude_type{
               deg{msg.value.vect3df.z},
               deg{msg.value.vect3df.x},
               deg{msg.value.vect3df.y}
         };
   }

   void get_gps_status(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.gps_status = msg.value.gps_info.status;
      info.gps_num_sats = msg.value.gps_info.num_sats;
      info.ground_speed = quan::velocity_<float>::m_per_s{msg.value.gps_info.ground_speed_m_per_s};
      info.ground_course = quan::angle_<float>::deg{msg.value.gps_info.ground_course_cd / 100.f};
   }

   void get_gps_location(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      quan::angle_<int32_t>::deg10e7 lat{msg.value.vect3di32.x};
      quan::angle_<int32_t>::deg10e7 lon{msg.value.vect3di32.y};
      quan::length_<int32_t>::cm     alt{msg.value.vect3di32.z};
      info.aircraft_position = quan::uav::osd::position_type{lat,lon,alt};
   }

   void get_home_location(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      quan::angle_<int32_t>::deg10e7 lat{msg.value.vect3di32.x};
      quan::angle_<int32_t>::deg10e7 lon{msg.value.vect3di32.y};
      quan::length_<int32_t>::cm     alt{msg.value.vect3di32.z};
      info.home_position = quan::uav::osd::position_type{lat,lon,alt};
      info.home_is_set = true;
   }

   void get_airspeed(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.airspeed = quan::velocity_<float>::m_per_s{msg.value.f};
   }

   void get_battery(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.battery_voltage = quan::voltage_<float>::V{msg.value.vect3df.x};
      info.battery_current = quan::current_<float>::A{msg.value.vect3df.y};
      info.battery_mAh_consumed = quan::charge_<float>::mA_h{msg.value.vect3df.z};
   }

   void get_system_status(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.system_status = msg.value.sys_status;
   }

   void get_rcin_0_to_5(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[i] = msg.value.u16_array6[i];
        }
   }

   void get_rcin_6_to_11(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[6 + i] = msg.value.u16_array6[i];
        }
   }

   void get_rcin_12_to_17(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[12 + i] = msg.value.u16_array6[i];
        }
   }

   void get_control_mode(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
       info.control_mode = msg.value.u8;
   }
   
   typedef void(*fun_ptr)(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info);

   // order must match enums
   fun_ptr funs[] = {
   //   get_heading,
      get_attitude,
   //   get_raw_compass,
   //   get_drift,
      get_gps_status, // uint8_t
      get_gps_location, // vect3di32
      get_home_location, // vect3di32
  //    get_baro_altitude, // float
      get_airspeed,
      get_battery,
      get_system_status,
      get_rcin_0_to_5,
      get_rcin_6_to_11,
      get_rcin_12_to_17,
      get_control_mode
   };

   QueueHandle_t osd_queue = nullptr;

} // namespace
   
// called by osd thread to get latest data
void AP_OSD::dequeue::read_stream(AP_OSD::dequeue::osd_info_t& info)
{
   if ( osd_queue != nullptr){
      AP_OSD::osd_message_t msg;
      while ( xQueueReceive(osd_queue,&msg,0) == pdTRUE){
         uint32_t const id = static_cast<uint32_t>(msg.id);
         if ( id < static_cast<uint32_t>(AP_OSD::msgID::max_messages)){
             fun_ptr fun = funs[id];
             fun(msg,info);
         }
      }
   }
}

void AP_OSD::dequeue::update(AP_OSD::dequeue::osd_info_t& info)
{
   // recalc distance to home and bearing
   quan::uav::get_bearing_and_distance(
      info.home_position,info.aircraft_position,
      info.bearing_to_home,info.distance_from_home);
}

namespace AP_OSD { namespace dequeue {namespace detail{

   QueueHandle_t  initialise()
   {
      osd_queue = xQueueCreate(30,sizeof(osd_message_t));
      return osd_queue;
   }
}}}

