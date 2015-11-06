#ifndef AP_OSD_H_INCLUDED
#define AP_OSD_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <quan/three_d/vect.hpp>

namespace AP_OSD{

// see also Plane::set_flight_stage
   enum class system_status_t{
      bootup = 0
     , in_cli_setup
     , initialising_sensors
     , waiting_for_gps
     , ready_to_fly
     , flying
     , system_crashed
   };

   // union for sending different data types
   union osd_data_t{
      osd_data_t(){}
      quan::three_d::vect<float>      vect3df;
      quan::three_d::vect<uint32_t> vect3du32;
      quan::three_d::vect<int32_t>  vect3di32;
      uint8_t                byte_array12[12];
      float                                 f;
      uint8_t                              u8;
      int8_t                               i8;
      uint16_t                            u16;
      int16_t                             i16;
      int32_t                             i32;
      uint32_t                            u32;
      bool                                  b;
      system_status_t              sys_status;
   };
   
   enum class msgID{
      heading , // float
      attitude, // vect3df
      raw_compass, // vect3df
      drift ,  // vect3df
      gps_status, // uint8_t
      gps_location, // vect3du32 lat, lon, alt
      baro_altitude,
      airspeed,
      battery,
      system_status,
      max_messages
   };

   // the message sent by copy on the queue
   struct osd_message_t{
      osd_message_t(){}
      msgID id;
      osd_data_t value;
   };
}


#endif // AP_OSD_H_INCLUDED
