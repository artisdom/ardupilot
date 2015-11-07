#ifndef AP_OSD_H_INCLUDED
#define AP_OSD_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <quan/three_d/vect.hpp>
#include <quan/angle.hpp>

namespace AP_OSD{

   enum class system_status_t{
      starting = 0
     , in_cli
     , initialising
     , running
   };

   // union for sending different data types
   union osd_data_t{
      
      osd_data_t(){}
      quan::three_d::vect<float>      vect3df;
      quan::three_d::vect<uint32_t> vect3du32;
      quan::three_d::vect<int32_t>  vect3di32;
      uint8_t                byte_array12[12];
      int16_t                   i16_array6[6];
      uint16_t                  u16_array6[6];
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
   // ID to identify what is in the message
   enum class msgID{
    //  heading , // float
      attitude, // vect3df
  //    raw_compass, // vect3df
    //  drift ,  // vect3df
      gps_status, // uint8_t
      gps_location, // vect3du32 lat, lon, alt
      home_location,
     // baro_altitude,
      airspeed,
      battery,
      system_status,
      rcin_0_to_5,
      rcin_6_to_11,
      rcin_12_to_17,
      
      max_messages
   };

   // the message object sent by copy on the queue
   struct osd_message_t{
      osd_message_t(){}
      msgID id;
      osd_data_t value;
   };

   struct OSD_params{
      OSD_params();
      quan::angle_<float>::deg artifical_horizon_pitch_adjustment;
      int32_t viewing_distance_px; 
   };

   namespace dequeue{
      struct osd_info_t;
   }

   void draw_artificial_horizon(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_compass(dequeue::osd_info_t const &,OSD_params const & osd);

}

#endif // AP_OSD_H_INCLUDED
