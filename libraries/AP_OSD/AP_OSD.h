#ifndef AP_OSD_H_INCLUDED
#define AP_OSD_H_INCLUDED

#include <quan/three_d/vect.hpp>

namespace AP_OSD{

   // union for sending different data types
   union osd_data_t{
      osd_data_t(){}
      quan::three_d::vect<float>    vect3df;
      quan::three_d::vect<uint32_t> vect3du32;
      quan::three_d::vect<int32_t>  vect3di32;
      float f;
      uint8_t u8;
   };

   // enum for all messages
   // The order is important
   // need status, where are we bootup
   // waiting for sats etc
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
