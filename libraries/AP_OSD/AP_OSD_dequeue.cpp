
#include <cstdio>
#include "AP_OSD_dequeue.h"
#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>

namespace{

   // these functions are put in an array indexed by the message id

   void get_heading(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.heading = msg.value.f;
   }

   void get_attitude(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.attitude = msg.value.vect3df;
   }

   void get_raw_compass(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.raw_compass = msg.value.vect3df;
   }

   void get_drift(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.drift = msg.value.vect3df;
   }

   void get_gps_status(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.gps_status = msg.value.u8;
   }

   void get_gps_location(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.gps_location = msg.value.vect3di32;
   }

   
   void get_baro_altitude(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.baro_altitude = msg.value.f;
   }

   void get_airspeed(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.airspeed = msg.value.f;
   }

   typedef void(*fun_ptr)(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info);

   // order must match enums
   fun_ptr funs[] = {
      get_heading,
      get_attitude,
      get_raw_compass,
      get_drift,
      get_gps_status, // uint8_t
      get_gps_location, // vect3di32
      get_baro_altitude, // float
      get_airspeed
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

 namespace AP_OSD { namespace dequeue {namespace detail{

   QueueHandle_t  initialise()
   {
      osd_queue = xQueueCreate(10,sizeof(osd_message_t));
      return osd_queue;
   }
}}}

