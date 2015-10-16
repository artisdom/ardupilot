#ifndef AP_OSD_H_INCLUDED
#define AP_OSD_H_INCLUDED

#include <quan/three_d/vect.hpp>

namespace AP_OSD{

   // union for sending different data types
   union osd_data_t{
      osd_data_t(){}
      quan::three_d::vect<float> vect3df;
      float f;
   };

   // enum for all messages
   // The order is important
   enum class msgID{
      heading , // float
      attitude, // vect3df
      raw_compass, // vect3df
      drift ,  // vect3df
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
