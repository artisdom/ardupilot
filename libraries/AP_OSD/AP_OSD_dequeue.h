#ifndef AP_OSD_DEQUEUE_H_INCLUDED
#define AP_OSD_DEQUEUE_H_INCLUDED

#include "AP_OSD.h"
#include "AP_OSD_enqueue.h"

namespace AP_OSD { namespace dequeue{

   // The dat structure to be read by the OSD
   struct osd_info_t{
      quan::three_d::vect<float> attitude;
      quan::three_d::vect<float> drift;
      quan::three_d::vect<float> raw_compass; // vect3df
      float  heading;
   } osd_info;

   void read_stream(osd_info_t& info);

}}

#endif // AP_OSD_DEQUEUE_H_INCLUDED
