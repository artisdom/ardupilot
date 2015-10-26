#ifndef AP_OSD_DEQUEUE_H_INCLUDED
#define AP_OSD_DEQUEUE_H_INCLUDED

#include "AP_OSD.h"
#include "AP_OSD_enqueue.h"

namespace AP_OSD { namespace dequeue{

   // The dat structure to be read by the OSD
   struct osd_info_t{
      osd_info_t()
      : attitude{0.f,0.f,0.f}
      , drift{0.f,0.f,0.f}
      , raw_compass{0.f,0.f,0.f}
      , gps_location{0,0,0}
      , heading{0.f}
      , baro_altitude{0.f}
      , airspeed{0.f}
      , gps_status{0}
      {}
      quan::three_d::vect<float> attitude;  // pitxh deg, roll deg, yaw deg
      quan::three_d::vect<float> drift;  // x,y,z units?
      quan::three_d::vect<float> raw_compass; // vect3df
      quan::three_d::vect<int32_t> gps_location; // lat deg1e7, lon deg1e7, alt cm
      float heading;  // deg
      float baro_altitude; // m
      float airspeed; // m.s[-1]
      uint8_t gps_status; // enum
   };

   void read_stream(osd_info_t& info);

}}

#endif // AP_OSD_DEQUEUE_H_INCLUDED
