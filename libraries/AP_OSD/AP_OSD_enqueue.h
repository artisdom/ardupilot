#ifndef AP_OSD_ENQUEUE_H_INCLUDED
#define AP_OSD_ENQUEUE_H_INCLUDED

#include "FreeRTOS.h"
#include <queue.h>
#include <quan/three_d/vect.hpp>

#include "AP_OSD.h"

namespace AP_OSD{ namespace enqueue{

   // the sendre inits when ready to start sending
   void initialise();

   bool attitude(quan::three_d::vect<float> const & attitude_in);
   bool drift(quan::three_d::vect<float> const & drift_in);
   bool raw_compass(quan::three_d::vect<float> const & raw_compass_in);
   bool heading(float heading_in);
   bool gps_status(uint8_t in);
   bool gps_location(quan::three_d::vect<int32_t> const & in);
   bool baro_altitude(float baro_alt_m);
   bool airspeed(float m_per_s);

}}

#endif // AP_OSD_ENQUEUE_H_INCLUDED
