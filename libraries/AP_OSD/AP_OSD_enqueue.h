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

}}

#endif // AP_OSD_ENQUEUE_H_INCLUDED
