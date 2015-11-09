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

   bool gps_status(gps_info_t info);
   bool gps_location(quan::three_d::vect<int32_t> const & in);
   bool home_location(quan::three_d::vect<int32_t> const & in);
 //  bool baro_altitude(float baro_alt_m);
   bool airspeed(float m_per_s);
   bool battery(quan::three_d::vect<float> const & in); // Voltage, Current, MaH
   bool system_status(AP_OSD::system_status_t status); // status, numsats
   bool rc_inputs_0_to_5(uint16_t * arr, uint8_t n);
   bool rc_inputs_6_to_11(uint16_t * arr, uint8_t n);
   bool rc_inputs_12_to_17(uint16_t * arr, uint8_t n);
}}

#endif // AP_OSD_ENQUEUE_H_INCLUDED
