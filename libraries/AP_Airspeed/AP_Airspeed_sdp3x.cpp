
#include <AP_HAL/AP_HAL.h>

#if (CONFIG_HAL_BOARD == HAL_BOARD_QUAN)  && (defined QUAN_MIXER_TRANQUILITY)

#include "AP_Airspeed_sdp3x.hpp"
#include <AP_HAL_Quan/i2c_task.hpp>

#include <task.h>

extern const AP_HAL::HAL& hal;

namespace {
   AP_Airspeed_sdp3x airspeed_driver;
}

template <> AP_Airspeed_Backend * connect_airspeed_driver<Quan::tag_board>(AP_Airspeed & airspeed)
{
 // give time for sensor to stabilise
   uint32_t now = AP_HAL::millis();
   if ( now < 1000){
         hal.scheduler->delay(1000 - now);
   }
   airspeed_driver.connect(Quan::get_airspeed_queue_handle());
   return &airspeed_driver;
}

void AP_Airspeed_sdp3x::update()
{
   Quan::detail::airspeed_args args;

   if ( (m_hQueue != NULL) && ( xQueueReceive(m_hQueue, &args,0) == pdTRUE) ) {
      m_diff_pressure = args.differential_pressure;
      m_temperature = args.temperature;
   }
}

#endif