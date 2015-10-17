

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Airspeed_Quan.h"

extern const AP_HAL::HAL& hal;

bool AP_Airspeed_Quan::init()
{
   // give time for sensor to stabilise
   uint32_t now = hal.scheduler->millis();
   if ( now < 5000){
         hal.scheduler->delay(5000 - now);
   }
   return true;
}

// MPXV7002
bool AP_Airspeed_Quan::get_differential_pressure(float &pressure)
{

//  sensitivity  = 1 e-3 Volts/ Pa
//
//  result Pa  = V / sensitivity
//
//  Pa = v_adc  * 1000
// In fact should be offset at VCC/2 but think that offset is taken care of externally
   // ch2 is selected as Airspeed
   pressure = hal.analogin->channel(2)->voltage_average_ratiometric() * 1000.f;
   return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
