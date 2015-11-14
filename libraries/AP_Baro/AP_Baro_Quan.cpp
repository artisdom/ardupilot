
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Baro_Quan.h"

extern const AP_HAL::HAL& hal;

 AP_Baro_Quan::AP_Baro_Quan(AP_Baro & baro)
 : AP_Baro_Backend{baro}
  ,m_hQueue{Quan::get_baro_queue_handle()}
  ,m_instance{baro.register_sensor()}
{}

void  AP_Baro_Quan::update()
{
   Quan::detail::baro_args args;
   // receive should be available in 1/5th sec!
   if ( xQueueReceive(m_hQueue, &args,200) == pdTRUE) {
       // convert temperature to Centigrade from Kelvin
      float const temperature_C = args.temperature.numeric_value() - 273.15f;
      float const pressure_Pa  = args.pressure.numeric_value();
      _copy_to_frontend(m_instance, pressure_Pa, temperature_C);
   }else{
      hal.console->printf("failed to receieve Baro update after 1/5th sec\n");
   }
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN



