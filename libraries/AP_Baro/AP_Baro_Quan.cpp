

#include "AP_Baro_Quan.h"

 AP_Baro_Quan::AP_Baro_Quan(AP_Baro & baro)
 : AP_Baro_Backend{baro}
  ,m_hQueue{Quan::get_baro_queue_handle()}
  ,m_instance{baro.register_sensor()}
{
  
}

void  AP_Baro_Quan::update()
{
   // n.b. ignore any new samples during the read
   uint32_t num_samples = uxQueueMessagesWaiting(m_hQueue);
   if ( num_samples > 0){
      quan::pressure_<float>::Pa pressure_sum = quan::pressure_<float>::Pa{0};
      quan::temperature_<float>::K temperature_sum = quan::temperature_<float>::K{0};
      for ( uint32_t i = 0; i < num_samples; ++i){
         Quan::detail::baro_args args;
         xQueueReceive(m_hQueue, &args,0) ;
         pressure_sum += args.pressure;
         temperature_sum += args.temperature;
      }
      quan::pressure_<float>::Pa const pressure = pressure_sum / num_samples;
      quan::temperature_<float>::K const temperature =  temperature_sum / num_samples;
      // convert temperature to Centigrade from Kelvin
      float const temperature_C = temperature.numeric_value() - 273.15f;
      float const pressure_Pa  = pressure.numeric_value();

      _copy_to_frontend(m_instance, pressure_Pa, temperature_C);
   }
}



