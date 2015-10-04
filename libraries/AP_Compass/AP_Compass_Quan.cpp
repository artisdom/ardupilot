

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Compass_Quan.h"

extern const AP_HAL::HAL& hal;

AP_Compass_Quan::AP_Compass_Quan(Compass &compass)
:AP_Compass_Backend(compass)
,m_hQueue{Quan::get_compass_queue_handle()}
{
   m_instance = register_compass();
   set_dev_id(m_instance,AP_COMPASS_TYPE_QUAN);
   set_milligauss_ratio(m_instance,1.0f);
   set_external(m_instance, true);
}

bool  AP_Compass_Quan::init(void)
{return true;}


// todo add device id etc 

void  AP_Compass_Quan::read(void)
{
   Quan::detail::compass_args args;
   // receive should be available in 1/5th sec!
   if ( xQueueReceive(m_hQueue, &args,200) == pdTRUE) {

      Vector3f raw_field{
         args.field.x.numeric_value()
         ,args.field.y.numeric_value()
         ,args.field.z.numeric_value()
      };
      rotate_field(raw_field, m_instance);
      publish_raw_field(raw_field, args.time_us, m_instance);
      correct_field(raw_field, m_instance);
      publish_unfiltered_field(raw_field, args.time_us, m_instance);
      publish_filtered_field(raw_field, m_instance);
   }else{
      hal.console->printf("failed to receive Compass update after 1/5th sec\n");
   }
}

AP_Compass_Backend * AP_Compass_Quan::detect(Compass &compass)
{  
   return new AP_Compass_Quan(compass);
}

#endif  //#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
