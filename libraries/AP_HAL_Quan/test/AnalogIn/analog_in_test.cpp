
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include "FreeRTOS.h"
#include <task.h>
#include <stm32f4xx.h>
#include <cstring>
#include <cstdio>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/functor.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <quantracker/osd/osd.hpp>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   constexpr uint8_t red_led_pin = 1U;
   constexpr uint8_t test_pin = 2U;
   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;
   constexpr uint8_t num_adc_channels = 5U;
   constexpr float battery_voltage_scale = 4.29541951026795;

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {
         if (++m_count == 500){
            m_count = 0;
            for ( int i = 0; i < num_adc_channels; ++i){
               float voltage = hal.analogin->channel(i)->voltage_average();
                  hal.console->printf("voltage[%d] = %f V\n",i,static_cast<double>(voltage));
               }
            }
         }
      };

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
      }
   private:
      uint32_t m_count ;
   } test_task;

   float get_battery_voltage()
   {
       vTaskSuspendAll();
       float result = hal.analogin->channel(3)->voltage_average() * battery_voltage_scale;
       xTaskResumeAll();
       return result;
   }

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM Analogin test\n");
   test_task.init();
}

void on_telemetry_transmitted()
{
}

namespace {
   const char * adc_descr[]  = {
          "ana  "
         ,"rssi "
         ,"curr "
         ,"volt "
         ,"aspd "
   };
}

void quan::uav::osd::on_draw() 
{ 
   pxp_type pos {-150,80};
   draw_text("Quan APM AnalogIn test",pos);

   float battery_voltage = get_battery_voltage();
   char buf[100];
   sprintf(buf,"battery voltage = % 5.2f",static_cast<double>(battery_voltage));
   pos.y -= 20;
   draw_text(buf,pos);
   for ( uint32_t i = 0; i < num_adc_channels; ++i){
      sprintf(buf,"v[%2lu]=% 5.2f (%s)",i,static_cast<double>(hal.analogin->channel(i)->voltage_average()),adc_descr[i]);
      pos.y -= 20;
      draw_text(buf,pos);
   }

}

namespace {

   TickType_t prev_wake_time= 0; 
   uint32_t led_count = 0;

}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,1); 
   test_task.fun();
   if ( ++led_count == 200){
      led_count = 0;
      hal.gpio->toggle(test_pin);
   }
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_uartA = true;
      flags.init_analogin = true;
      flags.init_gpio = true;
      flags.init_scheduler = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif
