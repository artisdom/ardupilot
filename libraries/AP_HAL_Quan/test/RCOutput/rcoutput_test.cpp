
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
   Test of the RC Output
   Reads the inputs and sends
   them to the outputs
*/

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   int count = 0;
   constexpr uint8_t red_led_pin = 1U;
   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {
         if (hal.rcin->new_input()){
            uint8_t num_rc_in_channels = hal.rcin->num_channels();
            for ( uint8_t i = 0; i < num_rc_in_channels; ++i){
               hal.rcout->write(i,hal.rcin->read(i));
            }
         }
         if (++count == 500){
            hal.gpio->toggle(red_led_pin);
         }
      };

      void init()
      {
          for (uint8_t i =0; i < 4; ++i){
             hal.rcout->enable_ch(i);
          }
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
      }
   private:
      uint32_t m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM RC Output test\n");
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Sched RC Output test",{-140,50});
}

namespace {

}
// called forever in apm_task
void loop() 
{
    hal.scheduler->delay(1);
    test_task.fun();
}

#if defined QUAN_WITH_OSD_OVERLAY
AP_HAL_MAIN();
#else
void create_apm_task();
void create_timer_task();

extern "C" {
   int main (void) 
   {
      osd_setup(); 
      create_draw_task(); 
      create_apm_task(); 
      vTaskStartScheduler (); 
   }
}

namespace { 
   char dummy_param = 0; 
   TaskHandle_t task_handle = NULL; 
   void apm_task(void * params) 
   { 
      hal.init(0, NULL);
      setup();
      hal.scheduler->system_initialized(); 
      test_task.init();
      for(;;){ 
         loop(); 
      } 
   } 
} 

void create_apm_task() 
{ 
  xTaskCreate( 
      apm_task,"apm task", 
      5000, 
      &dummy_param, 
      tskIDLE_PRIORITY + 1, 
      &task_handle 
  ); 
}
#endif

