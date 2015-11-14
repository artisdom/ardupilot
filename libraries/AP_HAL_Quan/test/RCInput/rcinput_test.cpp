
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
   Test of the RC Input
   shows positions of all inputs in microseconds

   also blinks heartbeat LED at 0.5 Hz
*/

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {
          if (++m_count == 500){
              m_count = 0;
          
               uint8_t num_rc_in_channels = hal.rcin->num_channels();
               if ( hal.rcin->num_channels() > 0){
                  for ( int i = 0; i < num_rc_in_channels; ++i){
                   uint16_t const value = hal.rcin->read(i);
                   hal.console->printf("rc in ch[%d] = %u usec\n",i,static_cast<unsigned int>(value));
                  }
               }else{
                  hal.console->printf("no input channels\n");
               }
               hal.gpio->toggle(red_led_pin);
          }
      };

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
         // hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&test_task_t::fun, void));
      }
   private:
      uint32_t m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
   float const test_val = 1.2345;
 	hal.console->printf("Quan APM RC Input test %f\n", static_cast<double>(test_val));
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
   test_task.init();
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Sched RC Input test",{-140,50});
}

namespace {
   uint64_t next_event = 10U;
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

