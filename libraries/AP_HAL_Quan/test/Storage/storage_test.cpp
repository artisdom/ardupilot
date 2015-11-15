
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
  test of basic storage function
*/

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   struct test_task_t{

      test_task_t(): m_count{false}{}

      void fun()
      {
          hal.gpio->toggle(red_led_pin);
          if ( m_count == false){
             char const text [] = "This is a string of stuff\n";
             hal.storage->write_block(5000,text,27);
             m_count == true;
          }else{
             char buffer[100];
             
             hal.storage->read_block(buffer,5000,27);

             hal.console->write((unsigned char const*)buffer,27);
             hal.console->printf("-------------------------\n");
             m_count == false;
          }
      };

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
          
      }
   private:
      bool m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM Storage test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
   test_task.init();
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Storage test",{-140,50});
}

namespace {

   TickType_t prev_wake_time= 0; 
   uint32_t led_count = 0;

}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,100); 
   test_task.fun();
   if ( ++led_count == 5){
      led_count = 0;
      hal.gpio->toggle(test_pin);
   }
}

AP_HAL_MAIN();


