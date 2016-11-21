
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
  test of basic storage function
*/

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

  // constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint32_t ee_addr = 6;

   struct test_task_t{

      test_task_t(): m_count{false}{}

      void fun()
      { 
          char const text [] = "Hello\n";
          uint32_t len = strlen(text)+1;
       //   hal.gpio->toggle(red_led_pin);
          if ( m_count == false){
             hal.storage->write_block(ee_addr,text,len);
             m_count = true;
          }else{
             hal.console->printf("-------------------------\n");
             char buffer[100] = {'\0'};
             hal.storage->read_block(buffer,ee_addr,len);
             if ( buffer[0] != '\0'){
               hal.console->write("got something\n");
               //hal.console->write((unsigned char const*)buffer,len);
             }else{
               hal.console->write("read failed\n");
             }
             hal.console->printf("-------------------------\n");
             m_count = false;
          }
      };

      void init()
      {
         // hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
         // hal.gpio->write(red_led_pin,pin_off);
          
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
//   test_task.init();
//
//   test_task.fun();
//   test_task.fun();
}

void on_telemetry_transmitted()
{
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
   vTaskDelayUntil(&prev_wake_time,500); 

   hal.gpio->toggle(test_pin);
   
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_uartC = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )


