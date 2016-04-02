
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
   Test of the RC Input
   shows positions of all inputs in microseconds

   also blinks heartbeat LED at 0.5 Hz
*/

//const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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

void on_telemetry_transmitted()
{
}

// called forever in apm_task
void loop() 
{
   hal.scheduler->delay(1);
   test_task.fun();
}

AP_HAL_MAIN();
