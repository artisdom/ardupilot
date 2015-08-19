
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL/utility/functor.h>
#include <AP_Math/vector3.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>


/*
   Test of the Timer task
   tset_task justs blinks an LED but in the timer task callback
*/

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   // connect a HMC5883 mag to i2c3 on pA8 scl, pa9 sda to test

   constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint8_t mag_addr = 0x1E;
   static const uint8_t configA = 0x0;
   static const uint8_t configB = 0x1;
   static const uint8_t modereg =  0x02;
   static uint8_t msb_x = 0x03;
   static const uint8_t single_measurement = 0x01;
   uint8_t values [6] = {0,0,0,0,0,0};


   int16_t convert_to_int16(uint8_t * d)
   {
      union{
         uint8_t in[2] ;
         int16_t out;
      }u;
      u.in[0] = d[1];
      u.in[1] = d[0];
      return u.out;
   }

   void copy_new_values(Vector3<int> & result_out)
   {
      result_out.x = convert_to_int16(values);
      result_out.y = convert_to_int16(values + 4);
      result_out.z = convert_to_int16(values + 2);
   }

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {
         if (++m_count == 500){
            m_count = 0;

            hal.gpio->toggle(red_led_pin);

            auto * sem = hal.i2c->get_semaphore();
            
            if ( sem && sem->take_nonblocking() ){

               if (  hal.i2c->writeRegister(mag_addr,modereg,single_measurement) != 0){
                  return; // should report on error
               }
               if ( hal.i2c->write(mag_addr,1,&msb_x) !=0 ){
                  return;
               }
               if ( hal.i2c->read(mag_addr,6,values) != 0){
                  return;
               }

               hal.console->printf("read success\n");

               Vector3<int> result;

               copy_new_values(result);

               hal.console->printf("result = [%d, %d,%d]\n",result.x,result.y,result.z);
               
               sem->give();

            }else{
               hal.console->printf("couldnt get semaphore\n");
            }
         }
      };

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
          hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&test_task_t::fun, void));
      }
   private:
      uint32_t m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM I2C test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM I2C test",{-140,50});
}

namespace {
   constexpr uint8_t interval = 10U;
   uint64_t next_event = interval;
}
// called forever in apm_task
void loop() 
{
   uint64_t const now = hal.scheduler->millis64();
   if ( next_event <= now ){
      hal.gpio->toggle(test_pin);
      next_event = now + interval;
   }
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

