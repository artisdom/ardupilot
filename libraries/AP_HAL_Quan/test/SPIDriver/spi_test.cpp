
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

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   struct test_task_t{

      test_task_t():m_count{0}{}

      void fun()
      {
         if (++m_count == 500){
            m_count = 0;

            hal.gpio->toggle(red_led_pin);

            auto * mpu6 = hal.spi->device(AP_HAL::SPIDevice_MPU6000);
            if ( mpu6){
               auto * sem = mpu6->get_semaphore();
               if ( sem && sem->take_nonblocking() ){
                 // do something with mpu
                  uint8_t who_am_i = reg_read(mpu6,117);
                  hal.console->printf("Who am i = %u\n",static_cast<unsigned int>(who_am_i));
                  sem->give();
               }else{
                  hal.console->printf("couldnt get spi semaphore\n");
               }
            }else{
               hal.console->printf("couldnt get mpu6000 ptr\n");
            }
         }
      };

      void reg_write(AP_HAL::SPIDeviceDriver* dev, uint8_t reg, uint8_t value)
      {
          uint8_t arr[2] = {reg, value};
          dev->transaction( arr, nullptr, 2);
      }

      uint8_t reg_read(AP_HAL::SPIDeviceDriver* dev, uint8_t reg)
      {
         uint8_t arr_out[2] = {static_cast<uint8_t>(reg | (1 << 7)),0};
         uint8_t arr_in [2] = {0,0};
         dev->transaction(arr_out,arr_in, 2);
         return arr_in[1];
      }

      void mpu6000_init()
      {
         auto * mpu6 = hal.spi->device(AP_HAL::SPIDevice_MPU6000);
         if (mpu6){
            auto * sem = mpu6->get_semaphore();
            if ( sem && sem->take_nonblocking() ){
               //disable i2c
               reg_write(mpu6, 106 , (1 << 4));
               hal.scheduler->delay(10);
               // reset
               reg_write(mpu6, 107 , (1 << 7));
               hal.scheduler->delay(100);

               sem->give();
            }else{
               hal.console->printf("(init) couldnt get spi semaphore\n");
            }
         }else{
            hal.console->printf("(init) couldnt get mpu6000\n");
         }
      }

      void init()
      {
     
       mpu6000_init();
       hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&test_task_t::fun, void));
      }
   private:
       uint32_t m_count;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
   hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(red_led_pin,pin_off);
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM SPI test",{-140,50});
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
      hal.gpio->toggle(red_led_pin);
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

