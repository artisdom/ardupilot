
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include "i2c_task.hpp"


#include <AP_HAL_Quan/i2c/i2c_driver/i2c_periph.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_eeprom_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_register_based_driver.cpp>

namespace {

      // queue for sending baro data to apm thread
   QueueHandle_t hBaroQueue = nullptr;

   // queue for sending compass data to apm thread
   QueueHandle_t hCompassQueue = nullptr;

   void wait_for_power_up();

   void i2c_task(void* params)
   {
      // need to wait for 5V on the external sensor voltage regs
      wait_for_power_up();

      Quan::i2c_periph::init();
      

      hal.console->printf("starting i2c task\n");

      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) ){
        // panic("create FreeRTOS queues failed in I2c task\n");
        AP_HAL::panic("create FreeRTOS queues failed in I2c task\n");
      }

      // start the baro
      // try to detect the external compass
      // if no internal compass use internal compass
      // list of tasks
      
      for (;;)
      {

      }
      
   }

   void wait_for_power_up()
   {
      uint32_t const now_ms = AP_HAL::millis();
      if( now_ms < 200){ // should be board_startup_time
         hal.scheduler->delay(200 - now_ms);
      }
   }

   TaskHandle_t task_handle;
   void * dummy_params;
}

namespace Quan {

   void create_i2c_task()
   {
      // As long as the baro data is read at less than 50 Hz
      // and the compass at less than 100 Hz
      // 1 in queue should be ok
      // The update rates are listed in ArduPlane.cpp
      // as 10 Hz for update_compass
      // and 10 Hz for update_alt ( baro)
      // (The higher rate accumulate functions are not used or required)
      // Same rates in ArduCopter
      constexpr uint32_t num_in_queue = 1;
      hBaroQueue = xQueueCreate(num_in_queue,sizeof(Quan::detail::baro_args));
      hCompassQueue = xQueueCreate(num_in_queue,sizeof(Quan::detail::compass_args));

      xTaskCreate(
         i2c_task,"I2C_task",
         1000,
         &dummy_params,
         tskIDLE_PRIORITY + 3, // want slightly higher than apm task priority
         & task_handle
      ) ;
   }

   QueueHandle_t get_compass_queue_handle()
   {
      if ( hCompassQueue == nullptr){
         panic("Requesting null compass queue handle\n");
      }
      return hCompassQueue;
   }

   QueueHandle_t get_baro_queue_handle()
   {
      if ( hBaroQueue == nullptr){
         panic("Requesting null baro queue handle\n");
      }
      return hBaroQueue;
   }
}


#endif // #if defined QUAN_AERFLITE_BOARD
#endif // AP_HAL_QUAN