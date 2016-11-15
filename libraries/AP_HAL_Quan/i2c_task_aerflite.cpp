
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include "i2c_task.hpp"

#include <AP_HAL_Quan/i2c/i2c_driver/i2c_periph.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_eeprom_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_register_based_driver.cpp>

#include <AP_HAL_Quan/AP_HAL_Quan.h>

using AP_HAL::millis;
extern const AP_HAL::HAL& hal;

namespace {

   // queue for sending baro data to apm thread
   QueueHandle_t hBaroQueue = nullptr;
   // queue for sending compass data to apm thread
   QueueHandle_t hCompassQueue = nullptr;

   void wait_for_power_up();

   struct task{
      task(const char* name_in, uint32_t begin_ms, uint32_t len_ms, bool (*pfn)())
      :m_name{name_in}, m_begin_ms{begin_ms}, m_task_length_ms{len_ms}, m_is_run{false}, m_pfn{pfn}{}

      bool run()
      {
        m_is_run = true;
        return m_pfn();
      }

      void reset()
      {
         m_is_run = false;
      }

      bool is_run()const{ return m_is_run;}
      uint32_t get_begin_ms() const {return m_begin_ms;}
      uint32_t get_length_ms() const { return m_task_length_ms;}
      const char* get_name() const {return m_name;}
     private:
         const char* const m_name;
         uint32_t const m_begin_ms;
         uint32_t const m_task_length_ms;
         bool m_is_run;
         bool (* const m_pfn)();
   };

   void i2c_task(void* params)
   {
      // need to wait for 5V on the external sensor voltage regs
      wait_for_power_up();

      hal.console->printf("starting i2c task\n");

      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) ){
        AP_HAL::panic("create FreeRTOS queues failed in I2c task\n");
      }
      
      Quan::i2c_periph::init();

      if (! Quan::setup_baro()) {
         AP_HAL::panic("baro setup failed");
      }
      vTaskDelay(10);
      task tasks [] = {
          {"baro : request conversion",  1 , 1, Quan::baro_request_conversion}
         ,{"baro : start read"       , 45 , 2, Quan::baro_start_read}
         ,{"baro : calculate"        , 47 , 1, Quan::baro_calculate}
      };
      constexpr uint32_t num_tasks = sizeof(tasks)/ sizeof(task);
      constexpr uint32_t loop_time_ms = 49;  // 1/20th second loop for baro measurement

      TickType_t previous_waketime = xTaskGetTickCount();
      
      for (;;){

         auto loop_start_ms = millis();
         for ( uint32_t i = 0; i < num_tasks ; ++i){
           task & t = tasks[i];
          // hal.console->printf("running %s\n",t.get_name());
           if ( t.get_begin_ms() > (millis() - loop_start_ms) ){
               vTaskDelay( t.get_begin_ms() - (millis() - loop_start_ms));
           }
           if ( !t.run()){
               hal.console->printf("i2c task %s failed\n",t.get_name());
           }
         }
         vTaskDelayUntil(&previous_waketime,loop_time_ms);
         // reset
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

   //uint32_t * get_i2c_task_notify(){return & task_notify;}
   TaskHandle_t get_i2c_task_handle() { return task_handle;}

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
         tskIDLE_PRIORITY + 4, // want slightly higher than apm task priority
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