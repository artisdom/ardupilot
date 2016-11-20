
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
      :m_name{name_in}, m_begin_ms{begin_ms}, m_task_length_ms{len_ms}, m_pfn{pfn}{}

      bool run()
      {
        return m_pfn();
      }

      uint32_t get_begin_ms() const {return m_begin_ms;}
      uint32_t get_length_ms() const { return m_task_length_ms;}
      const char* get_name() const {return m_name;}
     private:
         const char* const m_name;
         uint32_t const m_begin_ms;
         uint32_t const m_task_length_ms;
         bool (* const m_pfn)();
   };
#if defined QUAN_I2C_DEBUG
   void show_flags( uint32_t flags){
      if (flags & (1<<0)){
          hal.console->printf("sb |");
      }

      if(flags & (1<<1) ){
          hal.console->printf("addr |");
      }
      
      if(flags & (1<<2) ){
          hal.console->printf("btf |");
      }

      if(flags & (1<<4) ){
          hal.console->printf("stopf |");
      }

      if(flags & (1<<6) ){
          hal.console->printf("rxne |");
      }

      if(flags & (1<<7) ){
          hal.console->printf("txe |");
      }

      if(flags & (1<<8) ){
          hal.console->printf("berr |");
      }

      if(flags & (1<<9) ){
          hal.console->printf("arlo |");
      }

      if(flags & (1 << 10) ){
          hal.console->printf("af |");
      }
      if(flags & (1 << 10) ){
          hal.console->printf("ovr |");
      }

      hal.console->printf("\n");

   }
#endif
   void i2c_task(void* params)
   {
      // need to wait for 5V on the external sensor voltage regs
      wait_for_power_up();

      hal.console->printf("starting i2c task\n");

      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) ){
        AP_HAL::panic("create FreeRTOS queues failed in I2c task\n");
      }
      
      Quan::i2c_periph::init();

      if (!Quan::setup_baro()) {
         AP_HAL::panic("baro setup failed");
      }

      if (!Quan::setup_compass()) {
         AP_HAL::panic("compass setup failed");
      }

      uint32_t looptime_ms = 50U;
      TickType_t previous_waketime_ms = xTaskGetTickCount();

      task tasks [] = {
          {"baro : request conversion"    ,  1 ,   1, Quan::baro_request_conversion}
         ,{"compass : request conversion" ,  2 ,   1, Quan::compass_request_conversion}
          // 7 ms spare add Airspeed here
         ,{"compass : start_read"         ,  10,   2, Quan::compass_start_read}
         ,{"compass_calculate"            ,  13 ,  1, Quan::compass_calculate}  
         ,{"eeprom : opt write"           ,  14 , 25, Quan::eeprom_opt_write}
         ,{"baro : start read"            ,  45 ,  2, Quan::baro_start_read}
         ,{"baro : calculate"             ,  47 ,  1, Quan::baro_calculate}
      };

      constexpr uint32_t num_tasks = sizeof(tasks)/ sizeof(task);

      bool succeeded = true;
      for (;;){
#if defined QUAN_I2C_DEBUG
         flags_idx = 0;
#endif
         auto const loop_start_ms = millis();
         for ( uint32_t i = 0; i < num_tasks ; ++i){
           // do any eeprom reads in a timely fashion, though it messses up the timing of other tasks
           if (!Quan::eeprom_opt_read()){
               succeeded = false;
               break;
           }
           task & t = tasks[i];
           auto const time_since_task_started = millis() - loop_start_ms;
           if ( time_since_task_started < t.get_begin_ms()  ){
              vTaskDelay( t.get_begin_ms() - time_since_task_started);
           }
           if ( !t.run()){
               hal.console->printf("i2c task %s failed\n",t.get_name());
               succeeded = false;
               break;
           }
         }
         if (succeeded) { 
            vTaskDelayUntil(&previous_waketime_ms, looptime_ms);
         }else{
            break;
         }
      }

      // ----------------- get here on fail

      if ( Quan::i2c_periph::has_errored()){
         hal.console->printf("NOTE:--- i2c has flagged errored ---\n");
      }else{
         hal.console->printf("--- i2c has failed but no errors flagged ---\n");
      }
#if defined QUAN_I2C_DEBUG
      // show flags to here
      for (uint32_t idx = 0; idx < flags_idx; ++idx){
         while (hal.console->tx_pending() ){asm volatile ("nop":::);}
         hal.console->printf("flags[%s] = %lX\n",infos[idx].m_name,infos[idx].m_flags);
         show_flags(infos[idx].m_flags);
      }

      hal.console->printf("----------------\n");

      // show we are still running
      uint32_t count = 0;
      for(uint32_t i = 0; i < 120;++i){
         vTaskDelayUntil(&previous_waketime_ms,looptime_ms);
         hal.console->printf("*");
         if (++count == 40){
            count = 0;
            hal.console->printf("\n");
         }
      }
#endif
      // do nothing
      for(;;){
         vTaskDelayUntil(&previous_waketime_ms,looptime_ms);
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

      hBaroQueue = xQueueCreate(1,sizeof(Quan::detail::baro_args));
      hCompassQueue = xQueueCreate(1,sizeof(Quan::detail::compass_args));

      if (! Quan::setup_eeprom()){
         AP_HAL::panic("eeprom setup failed");
      }

      xTaskCreate(
         i2c_task,"I2C_task",
         1000,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
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