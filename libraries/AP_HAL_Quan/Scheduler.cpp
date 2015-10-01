
#include "Scheduler.h"

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/max.hpp>
#include "i2c_task.hpp"
//#include <quan/stm32/gpio.hpp>
//#include <resources.hpp>

using namespace Quan;

extern const AP_HAL::HAL& hal;


namespace {

   typedef quan::stm32::tim13 usec_timer;
   void setup_usec_timer()
   {
      quan::stm32::module_enable<usec_timer>();
      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<usec_timer>();
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");
      usec_timer::get()->psc = psc;
      usec_timer::get()->cnt = 0;
      usec_timer::get()->arr = 0xffff;
      usec_timer::get()->sr = 0;
      usec_timer::get()->dier.setbit<0>(); //(UIE)  

      NVIC_SetPriority(TIM8_UP_TIM13_IRQn,14);
      NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
   }

   void start_usec_timer()
   {
      usec_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }
   
   // overflow after  approx 9 years
   volatile uint32_t timer_micros_ovflo_count = 0U;
 
} // namespace


extern "C" void TIM8_UP_TIM13_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

extern "C" void TIM8_UP_TIM13_IRQHandler()
{
   if ( usec_timer::get()->sr & (1 << 0) ) {// UIF
      usec_timer::get()->sr = 0;
      ++ timer_micros_ovflo_count;
   }
}


#if 0
namespace {

   // for vTaskDelayUntil
 //  TickType_t last_scheduler_timer_task_wake_time = 0;
   // The queue for task messages
   // could be more sophisticated
   // to remove tasks as well as add
  // QueueHandle_t scheduler_timer_task_message_queue = nullptr;
   // max number of timer task procs
 //  constexpr uint32_t max_scheduler_timer_procs = 4;
 //  constexpr uint32_t scheduler_timer_task_queue_length = max_scheduler_timer_procs;

   // static temp for new task from queue
 //  AP_HAL::MemberProc new_scheduler_timer_task_proc = nullptr;
   // the array of timer procs to call in the slot
 //  AP_HAL::MemberProc timer_procs[max_scheduler_timer_procs] = {nullptr};

  // bool m_in_timer_process = false;

  // bool request_timer_procs_suspended = false;

  // bool timer_procs_suspended = false;

   void scheduler_timer_task(void * params)
   {
      scheduler_timer_task_message_queue = xQueueCreate(2,sizeof(AP_HAL::MemberProc));
      if ( scheduler_timer_task_message_queue == nullptr){
         hal.scheduler->panic("Create Sched timer_task Q failed");
      }

      for(;;){

         while ( uxQueueMessagesWaiting(scheduler_timer_task_message_queue) != 0){
            xQueueReceive(scheduler_timer_task_message_queue,&new_scheduler_timer_task_proc,0);
            if ( (new_scheduler_timer_task_proc == nullptr) == false){
               bool new_proc_installed = false;
               // check for duplicate
               for ( auto const & pfn : timer_procs){
                  if ( new_scheduler_timer_task_proc == pfn){
                     new_proc_installed = true;
                     break;
                  }
               }
               // check for free slot
               if (!new_proc_installed){
                  for (auto & pfn : timer_procs){
                     if (pfn == nullptr){
                        pfn = new_scheduler_timer_task_proc;
                        new_proc_installed = true;
                        break;
                     }
                  }
               }
               if (! new_proc_installed){
                   hal.console->printf("Install Sched timer task proc failed\n");
               }
            }
         }
         // 
         if ( request_timer_procs_suspended){
            timer_procs_suspended = true;
         }else{
            timer_procs_suspended = false;
            // run all the functions
            for ( auto& pfn : timer_procs){
               if ( (pfn == nullptr) == false){
               //   taskENTER_CRITICAL();
                  m_in_timer_process = true;
                  pfn();
                  m_in_timer_process = false;
                //  taskEXIT_CRITICAL();
               }
            } 
         }
         // can add other tasks here
         // e.g A2D
         // and sleep 1 ms till next time
         vTaskDelayUntil(&last_scheduler_timer_task_wake_time, 1);
      }
   }

   TaskHandle_t scheduler_timer_task_handle;
   void * dummy_params;

/*
 TODO look at functions to assess how much memory to allocate
  Check for task safe etc
*/
   void create_scheduler_timer_task()
   {
      xTaskCreate(
         scheduler_timer_task,"scheduler_timer_task",
         1000,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
         & scheduler_timer_task_handle
      ) ;
   }

} // namespace
#endif

QuanScheduler::QuanScheduler()
{}
//void create_adc_task();
// called in HAL_Quan::init( int argc, char * const * argv)
// after console and GPIO inited
void QuanScheduler::init(void* )
{
   setup_usec_timer();
  // create_scheduler_timer_task();
   Quan::create_i2c_task();
   // can now get Compass and baro q handles
   
   start_usec_timer();
}

namespace{
   AP_HAL::Proc m_delay_callback = nullptr;
   uint16_t m_delay_callback_min_delay_ms = 0;
}

void QuanScheduler::delay(uint16_t delay_length_ms)
{
   uint64_t const end_of_delay_ms  = millis64() + delay_length_ms;
   for (;;){
      uint64_t const time_now_ms = millis64();
      if ( m_delay_callback && (( time_now_ms + m_delay_callback_min_delay_ms) < end_of_delay_ms)){
         m_delay_callback();
         if ( millis64() < end_of_delay_ms){
            vTaskDelay(1);
         }else{
            break; // end of delay
         }
      }else{
         if ( time_now_ms < end_of_delay_ms){
            vTaskDelay(end_of_delay_ms - time_now_ms );
         }
         break; // end of delay
      }
   }
}

uint64_t QuanScheduler::micros64() {
   taskENTER_CRITICAL();
   uint32_t const hi1 = timer_micros_ovflo_count;
   uint16_t const lo1 = usec_timer::get()->cnt;
   uint32_t const hi2 = timer_micros_ovflo_count;
   if ( hi2 == hi1){
      taskEXIT_CRITICAL();
      return (static_cast<uint64_t>(hi1) << 16U) | lo1; 
   }else{
      uint16_t const lo2 = usec_timer::get()->cnt;
      taskEXIT_CRITICAL();
      return (static_cast<uint64_t>(hi2) << 16U) | lo2; 
   }
}

uint64_t QuanScheduler::millis64() 
{
   return micros64() / 1000ULL;
}

uint32_t QuanScheduler::millis() {
    return millis64();
}

uint32_t QuanScheduler::micros() {
    return micros64();
}

 // delay longer than 1 ms will in fact actively yield to other tasks
void QuanScheduler::delay_microseconds(uint16_t us)
{
   uint64_t const end_of_delay_us = micros64() + us;
   uint64_t const delay_ms = us / 1000;
   // yields
   if ( delay_ms > 0){
      delay(delay_ms);
   }
   // not going critical here. yield away!
   while (micros64() < end_of_delay_us){ 
      asm volatile ("nop":::);
   }
}

/*
a function to do useful stuff during the delay function
 maybe be null
 called during Plane::init_ardupilot fun
 to run Mavlink output fun
 TODO .. Remove this 
*/
void QuanScheduler::register_delay_callback(AP_HAL::Proc pfn,
            uint16_t min_time_ms)
{
   m_delay_callback = pfn;
   m_delay_callback_min_delay_ms = min_time_ms;
}

namespace{
     AP_HAL::MemberProc new_scheduler_timer_task_proc_in = nullptr;
}
void QuanScheduler::register_timer_process(AP_HAL::MemberProc mp)
{
    panic("QuanScheduler::register_timer_process called\n");
//   if ( (mp == nullptr) == false){
//      new_scheduler_timer_task_proc_in = mp;
//      if (xQueueSendToBack(scheduler_timer_task_message_queue,&new_scheduler_timer_task_proc_in,2) == errQUEUE_FULL){
//          hal.console->printf("failed to add new scheduler_timer_task proc\n");
//      }
//   }
}

//"not supported on AVR" so wont bother yet
void QuanScheduler::register_io_process(AP_HAL::MemberProc k)
{
   panic("QuanScheduler::register_io_process called\n");

}

// TODO register a function to call on failsafe ( eg. WDT timeout)
// and sort failsafe watchdog etc
void QuanScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

// request 
void QuanScheduler::suspend_timer_procs()
{
//   request_timer_procs_suspended = true;
//   while (!timer_procs_suspended){
//      delay(1);
//   }
}

void QuanScheduler::resume_timer_procs()
{
//    request_timer_procs_suspended = false;
//    while (timer_procs_suspended){
//      delay(1);
//   }
}

// call from interrupt?
bool QuanScheduler::in_timerprocess() 
{
    return false; // m_in_timer_process; 
}

namespace {
  bool m_system_initialised = false;
}

bool QuanScheduler::system_initializing() 
{
    return ! m_system_initialised;
}

void QuanScheduler::system_initialized()
{
   m_system_initialised = true;
}

void QuanScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

// TODO wdt
// no bootloader
void QuanScheduler::reboot(bool hold_in_bootloader) 
{
    NVIC_SystemReset();
   // for(;;);
}
