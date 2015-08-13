
#include "Scheduler.h"

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/max.hpp>

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

      NVIC_SetPriority(TIM8_UP_TIM13_IRQn,tskIDLE_PRIORITY + 1);
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
   //otherwise maybe other timer irq
}

QuanScheduler::QuanScheduler()
{}

// called in HAL_Quan::init( int argc, char * const * argv)
// with NULL arg
// TODO check singleton
void QuanScheduler::init(void* )
{
   setup_usec_timer();
   start_usec_timer();
}

namespace{
   TickType_t last_wake_time;
   AP_HAL::Proc timer_proc = nullptr;
   int32_t timer_proc_min_delay = 0;
}

void QuanScheduler::delay(uint16_t ms)
{
   
   int64_t const end_of_delay_ms = static_cast<int64_t>(millis64()) + static_cast<int64_t>(ms) ;
   // TODO Prob better to just put the timer_proc in a task!
   while( 
      (timer_proc != nullptr ) 
      && 
      (
         timer_proc_min_delay < static_cast<int32_t>(
            (end_of_delay_ms - static_cast<int64_t>(millis64()))
         ) 
      )
    ){
      timer_proc();
    }
   int64_t const time_left_ms = end_of_delay_ms - static_cast<int64_t>(millis64());
   // try to yield whenever we can!
   if ( time_left_ms > 0){
      vTaskDelayUntil(&last_wake_time,static_cast<TickType_t>(time_left_ms));
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

//N.B granularity is 1 ms! so just here for completeness
 // delay will in fact yield to other tasks
void QuanScheduler::delay_microseconds(uint16_t us)
{
   delay((us + 999U) / 1000U);
}

/*
a function to do useful stuff during the delay function
 Actually  called during Plane::init_ardupilot fun
 to run Mavlink in a delay process
 May be easier to run Mavlink in its own task
*/
void QuanScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{
   timer_proc = k;
   timer_proc_min_delay = static_cast<int32_t>(min_time_ms);
}

// needs impl
void QuanScheduler::register_timer_process(AP_HAL::MemberProc k)
{}

//"not supported on AVR"
void QuanScheduler::register_io_process(AP_HAL::MemberProc k)
{}

// TODO register a function to call on failsafe
void QuanScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

// NEEDS impl
 // 
void QuanScheduler::suspend_timer_procs()
{}

// needs impl
void QuanScheduler::resume_timer_procs()
{}

// 
bool QuanScheduler::in_timerprocess() 
{
    return false;
}

//void QuanScheduler::begin_atomic()
//{}
//
//void QuanScheduler::end_atomic()
//{}

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

// TODO
void QuanScheduler::reboot(bool hold_in_bootloader) 
{
    for(;;);
}
