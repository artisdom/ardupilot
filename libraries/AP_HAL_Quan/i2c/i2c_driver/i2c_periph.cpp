

#include "FreeRTOS.h"
#include <task.h>
//#include <semphr.h>
//#include <queue.h>
#include <stm32f4xx.h>
#include <quan/time.hpp>
#include <quan/frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/f4/i2c/module_enable_disable.hpp>
#include <quan/stm32/sys_freq.hpp>
#include <quan/stm32/i2c/detail/get_irq_number.hpp>
//#include <quan/stm32/millis.hpp>
//#include "../system/led.hpp"
//#include "../system/interrupt_priority.hpp"
//#include "../system/serial_port.hpp"
#include "i2c_periph.hpp"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace {
   typedef quan::mcu::pin<quan::stm32::gpioa,8> scl_pin;
   typedef quan::mcu::pin<quan::stm32::gpioc,9> sda_pin;
}

// token representing wthat i2c bus has been acquired
volatile bool Quan::i2c_periph::m_bus_taken_token = false;
volatile bool Quan::i2c_periph::m_errored = false;

void (* volatile Quan::i2c_periph::pfn_event_handler)()  = Quan::i2c_periph::default_event_handler;
void (* volatile Quan::i2c_periph::pfn_error_handler)()  = Quan::i2c_periph::default_error_handler;
void (* volatile Quan::i2c_periph::pfn_dma_tx_handler)()    = Quan::i2c_periph::default_dma_tx_handler;
void (* volatile Quan::i2c_periph::pfn_dma_rx_handler)()    = Quan::i2c_periph::default_dma_rx_handler;

namespace {

   typedef quan::stm32::tim5 usec_timer; // 32 bit timer 4 channels

   void setup_usec_timer()
   {
      quan::stm32::module_enable<usec_timer>();

      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<usec_timer>();
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");

      // use ch1 for i2c timing
      // set ch1 to compare
      {
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s = 0b00;   // channel is an output (compare)
         ccmr1.oc1fe = false; // no fast enable
         ccmr1.oc1pe = false; // no preload
         ccmr1.oc1m = 0b000; // output frozen timing base
         usec_timer::get()->ccmr1.set(ccmr1.value);
      }

      {
         quan::stm32::tim::ccer_t ccer = 0;
         ccer.cc1e = false; 
         usec_timer::get()->ccer.set(ccer.value);
      }

      {
         quan::stm32::tim::dier_t dier = 0;
         dier.cc1ie = false; // disable interrupt for now
         usec_timer::get()->dier.set(dier.value);
      }

      usec_timer::get()->psc = psc;
      usec_timer::get()->cnt = 0;
      usec_timer::get()->arr = 0xffffffff;
      usec_timer::get()->sr = 0;
      usec_timer::get()->dier.clearbit<0>(); //(UIE)  

      NVIC_SetPriority(TIM5_IRQn,14);
      NVIC_EnableIRQ(TIM5_IRQn);

      usec_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }

}

using AP_HAL::millis;

bool Quan::wait_for_i2c_bus_free(uint32_t t_ms)
{
   auto now = millis();
   while ( ! Quan::i2c_periph::bus_free() ){
      if ( (millis() - now) > t_ms){
         hal.console->printf("bus didnt free in %lu ms\n",t_ms);
         return false;
      }
   }
   return true;
}

/*
  set up the i2c bus
*/
void Quan::i2c_periph::init()
{
   setup_usec_timer();

   quan::stm32::module_enable<scl_pin::port_type>();
   quan::stm32::module_enable<sda_pin::port_type>();
   // TODO add check they are valid pins
   quan::stm32::apply<
      scl_pin
      ,quan::stm32::gpio::mode::af4 // all i2c pins are this af mode on F4
      ,quan::stm32::gpio::otype::open_drain
      ,quan::stm32::gpio::pupd::none         //  Use external pullup 5V tolerant pins
      ,quan::stm32::gpio::ospeed::slow
   >();

   quan::stm32::apply<
      sda_pin
      ,quan::stm32::gpio::mode::af4 // all i2c pins are this af mode on F4
      ,quan::stm32::gpio::otype::open_drain
      ,quan::stm32::gpio::pupd::none          //  Use external pullup 5V tolerant pins
      ,quan::stm32::gpio::ospeed::slow
   >();


   uint32_t count = 0;
   while ( count < 100){
      asm volatile ("nop":::);
      ++count;
   }

   quan::stm32::module_enable<i2c_type>();
   quan::stm32::module_reset<i2c_type>();

   // set the bus  speed
   // all i2c buses are on apb1?
   uint32_t constexpr apb1_freq = quan::stm32::get_bus_frequency<quan::stm32::detail::apb1>();
   static_assert(apb1_freq == 42000000,"unexpected freq");
   // set clock speed for 42 MHz APB1 bus
   uint32_t constexpr freq = apb1_freq / 1000000;
   static_assert(apb1_freq % 1000000 == 0, "invalid freq");
   uint32_t const temp_cr2 = i2c_type::get()->cr2.get() & ~0b111111;
   i2c_type::get()->cr2.set( temp_cr2 | freq);

   //set clock for slow freq
   quan::frequency_<int32_t>::Hz constexpr i2c_freq_slow{100000}; // 100 kHz
   uint32_t ccr_reg_val = apb1_freq / (i2c_freq_slow.numeric_value() * 2);

   uint32_t const temp_ccr = i2c_type::get()->ccr.get() & ~0xFFF;
   i2c_type::get()->ccr.set(temp_ccr | ccr_reg_val);

   constexpr quan::time::ns max_scl_risetime{1000};
   uint32_t constexpr trise_reg_val
      = static_cast<uint32_t>(max_scl_risetime * quan::frequency::Hz{apb1_freq} + 1.f);
   uint32_t const temp_trise = i2c_type::get()->trise.get() & ~0b111111;
   i2c_type::get()->trise.set(temp_trise | trise_reg_val);

   NVIC_EnableIRQ(quan::stm32::i2c::detail::get_event_irq_number<i2c_type>::value);
   NVIC_EnableIRQ(quan::stm32::i2c::detail::get_error_irq_number<i2c_type>::value);

   NVIC_SetPriority(quan::stm32::i2c::detail::get_event_irq_number<i2c_type>::value,tskIDLE_PRIORITY + 3);
#if defined QUAN_I2C_TX_DMA
   setup_tx_dma();
#endif
   setup_rx_dma();

   m_errored = false;
   release_bus(); 
   peripheral_enable(true);
}

#if defined QUAN_I2C_TX_DMA
// Set up transmit dma
void Quan::i2c_periph::setup_tx_dma()
{
   // i2c3 tx on DMA1_Stream4.CH3
   quan::stm32::rcc::get()->ahb1enr |= (1U << 21U); // DMA stream 1
//   quan::stm32::rcc::get()->ahb1rstr |= (1U << 21U);
//   quan::stm32::rcc::get()->ahb1rstr &= ~(1U << 21U);
   for ( uint8_t i = 0; i < 20; ++i){
      asm volatile ("nop" : : :);
   }
   DMA_Stream_TypeDef* dma_stream = DMA1_Stream4;
   constexpr uint32_t  dma_channel = 3;
   constexpr uint32_t  dma_priority = 0b00; // low
   constexpr uint32_t  msize = 0b00; // 8 bit mem loc
   constexpr uint32_t  psize = 0b00; // 8 bit periph loc
   dma_stream->CR = (dma_stream->CR & ~(0b111 << 25U)) | ( dma_channel << 25U); //(CHSEL) select channel
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 16U)) | (dma_priority << 16U); // (PL) priority
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 13U)) | (msize << 13U); // (MSIZE) 8 bit memory transfer
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 11U)) | (psize << 11U); // (PSIZE) 8 bit transfer
   dma_stream->CR |= (1 << 10);// (MINC)
   dma_stream->CR &= ~(1 << 9);// (PINC)
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 6U)) | (0b01 << 6U) ; // (DIR ) memory to peripheral
   dma_stream->CR |= ( 1 << 4) ; // (TCIE)
   dma_stream->PAR = (uint32_t)&I2C3->DR;  // periph addr
   NVIC_SetPriority(DMA1_Stream4_IRQn,15);  // low prio
   NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}
#endif
// Set up receive dma
void Quan::i2c_periph::setup_rx_dma()
{
   // dma1 module enable done in tx dma
#if !defined QUAN_I2C_TX_DMA
    quan::stm32::rcc::get()->ahb1enr |= (1U << 21U); // DMA stream 1
#endif
   DMA_Stream_TypeDef* dma_stream = DMA1_Stream2;
   constexpr uint32_t  dma_channel = 3;
   constexpr uint32_t  dma_priority = 0b00; // low
   constexpr uint32_t  msize = 0b00; // 8 bit mem loc
   constexpr uint32_t  psize = 0b00; // 8 bit periph loc
   dma_stream->CR = (dma_stream->CR & ~(0b111 << 25U)) | ( dma_channel << 25U); //(CHSEL) select channel
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 16U)) | (dma_priority << 16U); // (PL) priority
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 13U)) | (msize << 13U); // (MSIZE) 8 bit memory transfer
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 11U)) | (psize << 11U); // (PSIZE) 8 bit transfer
   dma_stream->CR |= (1 << 10);// (MINC)
   dma_stream->CR &= ~(1 << 9);// (PINC)
   dma_stream->CR = (dma_stream->CR & ~(0b11 << 6U))  ; // (DIR ) peripheral to memory
   dma_stream->CR |= ( 1 << 4) ; // (TCIE)
   dma_stream->PAR = (uint32_t)&I2C3->DR;  // periph addr
   NVIC_SetPriority(DMA1_Stream2_IRQn,15);  // low prio
   NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

const char* Quan::i2c_periph::get_error_string()
{
   return "error todo";
}

void Quan::i2c_periph::default_event_handler()
{
    AP_HAL::panic("i2c event default handler called");
}


// TODO. However for now I solved this by initing the pins before the module
/*
https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder \
=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex%5fmx%5fstm32%2f\
Proper%20Initialization%20of%20the%20I2C%20Peripheral&FolderCTID=0x01200200770978C69A1141439FE559EB\
459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=1824
Over the years I have encountered several I2C peripherals that don't always reset correctly on power up.
When I initialize the I2C controller, or after a timeout error, I disable the I2C on the STM32, put the I2C pins in GPIO open drain mode,
and then toggle the SCK pin (at < 100KHz) until the SDA pin shows the bus is free.  Then I enable the I2C on the STM32 and start a new transaction.
  Jack Peacock
*/

namespace {

   void delay_10usec()
   {
       // around 10 usec
       uint32_t count = 500;
       while (count > 0){
          -- count;
          asm volatile ("nop":::);
       }
   }

   bool clear_i2c_bus()
   {
      quan::stm32::apply<
         scl_pin
         ,quan::stm32::gpio::mode::output 
         ,quan::stm32::gpio::otype::open_drain
         ,quan::stm32::gpio::pupd::none         //  Use external pullup 5V tolerant pins
         ,quan::stm32::gpio::ospeed::slow
         ,quan::stm32::gpio::ostate::high
      >();

      quan::stm32::apply<
         sda_pin
         ,quan::stm32::gpio::mode::input 
         ,quan::stm32::gpio::otype::open_drain
         ,quan::stm32::gpio::pupd::none          //  Use external pullup 5V tolerant pins
         ,quan::stm32::gpio::ospeed::slow
      >();

      // sanity check
      // if bus not cleared in 100 clocks probably terminal
      int clock_count = 100;

      while (!quan::stm32::get<sda_pin>() && (clock_count > 0)){

         quan::stm32::clear<scl_pin>();
         delay_10usec();
         quan::stm32::set<scl_pin>();
         delay_10usec();
         -- clock_count;
      }

      if ( clock_count > 0){
         return true;
      }else{
         AP_HAL::panic("couldnt clear i2c bus");
         return false;
      }
   }

}

/*
  provide a 30 usec delay from time of call
  until the i2c bus is available
  Gives around 3 clocks at 100 kHz
  to allow for ack
*/
void Quan::i2c_periph::release_bus()
{
    __disable_irq();
   uint32_t const now = usec_timer::get()->cnt;
   uint32_t const done = (now + 30U) | 1;
   usec_timer::get()->ccr1 = done ;
   usec_timer::get()->sr.bb_clearbit<1>(); // (CC1IF)
   usec_timer::get()->dier.bb_setbit<1>(); // (CC1IE) 
   __enable_irq();
}

void Quan::i2c_periph::default_error_handler()
{
   hal.console->printf("i2c error handler called : ");

   NVIC_DisableIRQ(DMA1_Stream4_IRQn);
   NVIC_DisableIRQ(DMA1_Stream2_IRQn);
   enable_error_interrupts(false);
   enable_event_interrupts(false);
   enable_buffer_interrupts(false);

   // disable dma
#if defined QUAN_I2C_TX_DMA
   enable_dma_tx_stream(false);
#endif
   enable_dma_rx_stream(false);

   uint32_t const flags = get_sr1();
   bool flagged = false;
   // sr1 bit 8 Bus error
   if ( flags & ( 1 << 8)){ // (BERR)
     flagged = true;
     hal.console->printf("bus error");
   }
   if ( flags & ( 1 << 9)){  // (ARLO)
     flagged = true; 
     hal.console->printf("arbitration lost");
   }
   if ( flags & ( 1 << 10)){ // (AF)
     flagged = true;
     hal.console->printf("acknowledge failure");
   }
   if ( flagged == false){
     hal.console->printf("unknown error");
   }
   hal.console->printf("\n");
#if defined QUAN_I2C_TX_DMA
   clear_dma_tx_stream_flags();
#endif
   clear_dma_rx_stream_flags();

   quan::stm32::module_reset<i2c_type>();
   quan::stm32::module_disable<i2c_type>();

   clear_i2c_bus();
  
   m_errored = true;
}

#if defined QUAN_I2C_TX_DMA
void Quan::i2c_periph::default_dma_tx_handler()
{
    AP_HAL::panic("i2c dma tx def called");
}
#endif

void Quan::i2c_periph::default_dma_rx_handler()
{
    AP_HAL::panic("i2c dma rx def called");
}

#if defined QUAN_I2C_TX_DMA
// These irq handlers are aliased to the std stm32 irq handlers
template <> __attribute__ ((interrupt ("IRQ"))) void DMA_IRQ_Handler<1,4>()
{
     Quan::i2c_periph::pfn_dma_tx_handler();
}
#endif

template <> __attribute__ ((interrupt ("IRQ"))) void DMA_IRQ_Handler<1,2>()
{
     Quan::i2c_periph::pfn_dma_rx_handler();
}

#if defined QUAN_I2C_TX_DMA
// alias unmangled: void ::DMA_IRQ_Handler<1,4>() ;
extern "C" void DMA1_Stream4_IRQHandler() __attribute__ ((alias ("_Z15DMA_IRQ_HandlerILi1ELi4EEvv")));
#endif
// alias unmangled: void ::DMA_IRQ_Handler<1,2>() ;
extern "C" void DMA1_Stream2_IRQHandler() __attribute__ ((alias ("_Z15DMA_IRQ_HandlerILi1ELi2EEvv")));

extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler()
{
   Quan::i2c_periph::pfn_event_handler();
}

extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler()
{
   Quan::i2c_periph::pfn_error_handler();
}

extern "C" void TIM5_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void TIM5_IRQHandler()
{
    uint32_t const irq_en_flags = usec_timer::get()->dier.get();
    uint32_t const irq_flags = usec_timer::get()->sr.get(); 
    uint32_t const active_irqs = irq_flags & irq_en_flags;
     
    // ch 1 compare
    if ( active_irqs & ( 1 << 1 ) ) {
       usec_timer::get()->dier.bb_clearbit<1>(); // disable the irq
       usec_timer::get()->sr.bb_clearbit<1>(); // clear the irq
       Quan::i2c_periph::m_bus_taken_token = false;
    }
   //  usec_timer::get()->sr = 0;
}
