

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>

#include "RCOutput.h"
/*
 #define RC_OUTPUT_MIN_PULSEWIDTH 400
#define RC_OUTPUT_MAX_PULSEWIDTH 2100
*/


namespace {

   // tim4 ch1 to 4 used for rc out
   // tim4 : 16bit @ 84 MHz
   typedef quan::stm32::tim4 rc_out_timer;
   // use PWM on all channels
   typedef quan::mcu::pin<quan::stm32::gpiob,6> rc_out_ch1;
   typedef quan::mcu::pin<quan::stm32::gpiob,7> rc_out_ch2;
   typedef quan::mcu::pin<quan::stm32::gpiob,8> rc_out_ch3;
   typedef quan::mcu::pin<quan::stm32::gpiob,9> rc_out_ch4;

   constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<rc_out_timer>();
   constexpr uint32_t min_freq_hz = 50U;
   constexpr uint32_t max_freq_hz = 400U;

   constexpr uint16_t max_pulsewidth = 2100U;
   constexpr uint16_t min_pulsewidth = 400U;
   constexpr uint8_t num_channels = 4;

   void enable_pins()
   {
       quan::stm32::apply<
         rc_out_ch1,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();

      quan::stm32::apply<
         rc_out_ch2,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();

      quan::stm32::apply<
         rc_out_ch3,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();

      quan::stm32::apply<
         rc_out_ch4,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();
   }

   void rc_out_timer_setup()
   {
      quan::stm32::module_enable<quan::stm32::gpiob>();
      quan::stm32::module_enable<rc_out_timer>();
      enable_pins();
       
      // set all channels to pwm mode
      // period = 20 ms == 20,000 us
     // set 1 usec tick
      {
         // set all channels to PWM mode 1
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s = 0b00;  // output
         ccmr1.oc1m = 0b110; // PWM mode 1
         ccmr1.cc2s = 0b00;  // output
         ccmr1.oc2m = 0b110; // PWM mode 1
         rc_out_timer::get()->ccmr1.set(ccmr1.value);
      }
      {
         quan::stm32::tim::ccmr2_t ccmr2 = 0;
         ccmr2.cc3s = 0b00;  // output
         ccmr2.oc3m = 0b110; // PWM mode 1
         ccmr2.cc4s = 0b00;  // output
         ccmr2.oc4m = 0b110; // PWM mode 1
         rc_out_timer::get()->ccmr2.set(ccmr2.value);
      }
      {
         // default enabled or disabled
         quan::stm32::tim::ccer_t ccer = 0;
         ccer.cc1p =  false;
         ccer.cc1np = false;
         ccer.cc1e = true; // enable ch1 default?
         ccer.cc2p =  false;
         ccer.cc2np = false;
         ccer.cc2e = true; // enable ch2
         ccer.cc3p =  false;
         ccer.cc3np = false;
         ccer.cc3e = true; // enable ch3
         ccer.cc4p =  false;
         ccer.cc4np = false;
         ccer.cc4e = true; // enable ch4
         rc_out_timer::get()->ccer.set(ccer.value);
      }
      
      // set all ccr regs to center except throttle set low
      rc_out_timer::get()->ccr1 = 1500;
      rc_out_timer::get()->ccr2 = 1500;
      rc_out_timer::get()->ccr3 = 900;
      rc_out_timer::get()->ccr4 = 1500;

      // set the ocpe (preload) bits in ccmr1 , ccmr2 
      rc_out_timer::get()->ccmr1 |= (( 1<<3) | ( 1 << 11));
      rc_out_timer::get()->ccmr2 |= (( 1<<3) | ( 1 << 11));

      rc_out_timer::get()->psc = (timer_freq / 1000000 )-1;
      rc_out_timer::get()->arr = 20000 -1;
      rc_out_timer::get()->cnt = 0x0;
      {
         quan::stm32::tim::cr1_t cr1 = 0;
         cr1.arpe = true ;// auto preload
         rc_out_timer::get()->cr1.set(cr1.value);
      }
      
      rc_out_timer::get()->sr = 0;
   }

   void start_timer()
   {
      rc_out_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }
  
   struct rc_outputs_t : public AP_HAL::RCOutput {
      void     init(void*)
      {
         rc_out_timer_setup();
         start_timer();
      }
      // no op unless the mask is for all and only 4 channels
      void  set_freq(uint32_t chmask, uint16_t freq_hz)
      {
         if ((chmask == 0x0000000F) &&  (freq_hz >= min_freq_hz) && ( freq_hz <= max_freq_hz) ){
            rc_out_timer::get()->arr = (1000000U / freq_hz) -1 ;
         }
      }
      // all the same
      uint16_t get_freq(uint8_t ch)
      {
         return 1000000U / ( rc_out_timer::get()->arr + 1);
      }

      void enable_ch(uint8_t ch)
      {
         switch (ch){
            case 0:
               rc_out_timer::get()->ccmr1.bb_setbit<0>(); // (CC1E)
               return;
            case 1:
               rc_out_timer::get()->ccmr1.bb_setbit<4>(); // (CC2E)
               return;
            case 2:
               rc_out_timer::get()->ccmr1.bb_setbit<8>(); // (CC3E)
               return;
            case 3:
               rc_out_timer::get()->ccmr1.bb_setbit<12>(); // (CC4E)
               return;
            default:
               return;
         }
      }

      void disable_ch(uint8_t ch)
      {
         switch (ch){
            case 0:
               rc_out_timer::get()->ccmr1.bb_clearbit<0>(); // (CC1E)
               return;
            case 1:
               rc_out_timer::get()->ccmr1.bb_clearbit<4>(); // (CC2E)
               return;
            case 2:
               rc_out_timer::get()->ccmr1.bb_clearbit<8>(); // (CC3E)
               return;
            case 3:
               rc_out_timer::get()->ccmr1.bb_clearbit<12>(); // (CC4E)
               return;
            default:
               return;
         }
      }
     
      void write(uint8_t ch, uint16_t pulsewidth_in_us)
      {
         uint16_t const pulsewidth_us = quan::constrain(pulsewidth_in_us,min_pulsewidth, max_pulsewidth);

         switch (ch){
            case 0:
               rc_out_timer::get()->ccr1 = pulsewidth_us; 
               return;
            case 1:
               rc_out_timer::get()->ccr2 = pulsewidth_us; 
               return;
            case 2:
               rc_out_timer::get()->ccr3 = pulsewidth_us; 
               return;
            case 3:
               rc_out_timer::get()->ccr4 = pulsewidth_us; 
               return;
            default:
               return;
         }
      }

      void write(uint8_t ch, uint16_t* period_us, uint8_t len_in)
      {
         uint8_t const end = quan::min(ch + len_in,num_channels);
         for (uint8_t i = ch,period_idx = 0; i < end ; ++i, ++period_idx){
            this->write(i, period_us[period_idx]);
         }
      }
      
      
      uint16_t read(uint8_t ch)
      {
         switch (ch){
            case 0:
               return rc_out_timer::get()->ccr1; 
            case 1:
               return rc_out_timer::get()->ccr2;
            case 2:
               return rc_out_timer::get()->ccr3;
            case 3:
               return rc_out_timer::get()->ccr4;
            default:
               return 0U;
         }
      }

      void read(uint16_t* period_us, uint8_t len)
      {
          for (uint8_t i = 0U; i < len; ++i) {
            if ( i < num_channels){
               period_us[i] = this->read(i);
            }else{
               period_us[i] = 1500U;
            }
          }
      }

   } rc_outputs;

} // namespace 

namespace Quan{

   AP_HAL::RCOutput *  get_rc_outputs() 
   {
      return & rc_outputs;
   }
}



