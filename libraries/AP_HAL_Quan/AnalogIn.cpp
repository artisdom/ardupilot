
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <semphr.h>
#include "AnalogIn.h"
#include <Filter/Filter.h>
#include <Filter/Butter.h>
#include <atomic>
#include <quan/min.hpp>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/push_pop_fp.hpp>


/*

*/

extern const AP_HAL::HAL& hal;

//using namespace Quan;
/*
APM analog inputs detecteed


g.rssi_pin
airspeed
voltage
current
ANALOG_INPUT_NONE

 pin number 254 == vcc whether 3.3 V or 5v?
 pin number 255 == ANALOG_INPUT_NONE



function pins 
board_voltage
servo_rail_voltage


PC0   ADC123_IN10
PC1   ADC123_IN11
PC3   ADC123_IN13
PC4   ADC123_IN13

Use Regular Group Conversion
Use DMA to a memory array

Timer Trigger

use TIM8_CH1 or TIM8_TRGO ( can be update of TIM8)

Frequency 100 Hz? need an irq or task to recalc filters etcHi
*/

// timer8
#if 0
// test timer interrupt
namespace{
      int32_t adc_count =0;
}
void adc_timer_fun()
{

   if ( ++ adc_count == 12){
      adc_count = 0;
      hal.gpio->toggle(1);
   }

}
#endif

namespace {
   
   typedef quan::stm32::tim8 adc_timer;

   // raw A2D values updated by DMA
   // N.B. last index is a dummy to trap out of range
   volatile uint16_t adc_results [4 +1] = {0,0,0,0,0};

   typedef quan::mcu::pin<quan::stm32::gpioc,0> analog_pin1;
   typedef quan::mcu::pin<quan::stm32::gpioc,1> analog_pin2;
   typedef quan::mcu::pin<quan::stm32::gpioc,3> analog_pin3;
   typedef quan::mcu::pin<quan::stm32::gpioc,4> analog_pin4;

   template <typename Pin>
   void setup_adc_pin()
   {
      quan::stm32::module_enable<typename Pin::port_type>();
      quan::stm32::apply<
         Pin
         ,quan::stm32::gpio::mode::analog
         ,quan::stm32::gpio::pupd::none
      >();
   };

   // setup to overflow at 100 Hz
   // setup TRGO --> ADC on overflow
   void setup_adc_timer()
   {

      setup_adc_pin<analog_pin1>();
      setup_adc_pin<analog_pin2>();
      setup_adc_pin<analog_pin3>();
      setup_adc_pin<analog_pin4>();

      quan::stm32::module_enable<adc_timer>();

      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<adc_timer>();
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      // presc to 1 MHz
      adc_timer::get()->cr1 = 0;
      adc_timer::get()->psc = psc;
      adc_timer::get()->arr = 10000U-1U;  // 100Hz overflow
      adc_timer::get()->cnt = 0;
      adc_timer::get()->sr = 0;
#if 0
      adc_timer::get()->dier.setbit<0>();
#endif
      // trgo
      {
         quan::stm32::tim::cr2_t cr2 = adc_timer::get()->cr2.get();
         cr2.mms = 0b010; // TRGO is update, used to start an ADC conversion sequence
         adc_timer::get()->cr2.set(cr2.value);
      }

      // setup adc
      // All Analog pins available on channels ADC1 & ADC2 so
      // use say ADC1 for the moment

      // enable  and reset the adc1 module
      quan::stm32::rcc::get()->apb2enr |= (1 << 8); //( ADC1)
      quan::stm32::rcc::get()->apb2rstr |= ( 1 << 8 ); // (ADC1)
      quan::stm32::rcc::get()->apb2rstr &= ~( 1 << 8 );// (ADC1)

      // set adc clk to 21 MHz
      // PCLK == sysclk/2 == 84 MHz
      // so set /4 --> 21 MHz
      // but maybe *2 so set /8
      // TODO verify which it is!
      ADC->CCR |= (0b11 << 16);

      ADC1->CR1 &= ~(0b11 << 24); // 12 bit conversion
       //          
      // sequence  ADC123_IN10,ADC123_IN11,ADC123_IN1,ADC12_IN14
       // == ch10, ch11, ch13, ch14
      constexpr uint32_t num_channels = 4;
      constexpr uint32_t sati_bits = 0b011; // reg sampling time bits
       // individaul channel timings all the same
      ADC1->SMPR1 |= ( sati_bits << 0) | ( sati_bits << 3) | ( sati_bits << 9) | ( sati_bits << 12);
      ADC1->SQR1 |= ((num_channels-1) << 20); // ( L ) conversion sequence length
      // channel conversion sequence
      ADC1->SQR3 |= (10 << 0) | ( 11 << 5) | ( 13 << 10) | ( 14 << 15);
      ADC1->CR1  |= (1 << 11);  // (DISCEN) 
      ADC1->CR1  |= ((num_channels-1) << 13); // (DISCNUM)
      ADC1->CR2  |= (0b01 << 28); // (EXTEN) external trigger rising edge
      ADC1->CR2  |= (0b1110 << 24) ;// (EXTSEL) TIM8 TRGO
      ADC1->CR2  |= (1 << 8); //  (DMA) enable DMA
      ADC1->CR2  |= (1 << 9); //  (DDS) dma request continue sfter all done in sequence
      // set up regular channels
#if 1
      ADC1->CR1  |= ( 1<< 5); // (EOCIE)
#endif
      // ADC1 DMA ------------------------------------------------------------------------
      // USE DMA2.Stream4.Channel0  
      // Enable and reset DMA2 module
      quan::stm32::rcc::get()->ahb1enr |= (1 << 22);
      for ( uint8_t i = 0; i < 20; ++i){
         asm volatile ("nop" : : :);
      }
      quan::stm32::rcc::get()->ahb1rstr |= (1 << 22);
      quan::stm32::rcc::get()->ahb1rstr &= ~(1 << 22);

      // DMA setup
      // Do 4 16 bit transfers
      // med priority
      // 16 bit
      DMA_Stream_TypeDef * dma_stream = DMA2_Stream4;
      constexpr uint32_t  dma_channel = 0;
      constexpr uint8_t dma_priority = 0b01; // medium
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 25)) | ( dma_channel << 25); //(CHSEL) select channel
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 16)) | (dma_priority << 16); // (PL) priority
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) | (01 << 13); // (MSIZE) 16 bit memory transfer
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) | (01 << 11); // (PSIZE) 16 bit transfer
      dma_stream->CR |= (1 << 10);// (MINC)
      dma_stream->CR |= (1 << 8);// (CIRC)
      dma_stream->CR &= ~(0b11 << 6) ; // (DIR ) peripheral to memory
      dma_stream->CR |= ( 1 << 4) ; // (TCIE)
     
      dma_stream->FCR |= (1 << 2) ;// (DMDIS)
   // set threshold full
      dma_stream->FCR |= (0b11 << 0);
   // setup periph_reg
      dma_stream->PAR = (uint32_t)&ADC1->DR;  // periph addr
      dma_stream->M0AR = (uint32_t)adc_results; 
      dma_stream->NDTR = 4;
      // start the shebang...
      // enable adc
      ADC1->CR2 |= (1 << 0); // (ADON)
      // enable dma
      // NVIC irq etc
      NVIC_SetPriority(DMA2_Stream4_IRQn,14);  // low prio
      NVIC_EnableIRQ(DMA2_Stream4_IRQn);

      NVIC_SetPriority(ADC_IRQn,14);  // low prio
      NVIC_EnableIRQ(ADC_IRQn);
      DMA2->HIFCR |= (0b111101 << 0) ; // clear flags for Dma2 Stream 4
      DMA2->HIFCR &= ~(0b111101 << 0) ; // flags for Dma2 Stream 4
       // enable DMA
     // DMA2_Stream4->CR |= (1 << 0); // (EN)

   }
   void start_adc_timer()
   {
      // enable timer
      adc_timer::get()->cr1.bb_setbit<0>(); //(CEN)
   }

   SemaphoreHandle_t adc_semaphore;

   BaseType_t higherPriorityTaskWoken = 0;
 
}

#if 1
extern "C" void ADC_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

// test adc interrupt
namespace{
      int32_t adc_count =0;
}
void adc_irq_fun()
{
   if ( ++ adc_count == 75){
      adc_count = 0;
      hal.gpio->toggle(1);
   }
}

extern "C" void ADC_IRQHandler()
{
      if ( ADC1->SR & ( 1 << 5) ){ // (EOC)
          ADC1->SR &= ~( 1 << 5);
          adc_irq_fun();
      }
}
#endif

      // TODO add ADC Overrun interrupt
      // clear and reset adc and dma

// adc dma interrupt
extern "C" void DMA2_Stream4_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

extern "C" void DMA2_Stream4_IRQHandler() 
{   
   DMA2->HIFCR |= (0b111101 << 0) ; // clear flags for Dma2 Stream 4
   DMA2->HIFCR &= ~(0b111101 << 0) ; // flags for Dma2 Stream 4
   DMA2_Stream4->M0AR = (uint32_t)adc_results;
   DMA2_Stream4->CR |= (1 << 0); // (EN)

  
   xSemaphoreGiveFromISR(adc_semaphore,&higherPriorityTaskWoken );
}

namespace {

   typedef Filter<float> filter;

   struct dummy_filter_t : public Filter<float>{
      float apply(float sample){ return 0.f;}
      void reset(){;}
   };

   struct simple_filter_t : public Filter<float>{
       float apply(float sample){ return sample;}
       void reset(){;}
   };

   template <typename Coefficients>
   struct butterworth_filter_t : public Filter<float>{
      butterworth_filter_t() {m_filter.reset(1.65f);}
      float apply(float sample){ return m_filter.filter(sample);}
      void reset(){;}
      Butter2<Coefficients> m_filter;  
   };

   typedef butterworth_filter_t<butter100_4_coeffs> airspeed_adc_t;
   typedef butterworth_filter_t<butter100_8_coeffs> batt_current_adc_t;

   simple_filter_t      batt_voltage_adc;
   batt_current_adc_t   batt_current_adc;
   airspeed_adc_t       airspeed_adc;
   simple_filter_t      rssi_adc;
   dummy_filter_t       dummy_filter;
   
   filter* filters[5] = { 
      &batt_voltage_adc,
      &batt_current_adc,
      &airspeed_adc,
      &rssi_adc,
      &dummy_filter
   };

   float raw_adc_voltages[5] {0.f,0.f,0.f,0.f,0.f};
   float filtered_adc_values[5] = {0.f,0.f,0.f,0.f,0.f};

   void process_adc()
   {
      for (uint8_t i = 0; i < 4 ; ++i)
      {
         float const voltage = (adc_results[i] * 3.3f) / 4096;
         raw_adc_voltages[i] = voltage;
         filtered_adc_values[i]= filters[i]->apply(voltage);
      }
   };

   void adc_task( void * params){

      adc_semaphore = xSemaphoreCreateBinary();

      for (;;){
         xSemaphoreTake(adc_semaphore, portMAX_DELAY);
         process_adc();
      }
   }

   TaskHandle_t adc_task_handle;
   void * dummy_params;

   void create_adc_task()
   {
      xTaskCreate(
         adc_task,"adc_task",
         500,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
         & adc_task_handle
      ) ;
   }   

   template <uint8_t Pin>
   struct analog_source : public AP_HAL::AnalogSource{

      float voltage_latest(){return raw_adc_voltages[Pin];}
      float voltage_average() {return filtered_adc_values[Pin];}

      float voltage_average_ratiometric(){ return voltage_average();}
      // should be safe else
      // caller would have to know about impl
      float read_latest() {return voltage_latest();}
      float read_average() {return voltage_average();}
      // not available
      void set_pin(uint8_t p){}
      void set_stop_pin(uint8_t p){}
      void set_settle_time(uint16_t settle_time_ms){}
   };

   analog_source<0> battery_voltage;
   analog_source<1> battery_current;
   analog_source<2> airspeed;
   analog_source<3> rssi;
   analog_source<4> dummy;

   AP_HAL::AnalogSource* analog_sources[5] =
   {
     & battery_voltage
     ,& battery_current
     ,& airspeed
     ,& rssi
     ,& dummy
   };

   struct analog_in_t : public AP_HAL::AnalogIn{
      analog_in_t(){}
      void init(void* )
      {
         setup_adc_timer();
         create_adc_task();
         start_adc_timer();
      }

      AP_HAL::AnalogSource* channel(int16_t n) 
      {
         return analog_sources[quan::min(n,4)];
      }

      /*
         only sent to mavlink
      */
      float board_voltage(void)
      {
          return 3.3f;
      }
   } analog_in;

}// namespace 

namespace Quan{

  AP_HAL::AnalogIn* get_analog_in()
  {
    return & analog_in;
  }  

}
