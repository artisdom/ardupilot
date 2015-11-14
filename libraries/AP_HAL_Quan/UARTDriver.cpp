

#include <quan/stm32/freertos/apm/freertos_usart_task.hpp>
#include <quan/stm32/gpio.hpp>
#include <resources.hpp>

#include "UARTDriver.h"

/*
   TODO needs a REDO
   can be local since all in anon namespace
   then can get panic error messages
   put the non template data and no template dep fns in a non template member struct
   of QuanUARTDriver<Usart>. Since it is not global should be fine to do
*/


/*
 Though technically Usart can be read/written by multiple threads
 In practise when things are working correctly
  Will be used by one thread
  use a mutex to ensure this?
 Look at serial manager
*/

using namespace Quan;
   
/*
At startup this is set to non inverting
*/
void Quan::set_usart3_tx_inverting( bool val)
{
    quan::stm32::put<frsky_txo_sign_pin>(val);
}

/*
TODO add thread safe version for console
*/

namespace {

   // on isolated port
   // Pin 4 RXI Pin 5 TXO
   struct sp1{
      typedef quan::stm32::usart1                    usart;
      typedef quan::mcu::pin<quan::stm32::gpioa,9>   txo_pin;
      typedef quan::mcu::pin<quan::stm32::gpioa,10>  rxi_pin;
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };

   // on isolated port
   // Pin 2 TXO , Pin 6 RXI
   struct sp2{
      typedef quan::stm32::usart3                     usart;
      typedef quan::mcu::pin<quan::stm32::gpiob,10>   txo_pin; 
      typedef quan::mcu::pin<quan::stm32::gpiob,11>   rxi_pin; 
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };
   
   // on header
   struct sp3{
      typedef quan::stm32::uart4                     usart;
      typedef quan::mcu::pin<quan::stm32::gpioa,0>   txo_pin; 
      typedef quan::mcu::pin<quan::stm32::gpioa,1>   rxi_pin; 
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };

   template <typename SerialPort>
   class QuanUARTDriver final : public AP_HAL::UARTDriver {
   public:
      void begin(uint32_t b, uint16_t rxbufsize, uint16_t txbufsize) 
      {
         SerialPort::begin(b, rxbufsize, txbufsize);
      }

      void begin(uint32_t b)
      {
         SerialPort::begin(b, 0U, 0U);
      }
       
      void end()
      {
         SerialPort::end();
      }

      void flush()
      {
         SerialPort::flush();
      }

      bool is_initialized()
      {
         return SerialPort::is_initialised();
      }

      void set_blocking_writes(bool blocking)
      {
         SerialPort::set_blocking(blocking);
      }

      int16_t available(void) 
      {
         return static_cast<int16_t>(SerialPort::in_avail());
      }
      
      bool tx_pending()
      {
         return SerialPort::out_waiting() != 0;
      }

      int16_t read()
      {
         return SerialPort::get();
      }

      size_t write(uint8_t c) 
      {
         return SerialPort::put(c);
      }

      size_t write(const uint8_t *buffer, size_t size)
      {
         return SerialPort::write(buffer,size);
      }

      int16_t txspace()
      {
         return SerialPort::out_spaces_available();
      }
   };

   QuanUARTDriver<sp1::serial_port> serial_port1;
   QuanUARTDriver<sp2::serial_port> serial_port2;
   QuanUARTDriver<sp3::serial_port> serial_port3;

   AP_HAL::UARTDriver * serial_ports [] ={
      &serial_port1
      ,&serial_port2
      ,&serial_port3
   };

}// namespace

template <uint32_t I>
AP_HAL::UARTDriver * Quan::get_serial_port()
{
  return serial_ports[I];
}

template AP_HAL::UARTDriver * Quan::get_serial_port<0>();
template AP_HAL::UARTDriver * Quan::get_serial_port<1>();
template AP_HAL::UARTDriver * Quan::get_serial_port<2>();

// interrupts

extern "C" void USART1_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void USART3_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void UART4_IRQHandler() __attribute__ ((interrupt ("IRQ")));

extern "C" void USART1_IRQHandler()
{

   static_assert(
   std::is_same<
      sp1::usart,quan::stm32::usart1
   >::value
   ,"invalid usart for serial_port irq");
   sp1::serial_port::irq_handler();
}

extern "C" void USART3_IRQHandler()
{
   static_assert(
   std::is_same<
      sp2::usart,quan::stm32::usart3
   >::value
   ,"invalid usart for serial_port irq");
   sp2::serial_port::irq_handler();
}

extern "C" void UART4_IRQHandler()
{
   static_assert(
   std::is_same<
      sp3::usart,quan::stm32::uart4
   >::value
   ,"invalid usart for serial_port irq");
   sp3::serial_port::irq_handler();
}
   


  


