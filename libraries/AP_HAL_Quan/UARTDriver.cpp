
#include "UARTDriver.h"
#include <quan/stm32/freertos/apm/freertos_usart_task.hpp>

using namespace Quan;

namespace {

   struct sp1{
      typedef quan::stm32::usart1                    usart;
      typedef quan::mcu::pin<quan::stm32::gpioa,9>   txo_pin;
      typedef quan::mcu::pin<quan::stm32::gpioa,10>  rxi_pin;
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };

   struct sp2{
      typedef quan::stm32::usart3                     usart;
      typedef quan::mcu::pin<quan::stm32::gpiob,10>   txo_pin; 
      typedef quan::mcu::pin<quan::stm32::gpiob,11>   rxi_pin; 
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };
   
   struct sp3{
      typedef quan::stm32::uart4                     usart;
      typedef quan::mcu::pin<quan::stm32::gpioa,0>   txo_pin; 
      typedef quan::mcu::pin<quan::stm32::gpioa,1>   rxi_pin; 
      typedef quan::stm32::freertos::apm::usart_tx_rx_task<usart,txo_pin,rxi_pin> serial_port;
   };

   template <typename SerialPort>
   class QuanUARTDriver : public AP_HAL::UARTDriver {
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
         return static_cast<uint16_t>(SerialPort::in_avail());
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

AP_HAL::UARTDriver * Quan::get_serial_port(uint32_t i)
{
  return serial_ports[i];
}


  


