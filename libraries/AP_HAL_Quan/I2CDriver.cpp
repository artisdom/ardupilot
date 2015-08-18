
#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"
#include <quan/stm32/freertos/freertos_i2c_task.hpp>
#include <quan/malloc_free.hpp>

using namespace Quan;

/*
it appears that APM address in is to be shifted left 1
 then if its a write address ored with 1
 (See the AP_HALAVR::AVRI2CDriver source)
*/

namespace {

   typedef quan::mcu::pin<quan::stm32::gpioa,8> i2c3_scl;
   typedef quan::mcu::pin<quan::stm32::gpioc,9> i2c3_sda;
   //typedef quan::stm32::i2c3  i2c_task;
   typedef quan::stm32::freertos::freertos_i2c_task<
      quan::stm32::i2c3,i2c3_scl,i2c3_sda
    > i2c3_task;
} 
// TODO add for each I2C port
extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler()
{     
   static_assert(std::is_same<i2c3_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
   i2c3_task::handle_irq();
}

extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler()
{
   static_assert(std::is_same<i2c3_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
   uint32_t const sr1 = i2c3_task::i2c_type::get()->sr1.get();
   i2c3_task::i2c_type::get()->sr1.set(sr1 & 0xFF); 
   i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler;
}

namespace {

   template <typename I2CPort>
   struct QuanI2CDriver : public AP_HAL::UARTDriver {
      typedef I2CPort i2c_port;
      void begin() 
      {
         // TODO start in high speed when have some error handling
         i2c_port::init(false,false); 
          // todo init the semaphore
      }

      // TODO 
      void end()
      {
        // disable the i2c port
      }

      void setTimeout(uint16_t ms)
      { 
         i2c_port::set_max_addr_wait_time(quan::time_<int32_t>::ms{ms});
      }

      // TODO 
      void setHighSpeed(bool active) 
      {

      }

      uint8_t write(uint8_t addr, uint8_t len, uint8_t* data)
      {
        // check not busy etc
         // setup write address
        if( i2c_port::transfer_request( addr << 1, data, len) == true){
            while (i2c_port::busy()){
               // check timeout?
               vTaskDelay(1);
            }
            return 0;
        }else{
            // get error number
            // do diagnostic to console
            // try a reset?
            return 1;
        }
      } 

      uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
      { 
         uint8_t const data[2] = {reg,val};
         return write( addr,2,data );
      }

      uint8_t writeRegisters(uint8_t addr, uint8_t reg,
      uint8_t len, uint8_t* data_in)
      {  
         uint8_t * data_out = (uint8_t*)quan::malloc(len + 1);
         data_out[0] = reg;
         for ( uint32_t i = 1; i <= len; ++i){
            data_out[i] = data_in[i-1];
         }
         uint8_t const result = write(addr,len+1,data_out);
         quan::free(data_out);
         return result;
      }

      uint8_t read(uint8_t addr, uint8_t len, uint8_t* data)
      {
         if( i2c_port::transfer_request( (addr << 1) | 1U, data, len) == true){
            while (i2c_port::busy()){
               // check timeout?
               vTaskDelay(1);
            }
            return 0;
        }else{
            // get error number
            // do diagnostic to console
            // try a reset?
            return 1;
        }
         memset(data, 0, len);
         return 0;
      }

      uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data_in)
      {
         uint8_t const reg_out = reg;
         if ( ( write(addr,1,reg_out) == 0 ) && ( read(addr,1,data_in) == 0 ) ){
            return 0;
         }else{
            return 1;
         }
      }

      uint8_t readRegisters(uint8_t addr, uint8_t reg,
           uint8_t len, uint8_t* data_in)
      {
         uint8_t const reg_out = reg;
         if ( ( write(addr,1,reg_out) == 0 ) && ( read(addr,len,data_in) == 0 ) ){
            return 0;
         }else{
            return 1;
         }
      }

      uint8_t lockup_count() {return 0;}
      void ignore_errors(bool b) { ; }
      AP_HAL::Semaphore* get_semaphore() {return nullptr;}

   };

}// namespace

namespace Quan{
    AP_HAL::I2CDriver * get_i2c_driver()
    { return nullptr;}
}


///
