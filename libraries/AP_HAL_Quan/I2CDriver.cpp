
#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"
#include <quan/stm32/freertos/freertos_i2c_task.hpp>

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
    > i2c_task;
} 

extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler()
{     

   static_assert(std::is_same<i2c_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
   i2c_task::handle_irq();
}

extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler()
{
   static_assert(std::is_same<i2c_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
   uint32_t const sr1 = i2c_task::i2c_type::get()->sr1.get();
   i2c_task::i2c_type::get()->sr1.set(sr1 & 0xFF); 
   i2c_task::i2c_errno = i2c_task::errno_t::i2c_err_handler;
}

void QuanI2CDriver::begin() 
{
  // TODO start in high speed when have some error handling
  i2c_task::init(false,false); // will try 
}

// 
void QuanI2CDriver::end() {}
void QuanI2CDriver::setTimeout(uint16_t ms) {}
void QuanI2CDriver::setHighSpeed(bool active) {}

uint8_t QuanI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{return 1;} 

uint8_t QuanI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{return 1;}

uint8_t QuanI2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{return 1;}

uint8_t QuanI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
    memset(data, 0, len);
    return 0;
}
uint8_t QuanI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
    *data = 0;
    return 1;
}

uint8_t QuanI2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
    memset(data, 0, len);    
    return 1;
}

uint8_t QuanI2CDriver::lockup_count() {return 0;}
