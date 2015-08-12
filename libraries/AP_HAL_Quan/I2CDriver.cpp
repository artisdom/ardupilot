
#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"

using namespace Quan;

void QuanI2CDriver::begin() {}
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
