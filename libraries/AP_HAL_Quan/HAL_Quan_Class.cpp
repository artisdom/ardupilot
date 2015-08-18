
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "HAL_Quan_Class.h"
#include "AP_HAL_Quan_Private.h"

#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Private.h>

using namespace Quan;

//static QuanUARTDriver uartADriver;
//static QuanUARTDriver uartBDriver;
//static QuanUARTDriver uartCDriver;

namespace Quan{
  template <uint32_t I>
  AP_HAL::UARTDriver * get_serial_port();

  AP_HAL::AnalogIn* get_analog_in();
  AP_HAL::I2CDriver * get_i2c_driver();
}
//static QuanSemaphore  i2cSemaphore;
//static QuanI2CDriver  i2cDriver(&i2cSemaphore);
static QuanSPIDeviceManager spiDeviceManager;
//static QuanAnalogIn analogIn;
static QuanStorage storageDriver;
static QuanGPIO gpioDriver;
static QuanRCInput rcinDriver;
static QuanRCOutput rcoutDriver;
static QuanScheduler schedulerInstance;
static QuanUtil utilInstance;


HAL_Quan::HAL_Quan() :
    AP_HAL::HAL(
        Quan::get_serial_port<0>(),//   console ( hal.uartA)
        Quan::get_serial_port<1>(),//   1st GPS
        Quan::get_serial_port<2>(),//   telemetry
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        Quan::get_i2c_driver(),
        NULL, /* only one i2c */
        NULL, /* only one i2c */
        &spiDeviceManager,
        Quan::get_analog_in(),
        &storageDriver,
        Quan::get_serial_port<0>(),    // console  member
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance),
    _member(new QuanPrivateMember(123))
{
  
}

// called in APM in
void HAL_Quan::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.*/
    uartA->begin(115200);
    gpio->init();
    analogin->init(NULL);

    scheduler->init(NULL);
    
    _member->init();
}

const HAL_Quan AP_HAL_Quan;

#endif
