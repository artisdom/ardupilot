
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "HAL_Quan_Class.h"
#include "AP_HAL_Quan_Private.h"

#include <AP_HAL_Quan.h>
#include <AP_HAL_Quan_Private.h>

using namespace Quan;

//static QuanUARTDriver uartADriver;
//static QuanUARTDriver uartBDriver;
//static QuanUARTDriver uartCDriver;

namespace Quan{
  AP_HAL::UARTDriver * get_serial_port(uint32_t i);
}
static QuanSemaphore  i2cSemaphore;
static QuanI2CDriver  i2cDriver(&i2cSemaphore);
static QuanSPIDeviceManager spiDeviceManager;
static QuanAnalogIn analogIn;
static QuanStorage storageDriver;
static QuanGPIO gpioDriver;
static QuanRCInput rcinDriver;
static QuanRCOutput rcoutDriver;
static QuanScheduler schedulerInstance;
static QuanUtil utilInstance;



HAL_Quan::HAL_Quan() :
    AP_HAL::HAL(
        Quan::get_serial_port(0),  //       &uartADriver,
        Quan::get_serial_port(1),//        &uartBDriver,
        Quan::get_serial_port(2),//        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        NULL, /* only one i2c */
        NULL, /* only one i2c */
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        Quan::get_serial_port(0),
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance),
    _member(new QuanPrivateMember(123))
{
  
}

void HAL_Quan::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    uartA->begin(115200);
    _member->init();
}

const HAL_Quan AP_HAL_Quan;

#endif
