
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "HAL_Quan_Class.h"
#include "AP_HAL_Quan_Private.h"

#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Private.h>

using namespace Quan;


namespace Quan{
  void init_spi();
  template <uint32_t I>
  AP_HAL::UARTDriver*  get_serial_port();
  AP_HAL::AnalogIn*    get_analog_in();
  AP_HAL::I2CDriver*   get_i2c_driver();
  AP_HAL::RCInput*     get_rc_inputs();
  AP_HAL::RCOutput*    get_rc_outputs();
}

static Empty::EmptySPIDeviceManager spiDeviceManager;
static QuanStorage storageDriver;
static QuanGPIO gpioDriver;
//static QuanRCOutput rcoutDriver;
static QuanScheduler schedulerInstance;
static QuanUtil utilInstance;

HAL_Quan::HAL_Quan() 
:AP_HAL::HAL(
   Quan::get_serial_port<0>(),//   console ( hal.uartA)
   Quan::get_serial_port<1>(),//   1st GPS
   Quan::get_serial_port<2>(),//   telemetry
   NULL,            /* no uartD */
   NULL,            /* no uartE */
   Quan::get_i2c_driver(), // dummy
   NULL, /* only one i2c */
   NULL, /* only one i2c */
   &spiDeviceManager, // dummy
   Quan::get_analog_in(),
   &storageDriver,
   Quan::get_serial_port<0>(),    // console  member
   &gpioDriver,
   Quan::get_rc_inputs(),
   Quan::get_rc_outputs(),
   &schedulerInstance,
   &utilInstance
)
{}

// called as first item at the startup of apm_task before the main forever loop
void HAL_Quan::init(int argc,char* const argv[]) const 
{
   uartA->begin(115200);
   gpio->init();
   Quan::init_spi();
   rcin->init(NULL);
   rcout->init(NULL);
   analogin->init(NULL);
   i2c->begin();
   spi->init(NULL);  // this is a dummy
   scheduler->init(NULL); // start i2c_task
}

const HAL_Quan AP_HAL_Quan;

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
