
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp>
#include <task.h>

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
   Quan::get_serial_port<2>(),//   1st GPS ( on hack board)
   Quan::get_serial_port<1>(),//   telemetry
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

void AP_HAL::init() {}

// ignore callbacks for now
void HAL_Quan::run(int argc, char * const argv[], Callbacks* callbacks) const
{
   this->init(argc,argv);
}

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

namespace {
   const HAL_Quan hal_quan;
}

const AP_HAL::HAL& AP_HAL::get_HAL() 
{
    return hal_quan;
}

void setup();
void loop();

namespace { 
   char dummy_param = 0; 
   TaskHandle_t task_handle = NULL; 
   void apm_task(void * params) 
   { 
      hal_quan.run(0, NULL,NULL);
      setup();
      hal_quan.scheduler->system_initialized(); 
      for(;;){ 
         loop(); 
      } 

   } 
} 

void create_apm_task() 
{ 
  xTaskCreate( 
      apm_task,"apm task", 
      4000, 
      &dummy_param, 
      tskIDLE_PRIORITY + 1, 
      &task_handle 
  ); 
}



#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
