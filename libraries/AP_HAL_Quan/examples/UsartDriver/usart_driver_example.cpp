#if 0
#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
// for pin number defines
//#include <AP_Notify.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Math.h>

#include <AP_HAL_Quan.h>
//#include <AP_HAL_SITL.h>
//#include <AP_HAL_Empty.h>
#else

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
// for pin number defines
//#include <AP_Notify.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>

#include <AP_HAL_Quan/AP_HAL_Quan.h>

#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// called once in apm_task before loop
void setup() 
{
   hal.gpio->pinMode(1,HAL_GPIO_OUTPUT);
   hal.gpio->write(1,1);

   const char text[] = "Hi from Apm Quan Scheduler\n";
	hal.console->write((uint8_t const*)text,strlen(text));
}

// called forever
void loop() 
{

}

AP_HAL_MAIN();

