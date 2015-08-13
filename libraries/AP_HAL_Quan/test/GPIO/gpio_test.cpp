
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
// for pin number defines
//#include <AP_Notify.h>
// but includes a lot of stuff then
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <task.h>
#include <cstring>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {
   constexpr uint8_t heartbeat_led = 1U;
   constexpr uint8_t led_off = 0U;
   constexpr uint8_t led_on = 1U;
}

// called once at startup of apm task
void setup() 
{
   // test gpio
   hal.gpio->pinMode(heartbeat_led,HAL_GPIO_OUTPUT);
   hal.gpio->write(heartbeat_led,led_off);

   const char text[] = "Quan APM GPIO test\n";

	hal.console->write((uint8_t const*)text,strlen(text));

}

namespace {
   // shared resource
   // shared between apm_task and draw_task
   // use taskENTER_CRITICAL to read/write
   // or mutex
   // could be a problem with apm code
   // will require mutex
   // variables of length 32 bits or less are ok though
   
   char buffer[20] = {'\0'};
}

void quan::uav::osd::on_draw() 
{ 
   // local atomic copy
   char buffer1[20];
   taskENTER_CRITICAL();
   strncpy(buffer1, buffer,19);
   taskEXIT_CRITICAL();
   buffer1[19] = '\0';
   quan::uav::osd::draw_text(buffer1,{-100,0}); 
}

namespace{
   TickType_t prev_wake_time= 0; 

}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,750); 
   hal.gpio->toggle(heartbeat_led);
   if ( hal.gpio->read(heartbeat_led) == 0){
      taskENTER_CRITICAL();
      strcpy(buffer,"Led Off");
      taskEXIT_CRITICAL();
   }else{
      taskENTER_CRITICAL();
      strcpy(buffer,"Led On");
      taskEXIT_CRITICAL();
   }
   
	hal.console->write((uint8_t const*)buffer,strlen(buffer));
   hal.console->write((uint8_t const)'\n');

}

AP_HAL_MAIN();

