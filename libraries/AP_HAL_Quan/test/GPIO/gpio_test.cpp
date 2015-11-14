
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
   constexpr uint8_t orange_led =  2U;
   constexpr uint8_t green_led = 3U;

   constexpr uint8_t led_off = 0U;
   constexpr uint8_t led_on = 1U;

   constexpr char text[] = "Quan APM GPIO test\n";
}

// called once at startup of apm task
void setup() 
{
   // test gpio
   for ( uint8_t i = 1; i < 4; ++i){
      hal.gpio->pinMode(i,HAL_GPIO_OUTPUT);
      hal.gpio->write(i,led_off);
   }

	hal.console->write((uint8_t const*)text,strlen(text));

}

void quan::uav::osd::on_draw() 
{ 
   quan::uav::osd::draw_text(text,{-140,50}); 
}

namespace{
   TickType_t prev_wake_time= 0; 

   uint32_t red_count = 0;
   uint32_t green_count = 0;
   uint32_t orange_count = 0;
}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,1); 

   if ( ++red_count == 200 ){
      red_count = 0;
      hal.gpio->toggle(heartbeat_led);
   } 
   if (++orange_count == 330){
      orange_count = 0;
      hal.gpio->toggle(orange_led);
   }

   if ( ++green_count == 500){
      green_count = 0;
      hal.gpio->toggle(green_led);
   }
}

AP_HAL_MAIN();

