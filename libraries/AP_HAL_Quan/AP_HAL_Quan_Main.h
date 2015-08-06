
#ifndef __AP_HAL_QUAN_MAIN_H__
#define __AP_HAL_QUAN_MAIN_H__

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp> 
#define AP_HAL_MAIN() \
extern "C" {\
int main (void) \
{\
   osd_setup(); \
   hal.init(0, NULL); \
   setup(); \
   hal.scheduler->system_initialized(); \
   create_draw_task(); \
   vTaskStartScheduler (); \
   return 0; \
}\
}\
\
void quan::uav::osd::on_draw() \
{ \
   loop();\
   quan::uav::osd::draw_text("Hello APM World",{-100,0});\
}

#endif // HAL_BOARD_QUAN

#endif // __AP_HAL_QUAN_MAIN_H__
