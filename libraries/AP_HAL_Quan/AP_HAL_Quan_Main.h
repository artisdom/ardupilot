
#ifndef __AP_HAL_QUAN_MAIN_H__
#define __AP_HAL_QUAN_MAIN_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp>
#include <task.h>

/*
   use AVR main main function code but put in a task
*/
 
#define AP_HAL_MAIN() \
\
void create_apm_task();\
\
extern "C" {\
   int main (void) \
   {\
      osd_setup(); \
   \
      create_draw_task(); \
      create_apm_task(); \
      vTaskStartScheduler (); \
   }\
}\
\
namespace { \
   char dummy_param = 0; \
   TaskHandle_t task_handle = NULL; \
   void apm_task(void * params) \
   { \
      hal.init(0, NULL);\
      setup();\
      hal.scheduler->system_initialized(); \
      for(;;){ \
         loop(); \
      } \
\
   } \
} \
void create_apm_task() \
{ \
  xTaskCreate( \
      apm_task,"apm task", \
      4000, \
      &dummy_param, \
      tskIDLE_PRIORITY + 1, \
      &task_handle \
  ); \
}



/*
   hal.init inits quan.hal.scheduler
   const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
   AP_HAL_BOARD_DRIVER =  AP_HAL_Quan
   AP_HAL_Quan = 
  (AP_HAL/AP_HAL_Boards.h:267:#define AP_HAL_BOARD_DRIVER AP_HAL_Quan)
   Put apm scheduler in a task?
  in HAL_Quan_Class.h extern const HAL_Quan AP_HAL_Quan;

   loop is defined in ArduPlane/ArduPlane.cpp
   // waits for sample then loops
   
*/

#endif // HAL_BOARD_QUAN

#endif // __AP_HAL_QUAN_MAIN_H__
