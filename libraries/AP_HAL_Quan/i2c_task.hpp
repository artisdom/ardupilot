#ifndef APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED
#define APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED

#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <quan/pressure.hpp>
#include <quan/temperature.hpp>
#include <quan/time.hpp>
#include <quan/magnetic_flux_density.hpp>
#include <quan/three_d/vect.hpp>

#include <AP_HAL/AP_HAL.h>
#include "I2CDriver.h"

namespace Quan{ 

   void create_i2c_task();
   QueueHandle_t get_compass_queue_handle();
   QueueHandle_t get_baro_queue_handle();
   
   namespace detail{

      struct baro_args{
         quan::pressure::Pa         pressure;
         quan::temperature::K       temperature;
         int32_t                    update_time_ms;
      };

      struct compass_args{
         quan::three_d::vect<quan::magnetic_flux_density::milli_gauss> field;
         int32_t                                                       update_time_ms;
      };
   }
}

#endif // APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED
