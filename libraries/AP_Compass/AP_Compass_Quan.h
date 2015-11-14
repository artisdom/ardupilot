#ifndef AP_COMPASS_QUAN_H_INCLUDED
#define AP_COMPASS_QUAN_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_HAL_Quan/i2c_task.hpp>

class AP_Compass_Quan : public AP_Compass_Backend
{
public:
   bool        init(void);
   void        read(void);

   AP_Compass_Quan(Compass &compass);
   static AP_Compass_Backend *detect(Compass &compass);

 private:
   QueueHandle_t m_hQueue;
   uint8_t       m_instance;
};

#endif //CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // AP_COMPASS_QUAN_H_INCLUDED
