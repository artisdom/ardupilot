#ifndef AP_BARO_QUAN_H_INCLUDED
#define AP_BARO_QUAN_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Baro.h"
#include <AP_HAL_Quan/i2c_task.hpp>

class AP_Baro_Quan : public AP_Baro_Backend
{
public:
    AP_Baro_Quan(AP_Baro &);
    void update();
private:
    QueueHandle_t const                 m_hQueue;
    uint8_t const                       m_instance;
   // make non_copyable?
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // AP_BARO_QUAN_H_INCLUDED
