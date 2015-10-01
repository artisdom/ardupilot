#ifndef AP_BARO_QUAN_H_INCLUDED
#define AP_BARO_QUAN_H_INCLUDED

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

#endif // AP_BARO_QUAN_H_INCLUDED
