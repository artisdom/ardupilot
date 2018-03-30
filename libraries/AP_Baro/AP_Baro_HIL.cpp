/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Baro.h"
#include "AP_Baro_HIL.h"

extern const AP_HAL::HAL& hal;

template<> AP_Baro_Backend * create_baro_driver<HALSITL::tag_board>(AP_Baro & baro)
{
   return new AP_Baro_HIL(baro);
}

AP_Baro_HIL::AP_Baro_HIL(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

// Read the sensor
void AP_Baro_HIL::update(void)
{
    float pressure = 0.0;
    float temperature = 0.0;
    float pressure_sum = 0.0;
    float temperature_sum = 0.0;
    uint32_t sum_count = 0;

    while (_frontend._hil.press_buffer.is_empty() == false){
        _frontend._hil.press_buffer.pop_front(pressure);
        pressure_sum += pressure; // Pressure in Pascals
        _frontend._hil.temp_buffer.pop_front(temperature);
        temperature_sum += temperature; // degrees celcius
        sum_count++;
    }

    if (sum_count > 0) {
        pressure_sum /= (float)sum_count;
        temperature_sum /= (float)sum_count;
        _copy_to_frontend(0, pressure_sum, temperature_sum);
    }
}

#endif
