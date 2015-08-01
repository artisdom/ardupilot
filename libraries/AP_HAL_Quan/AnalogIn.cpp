#include "AnalogIn.h"

using namespace Quan;

QuanAnalogSource::QuanAnalogSource(float v) :
    _v(v)
{}

float QuanAnalogSource::read_average() {
    return _v;
}

float QuanAnalogSource::voltage_average() {
    return 5.0f * _v / 1024.0f;
}

float QuanAnalogSource::voltage_latest() {
    return 5.0f * _v / 1024.0f;
}

float QuanAnalogSource::read_latest() {
    return _v;
}

void QuanAnalogSource::set_pin(uint8_t p)
{}

void QuanAnalogSource::set_stop_pin(uint8_t p)
{}

void QuanAnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

QuanAnalogIn::QuanAnalogIn()
{}

void QuanAnalogIn::init(void* machtnichts)
{}

AP_HAL::AnalogSource* QuanAnalogIn::channel(int16_t n) {
    return new QuanAnalogSource(1.11);
}

float QuanAnalogIn::board_voltage(void)
{
    return 0;
}
