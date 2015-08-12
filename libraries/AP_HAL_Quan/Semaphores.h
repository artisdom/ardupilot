
#ifndef __AP_HAL_QUAN_SEMAPHORE_H__
#define __AP_HAL_QUAN_SEMAPHORE_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>

class Quan::QuanSemaphore : public AP_HAL::Semaphore {
public:
    QuanSemaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    bool _taken;
};

#endif // __AP_HAL_QUAN_SEMAPHORE_H__
