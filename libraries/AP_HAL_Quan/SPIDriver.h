
#ifndef __AP_HAL_QUAN_SPIDRIVER_H__
#define __AP_HAL_QUAN_SPIDRIVER_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include "Semaphores.h"



class Quan::QuanSPIDeviceManager final : public AP_HAL::SPIDeviceManager {
public:
    QuanSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);

};

#endif // __AP_HAL_QUAN_SPIDRIVER_H__
