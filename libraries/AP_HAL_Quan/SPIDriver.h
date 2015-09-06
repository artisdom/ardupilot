
#ifndef __AP_HAL_QUAN_SPIDRIVER_H__
#define __AP_HAL_QUAN_SPIDRIVER_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include "Semaphores.h"

class Quan::QuanSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    QuanSPIDeviceDriver();
    void init();
    AP_HAL::Semaphore* get_semaphore();
    bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    QuanSemaphore _semaphore;
};

class Quan::QuanSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    QuanSPIDeviceManager();
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice);
private:
    QuanSPIDeviceDriver _device;
};

#endif // __AP_HAL_QUAN_SPIDRIVER_H__
