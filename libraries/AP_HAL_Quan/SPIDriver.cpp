
#include "SPIDriver.h"

using namespace Quan;

QuanSPIDeviceDriver::QuanSPIDeviceDriver()
{}

void QuanSPIDeviceDriver::init()
{}

AP_HAL::Semaphore* QuanSPIDeviceDriver::get_semaphore()
{
    return &_semaphore;
}

void QuanSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
{}


void QuanSPIDeviceDriver::cs_assert()
{}

void QuanSPIDeviceDriver::cs_release()
{}

uint8_t QuanSPIDeviceDriver::transfer (uint8_t data)
{
    return 0;
}

void QuanSPIDeviceDriver::transfer (const uint8_t *data, uint16_t len)
{
}

QuanSPIDeviceManager::QuanSPIDeviceManager()
{}

void QuanSPIDeviceManager::init(void *)
{}

AP_HAL::SPIDeviceDriver* QuanSPIDeviceManager::device(enum AP_HAL::SPIDevice)
{
    return &_device;
}

