
#ifndef __AP_HAL_QUAN_NAMESPACE_H__
#define __AP_HAL_QUAN_NAMESPACE_H__

/* While not strictly required, names inside the Quan namespace are prefixed
 * with Quan for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Quan {
    //class QuanUARTDriver;
    class QuanI2CDriver;
    class QuanSPIDeviceManager;
    class QuanSPIDeviceDriver;
    class QuanAnalogSource;
    class QuanAnalogIn;
    class QuanStorage;
    class QuanGPIO;
    class QuanDigitalSource;
    class QuanRCInput;
    class QuanRCOutput;
    class QuanSemaphore;
    class QuanScheduler;
    class QuanUtil;
    class QuanPrivateMember;
}

#endif // __AP_HAL_QUAN_NAMESPACE_H__

