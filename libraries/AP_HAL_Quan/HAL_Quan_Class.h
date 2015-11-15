
#ifndef __AP_HAL_QUAN_CLASS_H__
#define __AP_HAL_QUAN_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Quan_Namespace.h"

class HAL_Quan final : public AP_HAL::HAL {
public:
    HAL_Quan();

    void run(int argc, char * const argv[], Callbacks* callbacks) const;
private:
    void init(int argc, char * const * argv) const;
};

//extern const HAL_Quan AP_HAL_Quan;

//extern const AP_HAL::HAL& hal;

#endif // __AP_HAL_QUAN_CLASS_H__

