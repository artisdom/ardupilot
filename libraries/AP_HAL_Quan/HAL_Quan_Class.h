
#ifndef __AP_HAL_QUAN_CLASS_H__
#define __AP_HAL_QUAN_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Quan_Namespace.h"

class HAL_Quan final : public AP_HAL::HAL {
public:
    HAL_Quan();
    void init(int argc, char * const * argv) const;
};

extern const HAL_Quan AP_HAL_Quan;

#endif // __AP_HAL_QUAN_CLASS_H__

