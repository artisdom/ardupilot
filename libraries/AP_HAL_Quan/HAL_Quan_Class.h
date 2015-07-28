
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include <AP_HAL.h>

#include "AP_HAL_Quan_Namespace.h"
#include "PrivateMember.h"

class HAL_Quan : public AP_HAL::HAL {
public:
    HAL_Quan();
    void init(int argc, char * const * argv) const;
private:
    Quan::QuanPrivateMember *_member;
};

extern const HAL_Quan AP_HAL_Quan;

#endif // __AP_HAL_EMPTY_CLASS_H__

