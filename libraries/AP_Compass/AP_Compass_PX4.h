/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_PX4 : public AP_Compass_Backend
{
public:
    bool        init(void);
    void        read(void);
    void        accumulate(void);

    AP_Compass_PX4(Compass &compass, uint8_t idx);

private:
    static uint8_t  _num_sensors;

    int      _mag_fd;
    Vector3f _sum;
    uint32_t _count;
    uint64_t _last_timestamp;
};
