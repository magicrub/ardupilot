#pragma once

#include "AP_Compass.h"
#include <AP_HAL/AP_HAL.h>


class AP_Compass_MAV : public AP_Compass_Backend
{
public:
    AP_Compass_MAV();
    void read(void);

    void handle_mavlink_values(const uint16_t msg_id, Vector3f &mag) override;

private:
    void intake_mag_data(Vector3f &mag);

    uint8_t _instance;
    Vector3f _sum;
    uint32_t _count;
};
