#pragma once

#include "AP_Baro_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_Baro_MAV : public AP_Baro_Backend {
public:
    AP_Baro_MAV(AP_Baro &baro);

    void update() override;

    void handle_msg(const mavlink_message_t *msg) override;

private:
    uint8_t _instance;

    bool new_pressure;
    float _pressure;
    float _temperature;
    uint64_t _last_timestamp;

    HAL_Semaphore _sem_baro;
};
