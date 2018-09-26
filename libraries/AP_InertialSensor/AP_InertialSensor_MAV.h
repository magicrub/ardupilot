/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_MAV : public AP_InertialSensor_Backend {
public:
    AP_InertialSensor_MAV(AP_InertialSensor &imu);

    bool update() override;

    void accumulate() override;
    void start() override;

    void handle_mavlink_values(const uint16_t msg_id, Vector3f &accel, Vector3f &gyro) override;

    // detect the sensor
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &_imu);

private:
    bool init_sensor(void);

    void intake_imu_data(Vector3f &accel, Vector3f &gyro);

    const enum Rotation _rotation = ROTATION_NONE;

    bool started;
    uint32_t last_mavlink_packet_ms;
    uint8_t _accel_instance;
    uint8_t _gyro_instance;
};

