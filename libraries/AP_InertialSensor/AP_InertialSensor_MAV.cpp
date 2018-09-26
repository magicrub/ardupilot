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

#include "AP_InertialSensor_MAV.h"
#include <GCS_MAVLink/GCS.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_MAV::AP_InertialSensor_MAV(AP_InertialSensor &imu)
    : AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_MAV::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_MAV *sensor = new AP_InertialSensor_MAV(_imu);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }

    sensor->_id = HAL_INS_MAV;

    return sensor;
}

bool AP_InertialSensor_MAV::init_sensor(void)
{
    return true;
}

void AP_InertialSensor_MAV::accumulate()
{
    gcs().update();
}

void AP_InertialSensor_MAV::start()
{
    const uint32_t dev_id_accel = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MAV, 0, 1, DEVTYPE_MAVLINK);
    const uint32_t dev_id_gyro  = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MAV, 0, 2, DEVTYPE_MAVLINK);

    // dev_id_accel == 2818309
    // dev_id_gyro  == 2818565
    _accel_instance = _imu.register_accel(250, dev_id_accel);
    _gyro_instance = _imu.register_gyro(250, dev_id_gyro);

    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    started = true;
}

bool AP_InertialSensor_MAV::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

void AP_InertialSensor_MAV::handle_mavlink_values(const uint16_t msg_id, Vector3f &accel, Vector3f &gyro)
{
    uint32_t now_ms = AP_HAL::millis();

#if 0
    static uint32_t debug1_last_ms = 0;
    static uint32_t samples1_per_sec = 0;
    samples1_per_sec++;
    if (now_ms - debug1_last_ms >= 1000) {
        debug1_last_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_DEBUG,"%d RX INS_MAV: %.3f, %.3f, %.3f", now_ms,
            #if 0
                (double)msg_id, (double)AP::ins().get_accel_rate_hz(_accel_instance), (double)samples1_per_sec,
            #elif 1
                (double)accel.x, (double)accel.y, (double)accel.z);
            #else
                (double)gyro.x, (double)gyro.y, (double)gyro.z);
            #endif
        samples1_per_sec = 0;
    }
#endif

    switch (msg_id) {
//    case MAVLINK_MSG_ID_RAW_IMU:
//        _msg_timestamp_ms[INS_MAV_PRIMARY_INDEX_RAW_IMU] = now_ms;
//        is_primary = (_primary == INS_MAV_PRIMARY_INDEX_RAW_IMU);
//
//        // undefined and therefore unknown units. However, when we transmit
//        // this packet we use the same units as MAVLINK_MSG_ID_SCALED_IMU
//        accel *= (GRAVITY_MSS / 1000.0f);   // convert milli-G to m/s/s
//        gyro *= 0.001f;                     // convert milli-rad/sec to rad/sec
//        break;

    case MAVLINK_MSG_ID_SCALED_IMU:
//        _msg_timestamp_ms[INS_MAV_PRIMARY_INDEX_SCALED_IMU1] = now_ms;
//        is_primary = (_primary == INS_MAV_PRIMARY_INDEX_SCALED_IMU1);

        accel *= (GRAVITY_MSS / 1000.0f);   // convert milli-G to m/s/s
        gyro *= 0.001f;                     // convert milli-rad/sec to rad/sec
        break;

//    case MAVLINK_MSG_ID_SCALED_IMU2:
//        _msg_timestamp_ms[INS_MAV_PRIMARY_INDEX_SCALED_IMU2] = now_ms;
//        is_primary = (_primary == INS_MAV_PRIMARY_INDEX_SCALED_IMU2);
//
//        accel *= (GRAVITY_MSS / 1000.0f);   // convert milli-G to m/s/s
//        gyro *= 0.001f;                     // convert milli-rad/sec to rad/sec
//        break;
//
//    case MAVLINK_MSG_ID_SCALED_IMU3:
//        _msg_timestamp_ms[INS_MAV_PRIMARY_INDEX_SCALED_IMU3] = now_ms;
//        is_primary = (_primary == INS_MAV_PRIMARY_INDEX_SCALED_IMU3);
//
//        accel *= (GRAVITY_MSS / 1000.0f);   // convert milli-G to m/s/s
//        gyro *= 0.001f;                     // convert milli-rad/sec to rad/sec
//        break;
//
    default:
        return;
    }

#if 0
    static uint32_t debug2_last_ms = 0;
    static uint32_t msg2_timestamp_ms = 0;
    static uint32_t samples2_per_sec = 0;
    (void)msg2_timestamp_ms; // touch it in case it's #if0'd out to make the compiler happy when it's unused
    (void)samples2_per_sec;
    samples2_per_sec++;
    msg2_timestamp_ms = now_ms;
    if (now_ms - debug2_last_ms >= 1000) {
        debug2_last_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_DEBUG,"%d RX INS_MAV: %.3f, %.3f, %.3f", now_ms,
            #if 0
                (double)msg_id, (double)AP::ins().get_accel_rate_hz(_accel_instance), (double)samples2_per_sec);
            #elif 1
                (double)accel.x, (double)accel.y, (double)accel.z);
            #else
                (double)gyro.x, (double)gyro.y, (double)gyro.z);
            #endif
        samples2_per_sec = 0;
    }
#endif

    last_mavlink_packet_ms = now_ms;
    intake_imu_data(accel, gyro);
}

void AP_InertialSensor_MAV::intake_imu_data(Vector3f &accel, Vector3f &gyro)
{
    if (!started) {
        return;
    }
    uint64_t now_us = AP_HAL::micros64();

    _rotate_and_correct_accel(_accel_instance, accel);
    _notify_new_accel_raw_sample(_accel_instance, accel, now_us);

    _rotate_and_correct_gyro(_gyro_instance, gyro);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro, now_us);
}
