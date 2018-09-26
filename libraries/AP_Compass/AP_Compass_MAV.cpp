/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_MAV.cpp - MAVLink backend for AP_Compass
 *
 */


#include "AP_Compass_MAV.h"
#include <GCS_MAVLink/GCS.h>


// constructor
AP_Compass_MAV::AP_Compass_MAV()
{
    _instance = register_compass();
    set_rotation(_instance, ROTATION_NONE);
    set_external(_instance, true);

    // COMPASS_DEV_ID for MAVLink is typically 1048581 == 0x00100005
    set_dev_id(_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_MAV, 0, 0, DEVTYPE_MAV));
}


void AP_Compass_MAV::handle_mavlink_values(const uint16_t msg_id, Vector3f &mag)
{
    switch (msg_id) {
//    case MAVLINK_MSG_ID_RAW_IMU:
//        _msg_timestamp_ms[MAG_MAV_PRIMARY_INDEX_RAW_IMU] = now_ms;
//        is_primary = (_primary == MAG_MAV_PRIMARY_INDEX_RAW_IMU);
//        // undefined and therefore unknown units. However, when we transmit
//        // this packet we use the same units as MAVLINK_MSG_ID_SCALED_IMU
//        break;

    case MAVLINK_MSG_ID_SCALED_IMU:
//        _msg_timestamp_ms[MAG_MAV_PRIMARY_INDEX_SCALED_IMU1] = now_ms;
//        is_primary = (_primary == MAG_MAV_PRIMARY_INDEX_SCALED_IMU1);
        break;

//    case MAVLINK_MSG_ID_SCALED_IMU2:
//        _msg_timestamp_ms[MAG_MAV_PRIMARY_INDEX_SCALED_IMU2] = now_ms;
//        is_primary = (_primary == MAG_MAV_PRIMARY_INDEX_SCALED_IMU2);
//        break;
//
//    case MAVLINK_MSG_ID_SCALED_IMU3:
//        _msg_timestamp_ms[MAG_MAV_PRIMARY_INDEX_SCALED_IMU3] = now_ms;
//        is_primary = (_primary == MAG_MAV_PRIMARY_INDEX_SCALED_IMU3);
//        break;

    default:
        return;
    }

#if 0
    static uint32_t debug_last_ms = 0;
    if (AP_HAL::millis() - debug_last_ms >= 1000) {
        debug_last_ms = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_DEBUG,"%d MAG_MAV (%2d): %.1f, %.1f, %.1f", AP_HAL::millis(), msg_id, (double)mag.x, (double)mag.y, (double)mag.z);
    }
#endif

    intake_mag_data(mag);
}

void AP_Compass_MAV::intake_mag_data(Vector3f &mag)
{
    // rotate raw_field from sensor frame to body frame
    rotate_field(mag, _instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(mag, _instance);

    // correct raw_field for known errors
    correct_field(mag, _instance);

    WITH_SEMAPHORE(_sem);
    // accumulate into averaging filter
    _sum += mag;
    _count++;
}

void AP_Compass_MAV::read()
{
    // avoid division by zero if we haven't received any mag reports
    if (_count == 0) {
        return;
    }

//    Vector3f mag = (_sum / _count);
//    gcs().send_text(MAV_SEVERITY_DEBUG, "%d: %.1f, %.1f, %.1f", AP_HAL::millis(), (double)mag.x, (double)mag.y, (double)mag.z);

    WITH_SEMAPHORE(_sem);
    _sum /= _count;

    publish_filtered_field(_sum, _instance);

    _sum.zero();
    _count = 0;
}
