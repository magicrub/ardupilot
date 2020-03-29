#pragma once

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
    Open Drone ID, an implementation of the Remote ID system
    https://www.opendroneid.org/
    http://github.com/opendroneid/

    Tom Pittenger, April 2020
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_OpenDroneID {
public:
    // constructor
    AP_OpenDroneID();

    /* Do not allow copies */
    AP_OpenDroneID(const AP_OpenDroneID &other) = delete;
    AP_OpenDroneID &operator=(const AP_OpenDroneID&) = delete;

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update(void);

    // get singleton instance
    static AP_OpenDroneID *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_OpenDroneID *_singleton;

    void init();
    void deinit();

    AP_Int8     _enabled;
    bool        _initialized;

    void send_basic_id(const mavlink_channel_t chan);
    void send_location(const mavlink_channel_t chan);
    void send_authentication(const mavlink_channel_t chan);
    void send_self_id(const mavlink_channel_t chan);
    void send_system(const mavlink_channel_t chan);
    void send_operator_id(const mavlink_channel_t chan);
    void send_message_pack(const mavlink_channel_t chan);

    const static uint16_t interval_static_ms   = 3000;
    const static uint16_t interval_dynamic_ms  = 333;

    struct odid_basic_id_t {
        mavlink_open_drone_id_basic_id_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint16_t interval_max_ms = interval_static_ms;
    } _odid_basic_id;

    struct odid_location_t {
        mavlink_open_drone_id_location_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint32_t interval_max_ms = interval_dynamic_ms;
    } _odid_location;

    struct odid_authentication_t {
        mavlink_open_drone_id_authentication_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint32_t interval_max_ms = interval_static_ms;
    } _odid_authentication;

    struct odid_self_id_t {
        mavlink_open_drone_id_self_id_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint32_t interval_max_ms = interval_static_ms;
    } _odid_self_id;

    struct odid_system_t {
        mavlink_open_drone_id_system_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint32_t interval_max_ms = interval_static_ms;
    } _odid_system;

    struct odid_operator_id_t {
        mavlink_open_drone_id_operator_id_t info; // the whole mavlink struct with all the juicy details.
        uint32_t last_send_ms[MAVLINK_COMM_NUM_BUFFERS]; // last time this was transmitted
        const uint32_t interval_max_ms = interval_static_ms;
    } _odid_operator_id;

};

namespace AP {
    AP_OpenDroneID *OpenDroneID();
};
