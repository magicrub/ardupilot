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
    AP_OpenDroneID.cpp

    Open Drone ID, an implementation of the Remote ID system
    https://www.opendroneid.org/
    http://github.com/opendroneid/

    Tom Pittenger, April 2020
*/

#include "AP_OpenDroneID.h"
#include "opendroneid.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;
AP_OpenDroneID *AP_OpenDroneID::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable OpenDroneID Broadcast
    // @Description: Enable OpenDroneID Broadcast
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE",     0, AP_OpenDroneID, _enabled,    0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OpenDroneID must be singleton");
    }
    _singleton = this;
}

/*
 * periodic update
 */
void AP_OpenDroneID::update(void)
{
    if (!_enabled) {
        if (_initialized) {
            deinit();
        }
        return;
    }
    if (!_initialized) {
        init();
        return; // return in case we failed the init
    }

    const uint32_t now_ms = AP_HAL::millis();

    // service static fields
    for (uint8_t chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {

        if (now_ms - _odid_basic_id.last_send_ms[chan] >= _odid_basic_id.interval_max_ms) {
            send_basic_id((mavlink_channel_t)chan);
        }

        if (now_ms - _odid_authentication.last_send_ms[chan] >= _odid_authentication.interval_max_ms) {
            send_authentication((mavlink_channel_t)chan);
        }

        if (now_ms - _odid_self_id.last_send_ms[chan] >= _odid_self_id.interval_max_ms) {
            send_self_id((mavlink_channel_t)chan);
        }

        if (now_ms - _odid_system.last_send_ms[chan] >= _odid_system.interval_max_ms) {
            send_system((mavlink_channel_t)chan);
        }

        if (now_ms - _odid_operator_id.last_send_ms[chan] >= _odid_operator_id.interval_max_ms) {
            send_operator_id((mavlink_channel_t)chan);
        }
    }


    // check dynamic fields. If anything has changed, send immediately. Otherwise send at the interval max
    mavlink_open_drone_id_location_t msg;
    memcpy(&msg, &_odid_location.info, sizeof(_odid_location.info));

    //_odid_location.info.status = ?
    _odid_location.info.direction           = 0;
    _odid_location.info.speed_horizontal    = 0;
    _odid_location.info.speed_vertical      = 0;
    _odid_location.info.latitude            = 0;
    _odid_location.info.longitude           = 0;
    _odid_location.info.altitude_barometric = 0;
    _odid_location.info.altitude_geodetic   = 0;
    _odid_location.info.height_reference    = 0;
    _odid_location.info.height              = 0;
    _odid_location.info.horizontal_accuracy = 0;
    _odid_location.info.vertical_accuracy   = 0;
    _odid_location.info.barometer_accuracy  = 0;
    _odid_location.info.speed_accuracy      = 0;
//    _odid_location.info.timestamp           = 0;
//    _odid_location.info.timestamp_accuracy  = 0;

    const bool something_changed = memcmp(&msg, &_odid_location.info, sizeof(_odid_location.info)) != 0;
    for (uint8_t chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {

        if (something_changed) {
            // force a send, and make sure we send on next update tick if the mavlink send fails
            _odid_location.last_send_ms[chan] = 0;
        }
        if (now_ms - _odid_location.last_send_ms[chan] >= _odid_location.interval_max_ms) {
            send_location((mavlink_channel_t)chan);
        }
    }


//    for (uint8_t chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {
//
//        if (now_ms - _odid_basic_id.last_send_ms[chan] >= _odid_basic_id.interval_max_ms) {
//            send_basic_id((mavlink_channel_t)chan);
//        }
//
//        if (now_ms - _odid_location.last_send_ms[chan] >= _odid_location.interval_max_ms) {
//            send_basic_id((mavlink_channel_t)chan);
//        }
//    }
}

void AP_OpenDroneID::send_basic_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_BASIC_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_basic_id_send_struct(chan, &_odid_basic_id.info);
    _odid_basic_id.last_send_ms[chan] = AP_HAL::millis();
}

void AP_OpenDroneID::send_location(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_LOCATION)) {
        return;
    }
    mavlink_msg_open_drone_id_location_send_struct(chan, &_odid_location.info);
    _odid_location.last_send_ms[chan] = AP_HAL::millis();
}

void AP_OpenDroneID::send_authentication(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_AUTHENTICATION)) {
        return;
    }
    mavlink_msg_open_drone_id_authentication_send_struct(chan, &_odid_authentication.info);
    _odid_authentication.last_send_ms[chan] = AP_HAL::millis();
}

void AP_OpenDroneID::send_self_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SELF_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_self_id_send_struct(chan, &_odid_self_id.info);
    _odid_self_id.last_send_ms[chan] = AP_HAL::millis();
}

void AP_OpenDroneID::send_system(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SYSTEM)) {
        return;
    }
    mavlink_msg_open_drone_id_system_send_struct(chan, &_odid_system.info);
    _odid_system.last_send_ms[chan] = AP_HAL::millis();
}

void AP_OpenDroneID::send_operator_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_OPERATOR_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_operator_id_send_struct(chan, &_odid_operator_id.info);
    _odid_operator_id.last_send_ms[chan] = AP_HAL::millis();
}

/*
 * Initialization
 */
void AP_OpenDroneID::init(void)
{

    // randomize all last_send_ms values so they're all out of phase so we don't hammer the mavlink buffers
    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {
        _odid_basic_id.last_send_ms[chan]       = now_ms + (get_random16() % _odid_basic_id.interval_max_ms);
        _odid_location.last_send_ms[chan]       = now_ms + (get_random16() % _odid_location.interval_max_ms);
        _odid_authentication.last_send_ms[chan] = now_ms + (get_random16() % _odid_authentication.interval_max_ms);
        _odid_self_id.last_send_ms[chan]        = now_ms + (get_random16() % _odid_self_id.interval_max_ms);
        _odid_system.last_send_ms[chan]         = now_ms + (get_random16() % _odid_system.interval_max_ms);
        _odid_operator_id.last_send_ms[chan]    = now_ms + (get_random16() % _odid_operator_id.interval_max_ms);
    }

    _initialized = true;

//    if (!_initialized) {
//        // if we failed the init, disable the system so we don't constantly re-init and notify GCS
//        _enabled = false;
//    }
}

/*
 * De-Initialization
 */
void AP_OpenDroneID::deinit(void)
{
    _initialized = false;
}

AP_OpenDroneID *AP::OpenDroneID()
{
    return AP_OpenDroneID::get_singleton();
}

