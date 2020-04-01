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
#include <AP_Vehicle/AP_Vehicle.h>

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
 * Initialization
 */
void AP_OpenDroneID::init(void)
{
    _odid_basic_id.info = {};
    _odid_location.info = {};
    _odid_authentication.info = {};
    _odid_self_id.info = {};
    _odid_system.info = {};
    _odid_operator_id.info = {};

    // populate with internal dynamic or user-param data
    populate_basic_id();
    populate_location();
    populate_authentication();
    populate_self_id();
    populate_system();
    populate_operator_id();

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


    // if dynamiac data is set externally, don't self-populate unless it times out
    const uint32_t location_interval = _odid_location.set_externally ? _odid_location.interval_max_ms*2.5f : _odid_location.interval_max_ms*0.2f;
    if (now_ms - _odid_location.last_populate_ms >= location_interval) {
        _odid_location.set_externally = false;
        populate_location();
    }


    // populate all the fields faster than we normally send them to check if we need to set the has_changed flag to push an update sooner
//    if (now_ms - _odid_basic_id.last_populate_ms >= _odid_basic_id.interval_max_ms*.2f) {
//        populate_basic_id();
//    }
//    if (now_ms - _odid_location.last_populate_ms >= _odid_location.interval_max_ms*.2f) {
//        populate_location();
//    }
//    if (now_ms - _odid_authentication.last_populate_ms >= _odid_authentication.interval_max_ms*.2f) {
//        populate_authentication();
//    }
//    if (now_ms - _odid_self_id.last_populate_ms >= _odid_self_id.interval_max_ms*.2f) {
//        populate_self_id();
//    }
//    if (now_ms - _odid_system.last_populate_ms >= _odid_system.interval_max_ms*.2f) {
//        populate_system();
//    }
//    if (now_ms - _odid_operator_id.last_populate_ms >= _odid_operator_id.interval_max_ms*.2f) {
//        populate_operator_id();
//    }



    // check timers and send the data if new or expired
    for (uint8_t chan = 0; chan < MAVLINK_COMM_NUM_BUFFERS; chan++) {

        if (_odid_basic_id.has_changed || (now_ms - _odid_basic_id.last_send_ms[chan] >= _odid_basic_id.interval_max_ms)) {
            // in case the send fails on this given channel, clear the timestamp so on the next update tick it will still show as timed out and retry
            _odid_basic_id.last_send_ms[chan] = 0;
            send_basic_id((mavlink_channel_t)chan);
        }

        if (_odid_location.has_changed || (now_ms - _odid_location.last_send_ms[chan] >= _odid_location.interval_max_ms)) {
            _odid_location.last_send_ms[chan] = 0;
            send_location((mavlink_channel_t)chan);
        }

        if (_odid_authentication.has_changed || (now_ms - _odid_authentication.last_send_ms[chan] >= _odid_authentication.interval_max_ms)) {
            _odid_authentication.last_send_ms[chan] = 0;
            send_authentication((mavlink_channel_t)chan);
        }

        if (_odid_self_id.has_changed || (now_ms - _odid_self_id.last_send_ms[chan] >= _odid_self_id.interval_max_ms)) {
            _odid_self_id.last_send_ms[chan] = 0;
            send_self_id((mavlink_channel_t)chan);
        }

        if (_odid_system.has_changed || (now_ms - _odid_system.last_send_ms[chan] >= _odid_system.interval_max_ms)) {
            _odid_system.last_send_ms[chan] = 0;
            send_system((mavlink_channel_t)chan);
        }

        if (_odid_operator_id.has_changed || (now_ms - _odid_operator_id.last_send_ms[chan] >= _odid_operator_id.interval_max_ms)) {
            _odid_operator_id.last_send_ms[chan] = 0;
            send_operator_id((mavlink_channel_t)chan);
        }
    }

    _odid_basic_id.has_changed = false;
    _odid_location.has_changed = false;
    _odid_authentication.has_changed = false;
    _odid_self_id.has_changed = false;
    _odid_system.has_changed = false;
    _odid_operator_id.has_changed = false;
}

void AP_OpenDroneID::populate_basic_id(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_set_basic_id_t packet {};
    mavlink_msg_open_drone_id_set_basic_id_decode(&msg, &packet);

    memcpy(&_odid_basic_id.info, &packet, sizeof(_odid_basic_id.info));

    _odid_basic_id.has_changed = true;
    _odid_basic_id.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::populate_basic_id()
{
//    <field type="uint8_t" name="id_type" enum="MAV_ODID_ID_TYPE">Indicates the format for the uas_id field of this message.</field>
//    <field type="uint8_t" name="ua_type" enum="MAV_ODID_UA_TYPE">Indicates the type of UA (Unmanned Aircraft).</field>
//    <field type="uint8_t[20]" name="uas_id">UAS (Unmanned Aircraft System) ID following the format specified by id_type. Shall be filled with nulls in the unused portion of the field.</field>
//
//    MAV_ODID_ID_TYPE_NONE
//    MAV_ODID_ID_TYPE_SERIAL_NUMBER // Manufacturer Serial Number (ANSI/CTA-2063 format)
//    MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID //CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]
//    MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID


    _odid_basic_id.info.id_type = MAV_ODID_ID_TYPE_NONE;
    _odid_basic_id.info.ua_type = MAV_ODID_UA_TYPE_NONE;

    const char* uas_id = "ArduPilot Test";

    memset(&_odid_basic_id.info.uas_id, 0, sizeof(_odid_basic_id.info.uas_id));
    memcpy(_odid_basic_id.info.uas_id, uas_id, sizeof(*uas_id));

    _odid_basic_id.has_changed = true;
    _odid_basic_id.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::send_basic_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_BASIC_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_basic_id_send_struct(chan, &_odid_basic_id.info);
    _odid_basic_id.last_send_ms[chan] = AP_HAL::millis();
}


void AP_OpenDroneID::populate_location(const mavlink_message_t &msg)
{

}
void AP_OpenDroneID::populate_location()
{
    AP_AHRS &ahrs = AP::ahrs();
    AP_GPS &gps = AP::gps();
    AP_Baro &baro = AP::baro();

    mavlink_open_drone_id_location_t msg {};

    // back-up existing
    memcpy(&msg, &_odid_location.info, sizeof(_odid_location.info));

    const AP_Vehicle *vehicle2 = AP_Vehicle::get_singleton();
    if (gcs().vehicle_initialised() && vehicle2 != nullptr) {
        _odid_location.info.status = vehicle2->get_likely_flying() ? MAV_ODID_STATUS_AIRBORNE : MAV_ODID_STATUS_GROUND;
    } else {
        _odid_location.info.status = MAV_ODID_STATUS_UNDECLARED;
    }


    // <field type="uint16_t" name="direction" units="cdeg">Direction over ground (not heading, but direction of movement) measured clockwise from true North: 0 - 35999 centi-degrees. If unknown: 36100 centi-degrees.</field>
    // <field type="uint16_t" name="speed_horizontal" units="cm/s">Ground speed. Positive only. If unknown: 25500 cm/s. If speed is larger than 25425 cm/s, use 25425 cm/s.</field>
    Vector2f gndVel = ahrs.groundspeed_vector();
    if (!gndVel.is_zero()) {
        // TODO: confirm 0 here means NORTH and not FORWARD.
        _odid_location.info.direction           = gndVel.angle() * 0.01f;    // deg -> cd
        _odid_location.info.speed_horizontal    = constrain_float(gndVel.length(), 0, 254.25f) * 100; // m/s -> cm/s
    } else {
        _odid_location.info.direction           = 36100;
        _odid_location.info.speed_horizontal    = 25500;
    }


    // <field type="int16_t" name="speed_vertical" units="cm/s">The vertical speed. Up is positive. If unknown: 6300 cm/s. If speed is larger than 6200 cm/s, use 6200 cm/s. If lower than -6200 cm/s, use -6200 cm/s.</field>
    Vector3f velNED;
    _odid_location.info.speed_vertical = ahrs.get_velocity_NED(velNED) ? constrain_float(velNED.z, -62.0f, 62.0f) * -100 : 6300;   // m -> cm plus invert (NED to positive up)


    Location current_loc;
    if (ahrs.get_position(current_loc)) {
        _odid_location.info.latitude            = current_loc.lat;
        _odid_location.info.longitude           = current_loc.lng;
        _odid_location.info.altitude_geodetic   = current_loc.alt * 0.01; // cm -> m
        _odid_location.info.height              = ahrs.home_is_set() ? (float)(current_loc.alt - ahrs.get_home().alt) * 0.01f : -1000;  // cm -> m
    } else {
        _odid_location.info.latitude            = 0;
        _odid_location.info.longitude           = 0;
        _odid_location.info.altitude_geodetic   = -1000;
        _odid_location.info.height              = -1000;
    }


    // Altitude difference between sea level pressure and current pressure. Result in meters
    const bool bara_is_healthy  = baro.healthy();
    _odid_location.info.altitude_barometric     = bara_is_healthy ? baro.get_altitude_difference(SSL_AIR_PRESSURE, baro.get_pressure())                 : -1000;
    _odid_location.info.barometer_accuracy      = bara_is_healthy ? (MAV_ODID_VER_ACC)opendroneid::encodeVerticalAccuracy(baro.get_vertical_accuracy()) : MAV_ODID_VER_ACC_UNKNOWN;


    float value_f;
    _odid_location.info.vertical_accuracy       = gps.vertical_accuracy(value_f)    ? (MAV_ODID_VER_ACC)opendroneid::encodeVerticalAccuracy(value_f)    : MAV_ODID_VER_ACC_UNKNOWN;
    _odid_location.info.horizontal_accuracy     = gps.horizontal_accuracy(value_f)  ? (MAV_ODID_HOR_ACC)opendroneid::encodeHorizontalAccuracy(value_f)  : MAV_ODID_HOR_ACC_UNKNOWN;
    _odid_location.info.speed_accuracy          = gps.speed_accuracy(value_f)       ? (MAV_ODID_SPEED_ACC)opendroneid::encodeSpeedAccuracy(value_f)     : MAV_ODID_SPEED_ACC_UNKNOWN;


    // TODO
    _odid_location.info.height_reference        = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;

    const uint16_t timestamp_interval = gps.get_rate_ms();
    const bool gps_is_healthy = gps.is_healthy();
    if (gps_is_healthy && timestamp_interval > 0 && timestamp_interval <= 1500) {
        // the enum is 0.1s == 1, 0.2 == 2, .. 1.5s == 15. So, we can easily calculate it instead of doing a long if/switch.
        // examples: 50ms -> 1, 250ms -> 3
        _odid_location.info.timestamp_accuracy = constrain_int16(timestamp_interval * 0.01f, MAV_ODID_TIME_ACC_0_1_SECOND, MAV_ODID_TIME_ACC_1_5_SECOND);
    } else {
        _odid_location.info.timestamp_accuracy  = MAV_ODID_TIME_ACC_UNKNOWN;
    }

    // make sure to do the comparison before the timestamp because that will likely always be different
    // note, the backup at the beginning copied the timestamp to msg so they are populated and will always match here
    if (memcmp(&msg, &_odid_location.info, sizeof(_odid_location.info)) != 0) {
        _odid_location.has_changed = true;
        _odid_location.last_populate_ms = AP_HAL::millis();
    }

    _odid_location.info.timestamp = gps_is_healthy ? ((float) (gps.time_week_ms() % (60*60*1000))) * 0.001f : 0;
}


void AP_OpenDroneID::send_location(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_LOCATION)) {
        return;
    }
    mavlink_msg_open_drone_id_location_send_struct(chan, &_odid_location.info);
    _odid_location.last_send_ms[chan] = AP_HAL::millis();
}



void AP_OpenDroneID::populate_authentication(const mavlink_message_t &msg)
{
//    memcpy(&_odid_authentication.info, msg, sizeof(_odid_authentication.info));

    _odid_authentication.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::populate_authentication()
{

}
void AP_OpenDroneID::send_authentication(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_AUTHENTICATION)) {
        return;
    }
    mavlink_msg_open_drone_id_authentication_send_struct(chan, &_odid_authentication.info);
    _odid_authentication.last_send_ms[chan] = AP_HAL::millis();
}


void AP_OpenDroneID::populate_self_id(const mavlink_message_t &msg)
{
}
void AP_OpenDroneID::populate_self_id()
{
    _odid_self_id.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::send_self_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SELF_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_self_id_send_struct(chan, &_odid_self_id.info);
    _odid_self_id.last_send_ms[chan] = AP_HAL::millis();
}


void AP_OpenDroneID::populate_system()
{
    _odid_system.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::send_system(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SYSTEM)) {
        return;
    }
    mavlink_msg_open_drone_id_system_send_struct(chan, &_odid_system.info);
    _odid_system.last_send_ms[chan] = AP_HAL::millis();
}


void AP_OpenDroneID::populate_operator_id()
{
    _odid_operator_id.last_populate_ms = AP_HAL::millis();
}
void AP_OpenDroneID::send_operator_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_OPERATOR_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_operator_id_send_struct(chan, &_odid_operator_id.info);
    _odid_operator_id.last_send_ms[chan] = AP_HAL::millis();
}

MAV_RESULT AP_OpenDroneID::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    // we only handle the _SET_ msgs
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_BASIC_ID:
        populate_basic_id(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_LOCATION:
        populate_location(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_AUTHENTICATION:
        populate_authentication(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_SELF_ID:
        populate_self_id(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_SYSTEM:
        populate_system(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_OPERATOR_ID:
        populate_operator_id(msg);
        break;

    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
    default:
        // status messages are unhandled
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

AP_OpenDroneID *AP::OpenDroneID()
{
    return AP_OpenDroneID::get_singleton();
}
