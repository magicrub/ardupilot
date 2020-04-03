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
#include <StorageManager/StorageManager.h>

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
    // populate with internal dynamic or user-param data
    init_basic_id();
    populate_location();
    init_authentication();
    init_self_id();
    init_system();
    init_operator_id();

    // if there's any values stored in eeprom, use them!
    eeprom_load_info();

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
    const uint32_t location_interval = _odid_location.set_externally ? _odid_location.interval_max_ms*3 : _odid_location.interval_max_ms*0.2f;
    if (now_ms - _odid_location.last_populate_ms >= location_interval) {
        _odid_location.set_externally = false;
        populate_location();
    }

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

    if (_eeprom_save_needed) {
        _eeprom_save_needed = false;
        eeprom_save_info();
    }
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AP_OpenDroneID::init_basic_id()
{
    mavlink_open_drone_id_basic_id_t packet = {};

    packet.id_type = MAV_ODID_ID_TYPE_NONE;
    packet.ua_type = MAV_ODID_UA_TYPE_NONE;

    populate_basic_id(packet);
}

void AP_OpenDroneID::populate_basic_id(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_basic_id_t packet {};
    mavlink_msg_open_drone_id_basic_id_decode(&msg, &packet);
    populate_basic_id(packet);
}

void AP_OpenDroneID::populate_basic_id(const mavlink_open_drone_id_basic_id_t &packet)
{
    if (memcmp(&packet, &_odid_basic_id.info, sizeof(_odid_basic_id.info)) != 0) {
        memcpy(&_odid_basic_id.info, &packet, sizeof(_odid_basic_id.info));
        _odid_basic_id.has_changed = true;
        _odid_basic_id.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_basic_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_BASIC_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_basic_id_send_struct(chan, &_odid_basic_id.info);
    _odid_basic_id.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

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

void AP_OpenDroneID::populate_location(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_location_t packet {};
    mavlink_msg_open_drone_id_location_decode(&msg, &packet);
    populate_location(packet);
}

void AP_OpenDroneID::populate_location(const mavlink_open_drone_id_location_t &packet)
{
    if (memcmp(&packet, &_odid_location.info, sizeof(_odid_location.info)) != 0) {
        memcpy(&_odid_location.info, &packet, sizeof(_odid_location.info));
        _odid_location.has_changed = true;
        _odid_location.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_location(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_LOCATION)) {
        return;
    }
    mavlink_msg_open_drone_id_location_send_struct(chan, &_odid_location.info);
    _odid_location.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AP_OpenDroneID::init_authentication()
{
    mavlink_open_drone_id_authentication_t packet = {};

    packet.authentication_type = MAV_ODID_AUTH_TYPE_NONE;
    packet.data_page = 0;
    packet.page_count = 0;
    packet.length = 0;
    packet.timestamp = 0;

    populate_authentication(packet);
}

void AP_OpenDroneID::populate_authentication(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_authentication_t packet {};
    mavlink_msg_open_drone_id_authentication_decode(&msg, &packet);
    populate_authentication(packet);
}

void AP_OpenDroneID::populate_authentication(const mavlink_open_drone_id_authentication_t &packet)
{
    if (memcmp(&packet, &_odid_authentication.info, sizeof(_odid_authentication.info)) != 0) {
        memcpy(&_odid_authentication.info, &packet, sizeof(_odid_authentication.info));
        _odid_authentication.has_changed = true;
        _odid_authentication.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_authentication(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_AUTHENTICATION)) {
        return;
    }
    mavlink_msg_open_drone_id_authentication_send_struct(chan, &_odid_authentication.info);
    _odid_authentication.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AP_OpenDroneID::init_self_id()
{
    mavlink_open_drone_id_self_id_t packet = {};

    packet.description_type = MAV_ODID_DESC_TYPE_TEXT;

    populate_self_id(packet);
}

void AP_OpenDroneID::populate_self_id(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_self_id_t packet {};
    mavlink_msg_open_drone_id_self_id_decode(&msg, &packet);
    populate_self_id(packet);
}

void AP_OpenDroneID::populate_self_id(const mavlink_open_drone_id_self_id_t &packet)
{
    if (memcmp(&packet, &_odid_self_id.info, sizeof(_odid_self_id.info)) != 0) {
        memcpy(&_odid_self_id.info, &packet, sizeof(_odid_self_id.info));
        _odid_self_id.has_changed = true;
        _odid_self_id.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_self_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SELF_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_self_id_send_struct(chan, &_odid_self_id.info);
    _odid_self_id.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AP_OpenDroneID::init_system()
{
    mavlink_open_drone_id_system_t packet = {};

    packet.flags = MAV_ODID_LOCATION_SRC_LIVE_GNSS;
    packet.operator_latitude = 0;
    packet.operator_longitude = 0;
    packet.area_count = 0;
    packet.area_radius = 0;
    packet.area_ceiling = 0;
    packet.area_floor = 0;

    populate_system(packet);
}

void AP_OpenDroneID::populate_system(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_system_t packet {};
    mavlink_msg_open_drone_id_system_decode(&msg, &packet);
    populate_system(packet);
}

void AP_OpenDroneID::populate_system(const mavlink_open_drone_id_system_t &packet)
{
    if (memcmp(&packet, &_odid_system.info, sizeof(_odid_system.info)) != 0) {
        memcpy(&_odid_system.info, &packet, sizeof(_odid_system.info));
        _odid_system.has_changed = true;
        _odid_system.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_system(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_SYSTEM)) {
        return;
    }
    mavlink_msg_open_drone_id_system_send_struct(chan, &_odid_system.info);
    _odid_system.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AP_OpenDroneID::init_operator_id()
{
    mavlink_open_drone_id_operator_id_t packet = {};

    packet.operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA;

    populate_operator_id(packet);
}

void AP_OpenDroneID::populate_operator_id(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_operator_id_t packet {};
    mavlink_msg_open_drone_id_operator_id_decode(&msg, &packet);
    populate_operator_id(packet);
}

void AP_OpenDroneID::populate_operator_id(const mavlink_open_drone_id_operator_id_t &packet)
{
    if (memcmp(&packet, &_odid_operator_id.info, sizeof(_odid_operator_id.info)) != 0) {
        memcpy(&_odid_operator_id.info, &packet, sizeof(_odid_operator_id.info));
        _odid_operator_id.has_changed = true;
        _odid_operator_id.last_populate_ms = AP_HAL::millis();
    }
}

void AP_OpenDroneID::send_operator_id(const mavlink_channel_t chan)
{
    if (!HAVE_PAYLOAD_SPACE(chan, OPEN_DRONE_ID_OPERATOR_ID)) {
        return;
    }
    mavlink_msg_open_drone_id_operator_id_send_struct(chan, &_odid_operator_id.info);
    _odid_operator_id.last_send_ms[chan] = AP_HAL::millis();
}

// ---------------------------------------------------------
// ---------------------------------------------------------

MAV_RESULT AP_OpenDroneID::handle_message(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    // we only handle the _SET_ msgs
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_BASIC_ID:
        populate_basic_id(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SET_LOCATION:
        populate_location(msg);
        _odid_location.set_externally = true;
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

    // upon a mavlink _SET_ command we want to store it to eeprom but not in this thread, do it in the next update tick
    _eeprom_save_needed =
            _odid_basic_id.has_changed ||
            //_odid_loction.has_changed ||         // this does not have any strings that need to be saved
            _odid_authentication.has_changed ||
            _odid_self_id.has_changed ||
            //_odid_system.has_changed ||          // this does not have any strings that need to be saved
            _odid_operator_id.has_changed;

    return MAV_RESULT_ACCEPTED;
}


/*
  save eeprom info. These are hidden params
 */
void AP_OpenDroneID::eeprom_save_info(void)
{
    // this is necessary to save and load strings while the normal Ardupilot param structure does not support strings
    StorageAccess eeprom_storage(StorageManager::StorageODID);
    struct eeprom_info_t eeprom;

    eeprom.magic = eeprom_magic_value;
    eeprom.version = 1;

    memcpy(eeprom.data_v1.uas_id, &_odid_basic_id.info.uas_id, sizeof(_odid_basic_id.info));
    memcpy(eeprom.data_v1.authentication_data, &_odid_authentication.info.authentication_data, sizeof(_odid_authentication.info.authentication_data));
    memcpy(eeprom.data_v1.description, &_odid_self_id.info.description, sizeof(_odid_self_id.info.description));
    memcpy(eeprom.data_v1.operator_id, &_odid_operator_id.info.operator_id, sizeof(_odid_operator_id.info.operator_id));

    eeprom_storage.write_block(0, &eeprom, sizeof(eeprom));
}

/*
  load eeprom info. These are hidden params. Return false on read failure
 */
bool AP_OpenDroneID::eeprom_load_info(void)
{
    // this is necessary to save and load strings while the normal Ardupilot param structure does not support strings
    StorageAccess eeprom_storage(StorageManager::StorageODID);
    struct eeprom_info_t eeprom;

    if (!eeprom_storage.read_block(&eeprom, 0, sizeof(eeprom)) || eeprom.magic != eeprom_magic_value) {
        return false;
    }

    if (eeprom.version != 1) {
        // we only know how to parse v1
        return false;
    }

    memcpy(&_odid_basic_id.info, eeprom.data_v1.uas_id, sizeof(_odid_basic_id.info));
    _odid_basic_id.has_changed = true;

    memcpy(&_odid_authentication.info, eeprom.data_v1.authentication_data, sizeof(_odid_authentication.info));
    _odid_authentication.has_changed = true;

    memcpy(&_odid_self_id.info, eeprom.data_v1.description, sizeof(_odid_self_id.info));
    _odid_self_id.has_changed = true;

    memcpy(&_odid_operator_id.info, eeprom.data_v1.operator_id, sizeof(_odid_operator_id.info));
    _odid_operator_id.has_changed = true;

    return true;
}



AP_OpenDroneID *AP::OpenDroneID()
{
    return AP_OpenDroneID::get_singleton();
}
