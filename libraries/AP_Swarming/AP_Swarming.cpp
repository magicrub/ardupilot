/*
   Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.

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

   Author: Tom Pittenger & Michael Day
 */

#include "AP_Swarming.h"

#if HAL_AP_SWARMING_ENABLED
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/AP_Vehicle.h>

#if AP_SWARMING_TIMESTAMP_IS_GPS
#include <AP_GPS/AP_GPS.h>
#endif

extern const AP_HAL::HAL& hal;

AP_Swarming *AP_Swarming::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_Swarming::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Swarming Type
    // @Description: Swarming Type
    // @Values: 0:Disabled,1:Radius
    // @User: Advanced
    AP_GROUPINFO_FLAGS("TYPE",     0, AP_Swarming, _params.type,    0, AP_PARAM_FLAG_ENABLE),

    // @Param: ID_AIRCRAFT
    // @DisplayName: Swarming Aircraft ID
    // @Description: Swarming Aircraft ID. This, in combination with SWRM_ID_SQUADRON, should be a unique number
    // @User: Advanced
    AP_GROUPINFO("ID_AIRCRAFT",   3, AP_Swarming, _params.id_aircraft, 1),

    // @Param: ID_SQUADRON
    // @DisplayName: Swarming Aircraft ID
    // @Description: Swarming Aircraft ID. This, in combination with SWRM_ID_AIRCRAFT, should be a unique number
    // @User: Advanced
    AP_GROUPINFO("ID_SQUADRON",   4, AP_Swarming, _params.id_squadron, 0),

#if HAL_ADSB_ENABLED
    // @Param: FWD_TO_ADSB
    // @DisplayName: Fwd Swarm vehicles to ADSB library
    // @Description: Forward Swarm Vehicles to the ADSB library so that it can be streamed down as an ADSB object via SRx_ADSB stream rates. Since ADSB vehicles are forwarded tot he avoidance library, this also gives us avoidance capabilities
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("FWD_TO_ADSB",   5, AP_Swarming, _params.fwd_inbound_vehicles_to_adsb_lib, 1),
#endif

#if AP_SWARMING_SIMULATOR_ENABLE
    // @Group: SIM_
    // @Path: AP_Swarming_Simulator.cpp
    AP_SUBGROUPINFO(_sim, "SIM_", 6, AP_Swarming, AP_Swarming_Simulator),
#endif

    // @Param: SEND_CHAN
    // @DisplayName: MAVLink channel to send Swarm Vehicle packets
    // @Description: MAVLink channel to send Swarm Vehicle packets. Use -1 to disable sending, 0 to only send to a MAVLink port that it receives other Swarm-specific packets, else any other value will force sending it to that channel
    // @User: Advanced
    AP_GROUPINFO("SEND_CHAN",   7, AP_Swarming, _params.chan_select, 0),

    // @Param: EFF_RAD
    // @DisplayName: Effective Radius
    // @Description: Effective Radius of RF reach for stable connection to neighboring swarm vehicle. This highly depends on the capabilities of the installed radio system.
    // @Units: m
    AP_GROUPINFO("EFF_RAD",   8, AP_Swarming, _params.effective_radius, 10000),


    //parameter that catches overlappyness of the radii

    ///parmeters (array) for radius of aircraft loiter point

    ///parameters for coverage

    // @Param: DEBUG1
    AP_GROUPINFO("DEBUG1",   50, AP_Swarming, _params.debug1, 0),
    // @Param: DEBUG2
    AP_GROUPINFO("DEBUG2",   51, AP_Swarming, _params.debug2, 0),
    // @Param: DEBUG3
    AP_GROUPINFO("DEBUG3",   52, AP_Swarming, _params.debug3, 0),

    AP_GROUPEND
};

// constructor
AP_Swarming::AP_Swarming()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Swarming must be singleton");
    }
#endif
    _singleton = this;
}

/*
 * Initialize variables and allocate memory for array
 */
void AP_Swarming::init(void)
{
    update_my_vehicle();
    _my_vehicle.state_nav = INGRESSING_TO_MESH;

    _is_initialized = true;
}

/*
 * periodic update to handle vehicle timeouts and trigger collision detection
 */
void AP_Swarming::update_50Hz(void)
{
    if (_params.type <= 0) {
        _is_initialized = false;
        return;
    }

    if (!_is_initialized) {
        init();
        return;
    }
    

    update_my_vehicle();
    _db.update();

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_swarm_vehicle_send_ms > 1000) {
        _last_swarm_vehicle_send_ms = now_ms;
        send_swarm_vehicle(_my_vehicle);
    }

    do_fancy_algorithm_stuff();

#if AP_SWARMING_SIMULATOR_ENABLE
    _sim.update();
#endif
}

void AP_Swarming::handle_swarm_vehicle(mavlink_swarm_vehicle_t &swarm_vehicle)
{
#if AP_SWARMING_TIMESTAMP_IS_GPS
    // create timestamp if empty
    if (swarm_vehicle.time_usec == 0) {
        swarm_vehicle.time_usec = AP::gps().time_epoch_usec();
    }
#else
    // ignore external time, always use our own internal time since boot
    swarm_vehicle.time_usec = AP_HAL::micros64();
#endif

    _db.handle_swarm_vehicle(_my_vehicle, swarm_vehicle);
    send_to_adsb(swarm_vehicle);
}

MAV_RESULT AP_Swarming::handle_msg(const mavlink_channel_t chan, const mavlink_message_t &msg)
{
    if (_chan_inbound == MAVLINK_CHANNEL_INVALID) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm found on MAVLink channel %d", (int)chan);
    }

    const uint32_t now_ms = AP_HAL::millis();

    _chan_inbound = chan;
    _chan_last_ms = now_ms;

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_SWARM_VEHICLE: {
        mavlink_swarm_vehicle_t swarm_vehicle {};
        mavlink_msg_swarm_vehicle_decode(&msg, &swarm_vehicle);

        gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm Vehicle: %u, %u, %u, %d, %.2f, %.2f, %d, %d, %.2f, %d, %d, %.2f", 
            (unsigned)swarm_vehicle.aircraft_id, (unsigned)swarm_vehicle.squadron_id,
            (unsigned)swarm_vehicle.state_nav, (int)swarm_vehicle.speed, (double)swarm_vehicle.cog,
            (double)swarm_vehicle.radius_effective,
            (int)swarm_vehicle.lat, (int)swarm_vehicle.lon, (double)swarm_vehicle.altMSL,
            (int)swarm_vehicle.lat_target, (int)swarm_vehicle.lon_target, (double)swarm_vehicle.altMSL_target);

        handle_swarm_vehicle(swarm_vehicle);
        }
        return MAV_RESULT_ACCEPTED;
    
    case MAVLINK_MSG_ID_SWARM_COMMLINK_STATUS: {
        mavlink_swarm_commlink_status_t swarm_commlink_status {};
        mavlink_msg_swarm_commlink_status_decode(&msg, &swarm_commlink_status);

        gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm Commlink Status: %d, %d, %f, %d", 
            swarm_commlink_status.aircraft_id_self, swarm_commlink_status.aircraft_id_external,
            swarm_commlink_status.ROSR, swarm_commlink_status.last_contact);
        }
        return MAV_RESULT_ACCEPTED;
    
    default:
        return MAV_RESULT_UNSUPPORTED;
    } // switch

    return MAV_RESULT_FAILED;
}

MAV_RESULT AP_Swarming::handle_command_long(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_SWARM_RADIUS:
        gcs().send_text(MAV_SEVERITY_DEBUG, "AP_Swarming: cmd long id %u: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
            (unsigned)packet.command,
            (double)packet.param1,
            (double)packet.param2,
            (double)packet.param3,
            (double)packet.param4,
            (double)packet.param5,
            (double)packet.param6,
            (double)packet.param7);

        hal.console->printf("%.2f", (double)packet.param1);
        return MAV_RESULT_ACCEPTED;

    default:
        return MAV_RESULT_UNSUPPORTED;

    } // switch

    return MAV_RESULT_FAILED;
}

void AP_Swarming::update_my_vehicle()
{
#if AP_SWARMING_TIMESTAMP_IS_GPS
    const uint64_t now_us = AP::gps().time_epoch_usec();
#else
    const uint64_t now_us = AP_HAL::micros64();
#endif
    _my_vehicle.time_usec = now_us; // this probably only needs to be updated when we call send_swarm_vehicle()

    _my_vehicle.aircraft_id = _params.id_aircraft;
    _my_vehicle.squadron_id = _params.id_squadron;

    const Vector2f groundspeed = AP::ahrs().groundspeed_vector();
    _my_vehicle.cog = degrees(atan2f(groundspeed.y, groundspeed.x));
    _my_vehicle.speed = groundspeed.length();

    _my_vehicle.radius_effective = _params.effective_radius;

    Location loc;
    if (!AP::ahrs().get_position(loc)) {
        // not sure how to handle this.. just quit here I guess
        return;
    }

    set_location(loc);

    Location loc_target;
    if (AP::vehicle()->get_target_location(loc_target)) {
        set_location_target(loc_target);
    } else {
        // if target is unknown, use current location
        set_location_target(loc);
    }
}

void AP_Swarming::assign_new_target(const Location loc)
{
    set_location_target(loc);
    if (AP::vehicle() != nullptr) {
        AP::vehicle()->set_mode_to_guided(ModeReason::SWARM); // this is a hack for PLANE to set us into GUided mode. Wont' work for Coptyer/rover... yet
        AP::vehicle()->set_target_location(loc); // this command is ignored if we're not already in guided
    }
}

bool AP_Swarming::chan_ok(const uint8_t chan) const
{
    if (chan >= gcs().num_gcs()) {
        return false;
    }

    GCS_MAVLINK *c = gcs().chan(chan);

    return c != nullptr && !c->is_private() && c->is_active();
}

void AP_Swarming::send_swarm_vehicle(const mavlink_swarm_vehicle_t &vehicle) 
{
    // _chan_select:
    // <0 never send
    // 0 send only to chan that sent to me
    // # send to that specific chan only

    mavlink_channel_t chan;

    if (_params.chan_select == 0) {
        chan = _chan_inbound;
    } else if (_params.chan_select > 0) {
        chan = (mavlink_channel_t)(_params.chan_select - 1);
    } else {
        // don't send
        return;
    }

    if (chan_ok(chan) && HAVE_PAYLOAD_SPACE(chan, SWARM_VEHICLE)) {
        mavlink_msg_swarm_vehicle_send_struct(chan, &vehicle);
    }
}

void AP_Swarming::send_to_adsb(const mavlink_swarm_vehicle_t &msg)
{
#if HAL_ADSB_ENABLED
    if (!_params.fwd_inbound_vehicles_to_adsb_lib) {
        return;
    }

    AP_ADSB *adsb = AP::ADSB();
    if (!adsb || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    AP_ADSB::adsb_vehicle_t vehicle {};

    vehicle.info.ICAO_address = get_id(msg);
    vehicle.info.tslc = 0;
    vehicle.info.lat = msg.lat;
    vehicle.info.lon = msg.lon;
    vehicle.info.altitude = msg.altMSL * 1000;   // convert m to mm
    vehicle.info.heading = msg.cog * 100;        // convert deg to cdeg
    vehicle.info.hor_velocity = msg.speed * 100; // convert m/s to cm/s
    vehicle.info.ver_velocity = 0;
    vehicle.info.squawk = 1200;
    vehicle.info.emitter_type = ADSB_EMITTER_TYPE_UAV;

    // convert squadron_id=5 aircraft_id=1234 to callsign "SWRMsaaa" like "SWRM5234"
    vehicle.info.callsign[0] = 'S';
    vehicle.info.callsign[1] = 'W';
    vehicle.info.callsign[2] = 'R';
    vehicle.info.callsign[3] = 'M';
    vehicle.info.callsign[4] = msg.squadron_id % 10 + '0';
    vehicle.info.callsign[5] = ((msg.aircraft_id / 100) % 10) + '0';
    vehicle.info.callsign[6] = ((msg.aircraft_id / 10) % 10) + '0';
    vehicle.info.callsign[7] = ((msg.aircraft_id / 1) % 10) + '0';
    vehicle.info.callsign[MAVLINK_MSG_ADSB_VEHICLE_FIELD_CALLSIGN_LEN-1] = 0;

    vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
    vehicle.info.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;

    if (msg.lat || msg.lon) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
    }

    vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
    vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
    vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    vehicle.info.flags |= ADSB_FLAGS_VALID_SQUAWK;
    //vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;

    vehicle.last_update_ms = AP_HAL::millis();
    adsb->handle_adsb_vehicle(vehicle);
#endif
}

void AP_Swarming::do_fancy_algorithm_stuff()
{
    // If we ever get 1km away from home in any flight mode, plot a course to the nearest swarm vehicle.
    // If none, just plot a course 90deg starboard and go out another 1km. Once we reach the target then
    // loiter for 2 min and then head home (but use vehicle current alt instead of home's 0 AGL)

    if (_params.type != 2 && _params.type != 3) {
        return;
    }
    if (!AP::ahrs().home_is_set()) {
        return;
    }
    static bool has_reached_fence = false;
    //static uint32_t start_loiter_ms = 0;
    //const uint32_t now_ms = AP_HAL::millis();
    const Location home = AP::ahrs().get_home();
    const Location my_loc = get_location();
    const Location my_target = get_location_target();
    Location new_target;

    const uint32_t loiter_duration = AP::vehicle()->loiter_duration(); // TODO: add support for all vehicle types (Rover, Copter... )
    const bool is_loitering = (loiter_duration > 0);

    switch (_my_vehicle.state_nav) {
        default:
        case INGRESSING_TO_MESH:
            if (!has_reached_fence) {
                if (my_loc.get_distance(home) > 1000) {
                    gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm Reached fence, algorithm stuff starting!");
                    // grab a target. Either closest swarm item or just take a right turn
                    has_reached_fence = true;
                    load_nearest_swarm_vehicle(new_target);
                    assign_new_target(new_target);
                }
            } else if (is_loitering || my_loc.get_distance(my_target) < 20) {
                _my_vehicle.state_nav = ON_STATION;
            }
            break;

        case ON_STATION:
            if (is_loitering && (loiter_duration > (2U * 60UL * 1000UL))) {
                gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm done loitering, lets go home");
                _my_vehicle.state_nav = PROCEEDING_HOME;
                new_target = home;
                new_target.alt = my_loc.alt;
                assign_new_target(new_target);
            }
        break;

        case PROCEEDING_HOME:
            if (is_loitering && (loiter_duration > (1U * 60UL * 1000UL))) {
            // nothing to do, we're just flying to home and we'll loiter forever once we're there.
                _my_vehicle.state_nav = INGRESSING_TO_MESH;
                load_nearest_swarm_vehicle(new_target);
                assign_new_target(new_target);
            }
            break;
    }
}

void AP_Swarming::load_nearest_swarm_vehicle(Location &loc)
{
    const Location my_loc = get_location();

    mavlink_swarm_vehicle_t closest_vehicle;
    if (_db.get_item(_db.get_nearest_index(my_loc), closest_vehicle)) {
        // there's a vehicle nearby, go to the nearest one!
        if (_params.type == 2) {
            loc = get_location(closest_vehicle);
        } else if (_params.type == 3) {
            loc = get_location_target(closest_vehicle);
        } else {
            _my_vehicle.state_nav = PROCEEDING_HOME;
            gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm - unknown TYPE");
            return;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm - heading to newest swarm item");

    } else {
        // from current location, turn right and plot a target 1km away
        loc = my_loc;
        const float bearing = wrap_360(_my_vehicle.cog + 90); // right turn
        loc.offset_bearing(bearing, 1000);
        gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm - heading to new loiter target");
    }

}

AP_Swarming *AP::swarm()
{
    return AP_Swarming::get_singleton();
}

#endif // HAL_AP_SWARMING_ENABLED
