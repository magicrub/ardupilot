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
#include "AP_Swarming_Simulator.h"

#if AP_SWARMING_SIMULATOR_ENABLE
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>

#if AP_SWARMING_TIMESTAMP_IS_GPS
#include <AP_GPS/AP_GPS.h>
#endif

extern const AP_HAL::HAL& hal;

//#include <AP_Swarming/AP_Swarming.h>

const AP_Param::GroupInfo AP_Swarming_Simulator::var_info[] = {

    // @Param: COUNT
    // @DisplayName: Swarming Simulator vehicle count
    // @Description: Swarming Simulator vehicle count
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("COUNT", 1, AP_Swarming_Simulator, _count_param, 0),

    // @Param: DIST
    // @DisplayName: Swarming Simulator spawn distance
    // @Description: Swarming Simulator spawn distance
    // @Units: m
    AP_GROUPINFO("DIST", 2, AP_Swarming_Simulator, _initial_distance_m, 1000),

    // @Param: SPD
    // @DisplayName: Swarming Simulator aircraft speed
    // @Description: Swarming Simulator aircraft speed
    // @Range: 1 100
    // @User: Advanced
    AP_GROUPINFO("SPD", 3, AP_Swarming_Simulator, _speed, 20),

    // @Param: EFF_RAD
    // @DisplayName: Swarming Simulator Effective Radius
    // @Description: Swarming Simulator Effective Radius. The distance we can reliably keep a data link
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("EFF_RAD", 4, AP_Swarming_Simulator, _effective_radius_m, 10E3),

    // @Param: OVERLAP
    // @DisplayName: Swarming Simulator signal overlap
    // @Description: Swarming Simulator signal overlap
    // @Range: 0 100
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("OVERLAP", 5, AP_Swarming_Simulator, _coverage_overlap_percent, 0),

    // @Param: LOIT_R
    // @DisplayName: Swarming Simulator loiter radius
    // @Description: Swarming Simulator loiter radius of the simulatoed vehicles
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("LOIT_R", 6, AP_Swarming_Simulator, _loiter_radius_m, 300),

    AP_GROUPEND
};

void AP_Swarming_Simulator::update()
{
    if (_count != _count_param) {
        _initialized = false;
    }
    if (!_initialized) {
        init();
    }
    if (_count <= 0) {
        // disabled
        return;
    }

    for (uint32_t i=0; i<_count; i++) {
        update_vehicle(_vehicles[i]);
    }
}

void AP_Swarming_Simulator::init()
{
    if (_count_param <= 0 || _count_param > AP_SWARMING_SIMULATOR_MAX_COUNT) {
        // quick sanity check. If count values are invalid are YUUUGE just disable immediately
        _count_param.set_and_notify(0);

    } else {
        _vehicles.resize(_count_param);
    }

    if (!AP::ahrs().home_is_set()) {
        // can't init yet since they spawn relative to home
        _count = 0;
        return;
    }

    _count = _count_param;

    for (uint32_t i=0; i<_count; i++) {
        init_vehicle(_vehicles[i]);
    }
    _initialized = true;
}



void AP_Swarming_Simulator::update_vehicle(SwarmSimVehicle &simVehicle)
{
    const uint32_t now_us = AP_HAL::micros64();
    const uint32_t now_ms = now_us * 1e-3;

    // if (now_ms - simVehicle.last_update_ms < 100) {
    //     // run at 10Hz
    //     return;
    // }
    // simVehicle.last_update_ms = now_ms;

    if (simVehicle.last_tick_us == 0) {
        simVehicle.last_tick_us = now_us;
        return;
    }

    if (now_ms - simVehicle.last_msg_send_ms >= 1000) {
        simVehicle.last_msg_send_ms = now_ms;
#if AP_SWARMING_TIMESTAMP_IS_GPS
    simVehicle.vehicle.time_usec = AP::gps().time_epoch_usec();
#else
    simVehicle.vehicle.time_usec = now_us;
#endif
        AP::swarm()->handle_swarm_vehicle(simVehicle.vehicle);
    }

    const float dt = (now_us - simVehicle.last_tick_us) * 1.0e-6f;
    simVehicle.last_tick_us = now_us;

    // get target Location
    const Location loc_target = AP_Swarming::get_location_target(simVehicle.vehicle);

    Location loc = AP_Swarming::get_location(simVehicle.vehicle);
    if (simVehicle.vehicle.speed > 0) {
        loc.offset_bearing(simVehicle.vehicle.cog, simVehicle.vehicle.speed*dt);
        AP_Swarming::set_location(simVehicle.vehicle, loc);
    }
 
    switch (simVehicle.vehicle.state_nav) {
    case PROCEEDING_HOME:
    case INGRESSING_TO_MESH:
        simVehicle.vehicle.speed = _speed; // allow the param to control the current aircraft speed

        // scoot it forward a bit and make it our new position

        // heading in a straight line to the target. Once we hit our loiter radius, change state to loiter
        if (loc.get_distance(loc_target) <= MAX(20, _loiter_radius_m)) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "Swarm id:%u REACHED TARGET, loitering", (unsigned)simVehicle.vehicle.aircraft_id);
            simVehicle.vehicle.state_nav = ON_STATION;
        }
        break;

    case ON_STATION:
        if (simVehicle.vehicle.speed <= 0 || _loiter_radius_m <= 20) {
            // with small radius, snap to target and park.
            loc = loc_target;
            simVehicle.vehicle.speed = 0;

        } else {
            simVehicle.vehicle.speed = _speed; // allow the param to continuously control the current aircraft speed
 
            // since we're lazy and don't want to calculate the angular velocities and such, lets just
            // snap the vehicle to exactly the loiter radius instead of calculating a controlled turn angle.
            // This keeps the loiter radius accurate which is pretty much all we care about
            const float bearing_target_to_current_loc = loc_target.get_bearing_to(loc) * 0.01f; // cd -> deg
            loc = loc_target; 
            loc.offset_bearing(bearing_target_to_current_loc, _loiter_radius_m);

            // make sure it's facing the correct way
            simVehicle.vehicle.cog = wrap_360(bearing_target_to_current_loc + 90); // Clockwise tangent, plus a little more runnder
        }
        AP_Swarming::set_location(simVehicle.vehicle, loc);
        break;

    default:
        // unknown state. Go home and cry!
        simVehicle.vehicle.state_nav = PROCEEDING_HOME;
        AP_Swarming::set_location_target(simVehicle.vehicle, AP::ahrs().get_home());
        break;
    }
}

void AP_Swarming_Simulator::init_vehicle(SwarmSimVehicle &simVehicle)
{
    memset(&simVehicle, 0, sizeof(simVehicle));

    // move it into the future a random amount to create a phase delay within <= 1s
    //simVehicle.last_update_ms = AP_HAL::millis() + (rand() % 1000);

    const mavlink_swarm_vehicle_t host_vehicle = AP::swarm()->get_vehicle();
    simVehicle.vehicle.aircraft_id = host_vehicle.aircraft_id + (++_aircraft_id);
    simVehicle.vehicle.squadron_id = host_vehicle.squadron_id;

    simVehicle.vehicle.cog = (rand() % 360);
    simVehicle.vehicle.effective_radius = _effective_radius_m;
    simVehicle.vehicle.state_nav = INGRESSING_TO_MESH;
    simVehicle.vehicle.speed = _speed;

    Location pt = AP_Swarming::get_location(host_vehicle);
    pt.alt += 20000; // in case we're launching them while on the ground, bump their altitude up 200m

    AP_Swarming::set_location(simVehicle.vehicle, pt);
 
    // set target distance to _initial_distance_m +/- (0.5*_initial_distance_m)
    pt.offset_bearing(simVehicle.vehicle.cog, (_initial_distance_m/2) + (rand() % (int32_t)_initial_distance_m));
    AP_Swarming::set_location_target(simVehicle.vehicle, pt);
}

#endif // AP_SWARMING_SIMULATOR_ENABLE

