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

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#define HAL_AP_SWARMING_ENABLED 1

#ifndef HAL_AP_SWARMING_ENABLED
    #define HAL_AP_SWARMING_ENABLED (APM_BUILD_TYPE_IS_VEHICLE && !HAL_MINIMIZE_FEATURES)
#endif

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_SwarmDB.h"
#include "AP_SwarmROI.h"
#include "AP_Swarming_Simulator.h"
#include <AP_Common/Location.h>

#define AP_SWARMING_TIMESTAMP_IS_GPS 0

class AP_Swarming {
    //friend class AP_Swarming_Simulator;

public:

    // constructor
    AP_Swarming();

    /* Do not allow copies */
    AP_Swarming(const AP_Swarming &other) = delete;
    AP_Swarming &operator=(const AP_Swarming&) = delete;

    // get singleton instance
    static AP_Swarming *get_singleton(void) {
        return _singleton;
    }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update_50Hz(void);

    // mavlink message handler
    MAV_RESULT handle_msg(const mavlink_channel_t chan, const mavlink_message_t &msg);
    MAV_RESULT handle_command_long(const mavlink_command_long_t &packet);

    // vehicle handler. Useful for recieving vehicles from external non-mavlink libraties
    void handle_swarm_vehicle(mavlink_swarm_vehicle_t &swarm_vehicle);
    
    // helper to always get MSL altitude as a float from a Location
    static float get_altitude_MSL(const Location loc) {
        int32_t alt_cm;
        if (loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            return alt_cm*0.01f; // cm -> m;
        }
        return loc.alt*0.01f; // cross your fingers and hope it's already absolute!
    }

    // helpers for set/get for loc/loc_target in static and non-static forms
    static Location get_location(const mavlink_swarm_vehicle_t &swarm_vehicle) {
        return Location(swarm_vehicle.lat, swarm_vehicle.lon, swarm_vehicle.altMSL*100, Location::AltFrame::ABSOLUTE);
    }
    static Location get_location_target(const mavlink_swarm_vehicle_t &swarm_vehicle) {
        return Location(swarm_vehicle.lat_target, swarm_vehicle.lon_target, swarm_vehicle.altMSL_target*100, Location::AltFrame::ABSOLUTE);
    }
    static void set_location(mavlink_swarm_vehicle_t &swarm_vehicle, const Location loc) {
        swarm_vehicle.lat = loc.lat;
        swarm_vehicle.lon = loc.lng;
        swarm_vehicle.altMSL = AP_Swarming::get_altitude_MSL(loc); // this enforces that we're always in MSL (m) even if the Location.AlrFrame is mixed
    }
    static void set_location_target(mavlink_swarm_vehicle_t &swarm_vehicle, const Location loc_target) {
        swarm_vehicle.lat_target = loc_target.lat;
        swarm_vehicle.lon_target = loc_target.lng;
        swarm_vehicle.altMSL_target = AP_Swarming::get_altitude_MSL(loc_target); // this enforces that we're always in MSL (m) even if the Location.AlrFrame is mixed
    }

    mavlink_swarm_vehicle_t get_vehicle() const { return _my_vehicle; }

    static uint32_t get_id(const mavlink_swarm_vehicle_t &vehicle) {
        return ((uint32_t)vehicle.squadron_id << 16) | ((uint32_t)vehicle.aircraft_id);
    }

    AP_SwarmROI &get_roi();
    const AP_SwarmROI &get_roi() const;

private:
    static AP_Swarming *_singleton;

    void init();
    void send_to_adsb(const mavlink_swarm_vehicle_t &msg);
    void send_swarm_vehicle(const mavlink_swarm_vehicle_t &vehicle);

    // helpers for set/get for loc/loc_target of _my_vehicle
    Location get_location_target() const { return get_location_target(_my_vehicle); }
    Location get_location() const { return get_location(_my_vehicle); }
    void set_location_target(const Location loc_target) { set_location_target(_my_vehicle, loc_target); }
    void set_location(const Location loc) { set_location(_my_vehicle, loc); }

    void update_my_vehicle();

    // Helper Functions to 
    void assign_new_target() { assign_new_target(get_location_target(_my_vehicle)); }
    void assign_new_target(const Location loc_target);

    void do_fancy_algorithm_stuff();
    void load_nearest_swarm_vehicle(Location &new_target);

    bool chan_ok(const uint8_t chan) const;
    bool chan_ok(const mavlink_channel_t chan) const { return chan_ok((uint8_t)chan); }

    AP_SwarmDB _db;
    AP_SwarmROI _roi;

    const mavlink_channel_t MAVLINK_CHANNEL_INVALID = (mavlink_channel_t)99;
    mavlink_channel_t _chan_inbound = MAVLINK_CHANNEL_INVALID;          //MAVLink channel
    uint32_t    _chan_last_ms;  //Last time a MAVLink channel was confirmed

    uint32_t    _last_swarm_vehicle_send_ms;
    bool        _is_initialized;

    struct {
        AP_Int8     type;
        AP_Int32    id_aircraft;
        AP_Int32    id_squadron;
        AP_Int8     fwd_inbound_vehicles_to_adsb_lib;
        AP_Int16    chan_select;
        AP_Int32    effective_radius;
        AP_Float    debug1;
        AP_Float    debug2;
        AP_Float    debug3;
    } _params;

    mavlink_swarm_vehicle_t _my_vehicle;

    bool _is_breached;

#if AP_SWARMING_SIMULATOR_ENABLE
    AP_Swarming_Simulator _sim;
#endif
};

namespace AP {
    AP_Swarming *swarm();
};
