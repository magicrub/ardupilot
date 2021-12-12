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

   Author: Tom Pittenger
 */
#pragma once


#include <AP_HAL/AP_HAL.h>
#include "AP_Swarming.h"

#ifndef AP_SWARMING_SIMULATOR_ENABLE
    #define AP_SWARMING_SIMULATOR_ENABLE (HAL_AP_SWARMING_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL))
#endif

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <vector>

#define AP_SWARMING_SIMULATOR_MAX_COUNT     100

class AP_Swarming_Simulator {
    friend class AP_Swarming;

    // constructor
    AP_Swarming_Simulator() {
        _vehicles.reserve(AP_SWARMING_SIMULATOR_MAX_COUNT);
        
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    void update();

private:
    void init();

    struct SwarmSimVehicle {
        uint32_t        last_msg_send_ms;
        uint32_t        last_update_ms;
        uint32_t        last_target_change_ms;
        mavlink_swarm_vehicle_t vehicle;
        uint32_t        last_tick_us;
    };

    void update_vehicle(SwarmSimVehicle &simVehicle);
    void init_vehicle(SwarmSimVehicle &simVehicle);
    
    std::vector<SwarmSimVehicle> _vehicles;

    uint16_t        _count;
    bool            _initialized;
    uint16_t        _aircraft_id;

    AP_Int32        _count_param;
    AP_Float        _initial_distance_m;
    AP_Float        _speed;
    AP_Float        _effective_radius_m;
    AP_Float        _coverage_overlap_percent;
    AP_Float        _loiter_radius_m;
};
