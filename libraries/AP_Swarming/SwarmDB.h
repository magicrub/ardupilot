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
#include <AP_HAL/AP_HAL_Boards.h>

//#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/AP_ExpandingArray.h>

#define SWARM_DB_LIST_MAX_SIZE                  1000

class SwarmDB {
public:

    // constructor
    SwarmDB() { }

    // periodic update to remove stale vehicles
    void update();

    uint32_t get_count() const { return _count; }

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    int32_t find_index(const mavlink_swarm_vehicle_t &vehicle) const;

    void handle_swarm_vehicle(const mavlink_swarm_vehicle_t &me, const mavlink_swarm_vehicle_t &vehicle);

    int32_t get_nearest_index(const Location &loc) const;

    bool get_item(const int32_t index, mavlink_swarm_vehicle_t &vehicle) const;

private:

    bool is_valid_index(int32_t index) const { return (index >= 0 && index < _count); }

    // remove a vehicle from the list
    void remove_vehicle(const int32_t index);

    // update an existing vehicle
    void set_vehicle(const int32_t index, const mavlink_swarm_vehicle_t &vehicle);

    // void set_id(const int32_t index, const uint32_t id) {
    //     set_id(index, id>>16, id);
    // }
    // void set_id(const int32_t index, const uint16_t id_MSB, const uint16_t id_LSB) {
    //     if (is_valid_index(index)) {
    //         _list[index].squadron_id = id_MSB;
    //         _list[index].aircraft_id = id_LSB;
    //     }
    // }
    
    struct SwarmDbItem_t {
        mavlink_swarm_vehicle_t item; // the whole mavlink struct with all the juicy details. sizeof() == 38
        uint32_t timestamp_ms; // last time this was refreshed, allows timeouts
    };

    AP_ExpandingArray<SwarmDbItem_t> _list {1};

    int32_t     _count;
    uint32_t    _swarmDb_update_ms;
};
