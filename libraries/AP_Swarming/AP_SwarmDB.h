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

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <vector>

using namespace std;

#define SWARM_DB_LIST_MAX_SIZE                  25

class AP_SwarmDB {
public:
    struct SwarmDbItem_t {
        mavlink_swarm_vehicle_t item; // the whole mavlink struct with all the juicy details. sizeof() == 38
        uint32_t timestamp_ms; // last time this was refreshed, allows timeouts
    };

    // constructor
    AP_SwarmDB();

    // periodic update to remove stale vehicles
    void update(const mavlink_swarm_vehicle_t &ownship);

    uint32_t get_count() const { return _count; }

    void handle_swarm_vehicle(const mavlink_swarm_vehicle_t &vehicle);

    int32_t get_nearest_index(const Location &loc) const;

    //returns ownship index
    uint32_t get_ownship_id() const { return get_item_id(0); }
    SwarmDbItem_t get_ownship() const { return _list[0]; };

    uint32_t get_item_id(const int32_t index) const;
    bool get_item(const int32_t index, mavlink_swarm_vehicle_t &vehicle) const;
    bool get_item(const int32_t index, SwarmDbItem_t &dbItem) const;

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    int32_t find_index(const mavlink_swarm_vehicle_t &vehicle) const;

    bool is_valid_index(int32_t index) const { return (index >= 0 && index < _count); }

private:
    // update an existing vehicle
    void set_vehicle(const int32_t index, const mavlink_swarm_vehicle_t &vehicle);

    // remove a vehicle from the list
    void remove_vehicle(const int32_t index);

    std::vector<SwarmDbItem_t> _list;

    int32_t     _count;
    uint32_t    _swarmDb_update_ms;
};
