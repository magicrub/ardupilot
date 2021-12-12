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

#include "AP_Swarming.h"

#if HAL_AP_SWARMING_ENABLED
#include "AP_SwarmDB.h"

#define SWARM_DB_LIST_UPDATE_INTERVAL_MAX_MS    1000
#define SWARM_DB_VEHICLE_TIMEOUT_US             (10UL*1E6)

extern const AP_HAL::HAL& hal;

AP_SwarmDB::AP_SwarmDB()
{
    _list.reserve(SWARM_DB_LIST_MAX_SIZE);

    // assign ownship at index. Index 0 will *always* be yourself.
    // if you ever want to ignore yourself in a list, just loop from index 1 to get_count() intead of 0
    _count = 1;
    set_vehicle(0, mavlink_swarm_vehicle_t {});
}

void AP_SwarmDB::handle_swarm_vehicle(const mavlink_swarm_vehicle_t &vehicle)
{
    if (get_ownship_id() == AP_Swarming::get_id(vehicle)) {
        // we've detected ourselves or someone is trying to impersonate us.
        return;
    }

    const Location vehicle_loc = AP_Swarming::get_location(vehicle);
    const int32_t index = find_index(vehicle);
    const bool is_tracked_in_list = (index > 0); // index == 0 is reserved for "me" which is not allowed to be updated externally

    if (vehicle_loc.is_latlng_zero() || !vehicle_loc.check_latlng()) {
        // invalid lat/lng or unknown time. If we're tracking it, delete from list. Otherwise ignore it.
        if (is_tracked_in_list) {
            remove_vehicle(index);
        }

    } else if (is_tracked_in_list) {
        // found, update it
        set_vehicle(index, vehicle);

    } else if (_count < SWARM_DB_LIST_MAX_SIZE) {
        // not found and there's room, add it to the end of the list
        _count++;
        _list.resize(_count);
        set_vehicle(_count-1, vehicle);
    }
}

// periodic update to remove stale vehicles and cache location
void AP_SwarmDB::update(const mavlink_swarm_vehicle_t &ownship)
{
    // assign ownship at index. Index 0 will *always* be yourself.
    set_vehicle(0, ownship);

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _swarmDb_update_ms < SWARM_DB_LIST_UPDATE_INTERVAL_MAX_MS) {
        return;
    }
    _swarmDb_update_ms = now_ms;

    // check current list for vehicles that time out
    int32_t index = 0;
    while (index < _count) {
        // check list and drop stale vehicles. When disabled, the list will get flushed
        if (now_ms - _list[index].timestamp_ms > SWARM_DB_VEHICLE_TIMEOUT_US) {
            // don't increment index, we want to check this same index again because the contents changed
            // also, if we're disabled then clear the list
            remove_vehicle(index);
        } else {
            index++;
        }
    }
}

// return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
int32_t AP_SwarmDB::find_index(const mavlink_swarm_vehicle_t &vehicle) const
{
    const uint32_t vehicle_id = AP_Swarming::get_id(vehicle);
    for (int32_t i = 0; i < _count; i++) {
        const uint32_t item_id = AP_Swarming::get_id(_list[i].item);
        if (item_id == vehicle_id) {
            return i;
        }
    }
    return -1;
}

// update an existing vehicle
void AP_SwarmDB::set_vehicle(const int32_t index, const mavlink_swarm_vehicle_t &vehicle)
{
    if (!is_valid_index(index)) {
        return;
    }

    memcpy(&_list[index].item, &vehicle, sizeof(_list[0].item));

    _list[index].timestamp_ms = AP_HAL::millis();
}

// remove a vehicle from the list
void AP_SwarmDB::remove_vehicle(const int32_t index)
{
    if (!is_valid_index(index)) {
        return;
    }

    // 'forget' the last element by reducing the list size by 1
    _count--;

    // if the one we're removing is the last one, then we're done!
    if (index == _count) {
        return;
    }
    // Else, overwrite the removed index with the last index
    memcpy(&_list[index], &_list[_count], sizeof(_list[0]));
}

int32_t AP_SwarmDB::get_nearest_index(const Location &loc) const
{
    float shortest = -1;
    int32_t index = -1;
    
    for (uint16_t i = 0; i < _count; i++) {
        const float distance = AP_Swarming::get_location(_list[i].item).get_distance(loc);
        if (distance < shortest || shortest <= 0) {
            shortest = distance;
            index = i;
        }
    }

    return index;
}

uint32_t AP_SwarmDB::get_item_id(const int32_t index) const
{
    if (!is_valid_index(index)) {
        return 0;
    }
    return AP_Swarming::get_id(_list[index].item);
}

bool AP_SwarmDB::get_item(const int32_t index, mavlink_swarm_vehicle_t &vehicle) const
{
    if (!is_valid_index(index)) {
        return false;
    }

    memcpy(&vehicle, &_list[index].item, sizeof(vehicle));
    return true;
}

bool AP_SwarmDB::get_item(const int32_t index, SwarmDbItem_t &dbItem) const
{
    if (!is_valid_index(index)) {
        return false;
    }

    memcpy(&dbItem, &_list[index], sizeof(dbItem));
    return true;
}

#endif // HAL_AP_SWARMING_ENABLED
