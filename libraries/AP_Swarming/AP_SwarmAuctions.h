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

   Author: Tom Pittenger and Michael Day
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
//#include "AP_SwarmDB.h"

#include <vector>
#include <map>
#include <stdio.h>

using namespace std;

class AP_SwarmAuctions {
public:

    struct SwarmAuctionItem_t {
        mavlink_swarm_vehicle_t vehicle;
        float       sort_criteria;  // used by the sort() callback as sort criteria


        // Bidding
        float       bid;
        float       distance;
        Location    loc;
        float       batt_remaining;

        void init() {
            sort_criteria = 0;
            bid = -1;
            distance = -1;
            loc = Location();
            batt_remaining = 0;
        }

        bool has_bidded() const { return bid >= 0; }

        void init_bidding() {
            bid = -1;
            distance = -1;
            loc = Location();
        }
    };

    // constructor
    AP_SwarmAuctions();

    // periodic update to remove stale vehicles
    void update(); 

    static void sort_list_by_distance_to(vector<SwarmAuctionItem_t> list, const Location &loc);
    static void sort_list_by_effective_radius(vector<SwarmAuctionItem_t> list);

    static bool compare_sort_criteria(SwarmAuctionItem_t a, SwarmAuctionItem_t b) { return (a.sort_criteria < b.sort_criteria); }
    static bool compare_sort_effective_radius(SwarmAuctionItem_t a, SwarmAuctionItem_t b) { return (a.vehicle.effective_radius < b.vehicle.effective_radius); }

protected:
    static bool all_bidding_is_complete(const vector<SwarmAuctionItem_t> list, const uint32_t skip_this_id = 0);
    static bool all_locations_are_valid_and_unique(const vector<SwarmAuctionItem_t> list);

    static void generate_target_locations(vector<Location> loc, uint32_t count);

    static bool find_nearest_target_loc(SwarmAuctionItem_t &item, const vector<Location> target_locs, uint32_t skip_count);

    static void assign_locations_by_distance_mins(vector<SwarmAuctionItem_t> list, const vector<Location> target_locs);

    //void sync_db_sorted_list();
    void copy_db_to(vector<SwarmAuctionItem_t> list) const;

    std::vector<SwarmAuctionItem_t> _sorted_list;
    std::vector<Location> _target_locs;
    
    uint32_t _last_assign_target_ms;
    uint32_t _last_update_ms;
};

//#endif
