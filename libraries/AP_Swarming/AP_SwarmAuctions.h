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
#include "AP_SwarmDB.h"

#if HAL_AP_SWARMING_ENABLED
#include <vector>

using namespace std;

class AP_SwarmAuctions {
public:

    struct SwarmAuctionItem_t {
        AP_SwarmDB::SwarmDbItem_t dbItem;
        float sort_criteria;
    };

    // constructor
    AP_SwarmAuctions();

    void init();

    // periodic update to remove stale vehicles
    void update(); 

    void sort_list_by_distance_to(const Location &loc, vector<SwarmAuctionItem_t> list) const;

    void sort_list_by_distance_to_me();
    void sort_list_by_distance_to(const Location &loc);
    void sort_list_by_effective_radius();

    static bool compare_sort_criteria(SwarmAuctionItem_t a, SwarmAuctionItem_t b) { return (a.sort_criteria < b.sort_criteria); }
    static bool compare_sort_effective_radius(SwarmAuctionItem_t a, SwarmAuctionItem_t b) { return (a.dbItem.item.effective_radius < b.dbItem.item.effective_radius); }

protected:
    struct AuctionBid_t {
        AP_Int32    sender_id;
        AP_Float    bid;
    };

    struct CostFunction_t {
        AP_Float    distance;
        AP_Float    batt_remaining;
        //TODO: other factors
    };

    void sync_db_sorted_list();
    void sync_db(vector<SwarmAuctionItem_t> list) const;

    std::vector<SwarmAuctionItem_t> _sorted_list;

    AP_ExpandingArray<AuctionBid_t> _bid_list {1};
    uint16_t _bid_count;

    //desired_state_->vel() << 0, 0, 0;
    //desired_state_->quat().set(0, 0, state_->quat().yaw());
    //desired_state_->pos() = Eigen::Vector3d::UnitZ()*state_->pos()(2);

    //struct {
    //    AP_Int8     bid_type;
    //    AP_Float    distance;
        //AP_Float    batt_remaining;
    //    AP_Int32    id_target;
    //} _params;
    
    uint32_t _last_update_ms;
};

#endif
