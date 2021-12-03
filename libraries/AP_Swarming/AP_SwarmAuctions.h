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

class AP_SwarmAuctions {
public:

    // constructor
    AP_SwarmAuctions();

    void init();

    // periodic update to remove stale vehicles
    void update(); 

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
