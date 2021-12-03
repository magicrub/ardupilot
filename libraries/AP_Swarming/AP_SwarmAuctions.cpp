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

#include "AP_Swarming.h"

#if HAL_AP_SWARMING_ENABLED
#include "AP_SwarmAuctions.h"

extern const AP_HAL::HAL& hal;

AP_SwarmAuctions::AP_SwarmAuctions()
{//TODO: initalize class
}

// periodic update
void AP_SwarmAuctions::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms < 1000) {
        return;
    }
    _last_update_ms = now_ms;

    AP_SwarmROI &roi = AP::swarm()->get_roi();
    (void)roi.get_count();

    AP_SwarmDB &db = AP::swarm()->get_db();
    (void)db.get_count();
    //int32_t swarm_count = db.get_count();

    //sort distances 

    // TODO: auction stuff at 1Hz
    for (uint16_t i=0; i<_bid_count; i++) {
        //TODO: take battery remaining into account for bid as well

        //db.get_nearest_index
        //TODO: return sorted indexs

        if (db.get_item_no_copy(i)) {
            //db.get_sorted_distances(db.get);
        } 

    }


    //void assign_new_target() { assign_new_target(get_location_target(_my_vehicle)); }
       
}

#endif // HAL_AP_SWARMING_ENABLED
