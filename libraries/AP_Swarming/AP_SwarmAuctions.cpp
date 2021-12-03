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
#include <algorithm>

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

    // Auction stuff at 1Hz



    // AP_SwarmROI &roi = AP::swarm()->get_roi();
    // (void)roi.get_count();

    // AP_SwarmDB &db = AP::swarm()->get_db();
    // (void)db.get_count();




    // sort vector<>_sorted_list by distance to my vehicle from ap::swarming().get_vehicle() which is updated at 50Hz
    sort_list_by_distance_to_me();



    // sort an arbitrary list to an arbitrary locaction
    vector<SwarmAuctionItem_t> my_list;
    Location loc; loc.alt = 123;
    sort_list_by_distance_to(loc, my_list);



    

    for (uint16_t i=0; i<_bid_count; i++) {
        //TODO: take battery remaining into account for bid as well
        //TODO: return sorted indexes
    }


    //void assign_new_target() { assign_new_target(get_location_target(_my_vehicle)); }
}

void AP_SwarmAuctions::sync_db_sorted_list()
{
    AP_SwarmDB &db = AP::swarm()->get_db();
    _sorted_list.resize(db.get_count());

    for (uint16_t i=0; i< _sorted_list.size(); i++) {
        db.get_item(i, _sorted_list[i].dbItem);
    }
}
void AP_SwarmAuctions::sync_db(vector<SwarmAuctionItem_t> list) const
{
    AP_SwarmDB &db = AP::swarm()->get_db();
    list.resize(db.get_count());

    for (uint16_t i=0; i< list.size(); i++) {
        db.get_item(i, list[i].dbItem);
    }
}

void AP_SwarmAuctions::sort_list_by_distance_to_me()
{
    // get my location
    mavlink_swarm_vehicle_t me = AP::swarm()->get_vehicle();
    Location my_loc = AP_Swarming::get_location(me);

    sort_list_by_distance_to(my_loc);
}

void AP_SwarmAuctions::sort_list_by_distance_to(const Location &loc)
{
    sync_db_sorted_list();
    for (uint16_t i = 0; i < _sorted_list.size(); i++) {
        _sorted_list[i].sort_criteria = AP_Swarming::get_location(_sorted_list[i].dbItem.item).get_distance(loc);
    }
    std::sort(std::begin(_sorted_list), std::end(_sorted_list), compare_sort_criteria);
}

void AP_SwarmAuctions::sort_list_by_effective_radius()
{
    sync_db_sorted_list();
    std::sort(std::begin(_sorted_list), std::end(_sorted_list), compare_sort_effective_radius);
}


//  SwarmAuctionItem_t AP_SwarmAuctions::get_nearest_db_item_to_me()
// {
//     // Location loc = tmp_location[instance];
//     // loc.lat = _RGPJ[instance].lat;
//     // loc.lng = _RGPJ[instance].lng;
//     // loc.alt = _RGPJ[instance].alt;
//     // return loc;

//     // get my location
//     mavlink_swarm_vehicle_t me = AP::swarm()->get_vehicle();
//     Location my_loc = AP_Swarming::get_location(me);

//     // ask db for nearest index
//     int32_t index = db.get_nearest_index(my_loc);
//     mavlink_swarm_vehicle_t db_item;
//     if (db.get_item(index, db_item)) {
        
//     }
// }


void AP_SwarmAuctions::sort_list_by_distance_to(const Location &loc, vector<SwarmAuctionItem_t> list) const
{
    sync_db(list);
    for (uint16_t i = 0; i < list.size(); i++) {
        list[i].sort_criteria = AP_Swarming::get_location(list[i].dbItem.item).get_distance(loc);
    }
    std::sort(std::begin(list), std::end(list), compare_sort_criteria);

}
#endif // HAL_AP_SWARMING_ENABLED
