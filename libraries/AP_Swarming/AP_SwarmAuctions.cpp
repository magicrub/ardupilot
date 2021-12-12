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
#include "AP_SwarmDB.h"
#include <algorithm>

extern const AP_HAL::HAL& hal;

AP_SwarmAuctions::AP_SwarmAuctions()
{
    _sorted_list.reserve(SWARM_DB_LIST_MAX_SIZE);
}


// periodic update
void AP_SwarmAuctions::update()
{
    const uint32_t now_ms = AP_HAL::millis();
     if (now_ms - _last_update_ms < 1000) {
        return;
    }
    _last_update_ms = now_ms;

    // Vector2l roi_centroid;
    // roi.calc_poly_centroid(roi_centroid);

    // AP_SwarmDB &db = AP::swarm()->get_db();
    // const uint32_t ownship_id = db.get_ownship_id();
    // (void)ownship_id;

    // sync db to _sorted_list
    copy_db_to(_sorted_list);

    generate_target_locations(_target_locs, _sorted_list.size());

    // sort _sorted_list by 
    // mavlink_swarm_vehicle_t my_vehicle = AP::swarm()->get_vehicle();
    // const Location my_loc = AP_Swarming::get_location(my_vehicle);


    // may result in duplicates
    assign_locations_by_distance_mins(_sorted_list, _target_locs);



    if (now_ms - _last_assign_target_ms >= 5000) {
        _last_assign_target_ms = now_ms;
        AP::swarm()->assign_new_target(_sorted_list[0].loc);
    }


    // for (uint32_t i = 0; i < _sorted_list.size(); i++) {
    //     _sorted_list[i].init_bidding();
    // }

    // uint32_t bit_loop_count = 0;

    // //do
    // {
    //     for (uint32_t i = 0; i < _sorted_list.size(); i++) {
            
    //         // if (AP_Swarming::get_id(_sorted_list[i].vehicle) == ownship_id) {
    //         //     // skip over ourself
    //         //     _sorted_list[i].distance = FLT_MAX;
    //         //     _sorted_list[i].loc;
    //         //     continue;
    //         // }

    //         // HACK for small sized known-fixed-geometry
    //         const uint32_t total_locations = MIN(_sorted_list.size(), _target_locs.size());

    //         // Map aircraft to locations:
    //         if (!_sorted_list[i].has_bidded() && !find_nearest_target_loc_to(_sorted_list[i], _target_locations, bit_loop_count)) {
    //             // no nearest found. This should be unreachable but is handled just in case
    //             _sorted_list[i].bid = -1;
    //             printf("find_nearest_target_loc_to -- BAD\n");
    //         }
    //     } // for i (all aircraft except ourselves)

    //     if (!all_locations_are_valid_and_unique(_sorted_list)) {
    //         // well oh crap, gotta fix that. 

    //     }
    // }
    // //while (all_locations_are_valid_and_unique(_sorted_list) && (bit_loop_count++ < 100));

    // if (bit_loop_count > _target_locs.size() + 5) {
    //     printf("bit_loop_count got big %d\n", bit_loop_count);
    // }

        // TODO: resolve conflicting bids. Notify other plane of conflicting bids


            // for(int l = 0; l < _sorted_list.size(); l++) {
            //     //IDEA: DEFINE desired ROI locations based on centroid and number of desired locations
            //     //AND Base it on Effective Radii of individual aircraft!

            //     //Get next closest location from list
            //     SwarmAuctionItem_t next_loc = _sorted_list[l];
            //     next_loc.dbItem.item;
            // }

            //if (get_num_assignments(agentMatrix[i][j]) < s) {       
                // next_bid = (s-j) * int(bid_factor);
                //TODO: Bid factor
                
                // TODO: aircraft subteams?
                // if (num_bids_made == s) {
                //      break;
                // }
            //}
        //}

        //TODO: iterate through bids and determine auction winners for this round       
        
        //TODO: take battery remaining into account for bid as well
        //TODO: take aircraft capability into account
    // }

    //void assign_new_target() { assign_new_target(get_location_target(_my_vehicle)); }
}



bool AP_SwarmAuctions::all_locations_are_valid_and_unique(const vector<SwarmAuctionItem_t> list)
{
    const uint32_t list_size = list.size();

    if (list_size == 0) {
        return false;
    } else if (list_size == 1) {
        return !list[0].loc.is_zero();
    }

    std::vector<Location> locs;
    locs.reserve(list_size);

    for (uint32_t i = 0; i < list_size; i++) {
        if (list[i].loc.is_zero()) {
            // list item is NOT valid
            return false;
        }
        for (uint32_t j = 0; j < locs.size(); j++) {
            if (list[i].loc == locs[j]) {
                // list item already found in locs from previous loop iteration. NOT unique
                return false;
            }
        }
        locs.push_back(list[i].loc);
    }
    return true;
}

bool AP_SwarmAuctions::find_nearest_target_loc(SwarmAuctionItem_t& item, const vector<Location> target_locs, uint32_t skip_count)
{
    bool success = false;

    for (uint32_t j = 0; j < target_locs.size(); j++) {
        const Location loc = target_locs[j];
        if (loc == item.loc) {
            item.distance = 0;
            item.loc = loc;
            continue;
        }
        const float distance = AP_Swarming::get_location(item.vehicle).get_distance(loc);

        if (j == 0 || distance < item.distance) {
            if (skip_count > 0) {
                skip_count--;
            } else {
                item.distance = distance;
                item.loc = loc;
                success = true;
            }
        }
    }
    return success;
}

bool AP_SwarmAuctions::all_bidding_is_complete(const vector<SwarmAuctionItem_t> list, const uint32_t skip_this_id)
{
    for (uint32_t i=0; i< list.size(); i++) {
        if (skip_this_id > 0 && AP_Swarming::get_id(list[i].vehicle) == skip_this_id) {
            // skip this id
            continue;
        }

        if (list[i].bid < 0) {
            return false;
        }
    }
    return true;
}


void AP_SwarmAuctions::copy_db_to(vector<SwarmAuctionItem_t> list) const
{
    AP_SwarmDB &db = AP::swarm()->get_db();
    const uint32_t count = db.get_count();
    list.resize(count);

    for (uint32_t i=0; i< count; i++) {
        list[i].init();
        db.get_item(i, list[i].vehicle);
    }
}

// void AP_SwarmAuctions::sort_list_by_distance_to(const Location &loc)
// {
//     for (uint16_t i = 0; i < _sorted_list.size(); i++) {
//         _sorted_list[i].sort_criteria = AP_Swarming::get_location(_sorted_list[i].vehicle).get_distance(loc);
//         _sorted_list[i].distance = _sorted_list[i].sort_criteria;
//     }
//     std::sort(std::begin(_sorted_list), std::end(_sorted_list), compare_sort_criteria);
// }

void AP_SwarmAuctions::assign_locations_by_distance_mins(vector<SwarmAuctionItem_t> list, const vector<Location> target_locs)
{
    for (uint16_t i_list = 0; i_list < list.size(); i_list++) {
        for (uint16_t j_locs = 0; j_locs < target_locs.size(); j_locs++) {

            const Location list_vehicle = AP_Swarming::get_location(list[i_list].vehicle);
            const float distance = list_vehicle.get_distance(target_locs[j_locs]);

            if (j_locs == 0 || (list[i_list].distance < distance)) {
                list[i_list].distance = distance;
                list[i_list].loc = target_locs[j_locs];
            }
        }
    }
}

void AP_SwarmAuctions::sort_list_by_effective_radius(vector<SwarmAuctionItem_t> list)
{
    std::sort(std::begin(list), std::end(list), compare_sort_effective_radius);
}

void AP_SwarmAuctions::sort_list_by_distance_to(vector<SwarmAuctionItem_t> list, const Location &loc)
{
    for (uint16_t i = 0; i < list.size(); i++) {
        list[i].distance = AP_Swarming::get_location(list[i].vehicle).get_distance(loc);
        list[i].sort_criteria = list[i].distance;
    }
    std::sort(std::begin(list), std::end(list), compare_sort_criteria);
}

void AP_SwarmAuctions::generate_target_locations(vector<Location> loc, uint32_t count)
{
    // AP_SwarmROI &roi = AP::swarm()->get_roi();
    // const uint32_t crc32 = roi.get_crc32();
    // (void)crc32;

    // TODO: change hard-coded Locations with dynamically generated ones considering:
    // - Roi dimenstions
    // - count

    // if (loc.size == count) {
    //     return;
    // }

    const uint32_t static_target_locactions_max_count = 5;
    const Location static_target_locactions[][5] = {
        {Location(39.1, -122.1), Location(), Location(), Location(), Location()},
        {Location(39.1, -122.1), Location(39.2, -122.2), Location(), Location(), Location()},
        {Location(39.1, -122.1), Location(39.2, -122.2), Location(39.3, -122.3), Location(), Location()},
        {Location(39.1, -122.1), Location(39.2, -122.2), Location(39.3, -122.3), Location(39.4, -122.4), Location()},
        {Location(39.1, -122.1), Location(39.2, -122.2), Location(39.3, -122.3), Location(39.4, -122.4), Location(39.5, -122.5)},
    };

    count = MIN(count, static_target_locactions_max_count);

    loc.clear();
    loc.resize(count);

    for (uint32_t i=0; i<count; i++) {
        loc.push_back(static_target_locactions[i][count]);
    }
}

#endif // HAL_AP_SWARMING_ENABLED
