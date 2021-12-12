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

void AP_SwarmAuctions::init() 
{
    init_ac_to_locs();
}

void AP_SwarmAuctions::init_ac_to_locs() 
{
    // if(_sorted_list.size() > 2) {
    //      //no swarm
    //      return;
    // }

    // _ac_to_location.clear();

    // _ac_to_locs_inited = true;
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
    //int m = 3;
    //int n = 3;
    //float agentMatrix[m][n];

    //introduce inertia into the bidding process to prevent assignment oscillation
    //float bid_factor = 200.0; //TODO: figure out better bid factor via sim

    AP_SwarmROI &roi = AP::swarm()->get_roi();
        //***IDEA: define the center of the swarm based on the centroid of the swarm_ROI 
        //how to get the centroid of the swarm_roi?
        //***Note that this may not work for some concave polygons
    //TODO: figure out where the centroid of the swarm ROI lives
    //(void)roi.get_count();
    //A side effect of calling roi.get_crc32 is we update the crc and centroid
    //TODO: find a better way to do this?
    (void) roi.get_crc32();
    //uint32_t crc32 = roi.get_crc32();
    Vector2l roi_centroid;
    roi.calc_poly_centroid(roi_centroid);

    AP_SwarmDB &db = AP::swarm()->get_db();
    (void)db.get_count();
    const uint32_t ownship_id = db.get_ownship_id();

    //int32_t ownship_index = -1;
    //if (!db.get_ownship_id(ownship_index)) {
        //printf("inside if\n");
        //TODO: where is ownship??
        //printf("setting ownship_id to: %d \n", ); //where is the ownship index??
    //}
    //printf("ownship id: %d \n", ownship_index);

    //db.get_item(  
        //where is THIS aircraft?? //item.effective_radius;

    //int aircraft_assignment_num = 0;
    //int location_assignment_num = 0;

    //+1 because _sorted_list does not include ownship:
    // const int total_swarm_aircraft = _sorted_list.size();
    // printf("Total swarm aircraft: %d\n", total_swarm_aircraft);

    //TODO: store total_locations elsewhere
    uint32_t total_locations = 3;

    // int total_aircraft_assignments = 0;
    // int total_location_assignments = 0;

    //if (! _ac_to_locs_inited) {
    //    init_ac_to_locs();
    //}

    // sort vector<>_sorted_list by distance to my vehicle from ap::swarming().get_vehicle() which is updated at 50Hz
    //AP_SwarmAuctions::SwarmAuctionItem_t next_item;
    //Location next_loc;
    sort_list_by_distance_to_me();

    // for(auto& itr : _sorted_list) {
    //     Location next_loc;
    //     next_loc.lat = itr.dbItem.item.lat;
    //     //printf("%d\n", next_loc.lat);
    // }
    //printf("\n");

    // sort an arbitrary list to an arbitrary locaction
    //vector<SwarmAuctionItem_t> my_list;
    //Location loc; loc.alt = 123;
    //sort_list_by_distance_to(loc, my_list);

    //s is number of backup relays (s = size of subteams)
    //int s = 1; // !!!! can't be 0 because this gets multiplied later on

    const uint32_t total_swarm_aircraft = _sorted_list.size();

    // AuctionBid_t next_bid;
    // (void)next_bid;

    // while (aircraft_assignment_num < total_swarm_aircraft) {
    //     //for (uint16_t i=0; i<_bid_count; i++) {

        for (uint32_t i = 0; i < total_swarm_aircraft; i++) {

            SwarmAuctionItem_t auctionItem = _sorted_list[i];
            
            const uint32_t id = AP_Swarming::get_id(auctionItem.vehicle);
            if (id == ownship_id) {
                continue;
            }

            uint32_t num_bids_made = 0;

            // Map aircraft to locations:
            for (uint32_t j = 0; j < total_locations; j++) {
                //TODO: implement get_num_assignments
                printf("%d %d, ", i, j);

                // for(int l = 0; l < _sorted_list.size(); l++) {
                //     //IDEA: DEFINE desired ROI locations based on centroid and number of desired locations
                //     //AND Base it on Effective Radii of indivdual aircraft!
                    

                //     //Get next closest location from list
                //     SwarmAuctionItem_t next_loc = _sorted_list[l];
                //     next_loc.dbItem.item;
                // }

                //if (get_num_assignments(agentMatrix[i][j]) < s) {       
                    // next_bid = (s-j) * int(bid_factor);
                    num_bids_made++;
                    // if (num_bids_made == s) {
                    //      break;
                    // }
                //}
            }



            // 7x$10 1x$25
            // holiday thank you

            printf("\n");
            //TODO: iterate through bids and determine auction winners for this round

    //         total_aircraft_assignments++;
    //         total_location_assignments++;
         }
         printf("\n");
        
    //     //TODO: take battery remaining into account for bid as well
    // }

    //void assign_new_target() { assign_new_target(get_location_target(_my_vehicle)); }
}

void AP_SwarmAuctions::sync_db_sorted_list()
{
    AP_SwarmDB &db = AP::swarm()->get_db();
    _sorted_list.resize(db.get_count());

    for (uint16_t i=0; i< _sorted_list.size(); i++) {
        db.get_item(i, _sorted_list[i].vehicle);
    }
}

void AP_SwarmAuctions::sync_db(vector<SwarmAuctionItem_t> list) const
{
    AP_SwarmDB &db = AP::swarm()->get_db();
    list.resize(db.get_count());

    for (uint16_t i=0; i< list.size(); i++) {
        db.get_item(i, list[i].vehicle);
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
        _sorted_list[i].sort_criteria = AP_Swarming::get_location(_sorted_list[i].vehicle).get_distance(loc);
        _sorted_list[i].distance = _sorted_list[i].sort_criteria;
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
        list[i].sort_criteria = AP_Swarming::get_location(list[i].vehicle).get_distance(loc);
        list[i].distance = list[i].sort_criteria;
    }
    std::sort(std::begin(list), std::end(list), compare_sort_criteria);

}
#endif // HAL_AP_SWARMING_ENABLED
