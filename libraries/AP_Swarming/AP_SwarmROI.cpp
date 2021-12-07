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
#include "AP_SwarmROI.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

AP_SwarmROI::AP_SwarmROI() 
{
    compute_crc32();
    _crc32_is_calculated = true;
}

//Retuns false if the ROI CRC is new or updated; true otherwise
bool AP_SwarmROI::handle_swarm_ROI(const mavlink_swarm_vehicle_t &this_vehicle)
{
    //if CRC is new or changed
    if(! _crc32_is_calculated || get_crc32() != this_vehicle.ROI_crc) {  
        _crc32 = this_vehicle.ROI_crc;
        _crc32_is_calculated = false;

        //blow the old ROI away
        clear();

        //let us know the CRC is changed or it's the first time we've seen it
        return false;
    }

    return true;
}

bool AP_SwarmROI::add_poly(const Vector2l& point) {
    if (_poly_count >= SWARM_ROI_POLY_MAX_SIZE) {
            return false;
        }
    _poly_count++;

    return set_poly(_poly_count-1, point);
}

bool AP_SwarmROI::set_poly(const uint16_t index, const Vector2l& point) 
{
    if (index >= _poly_count) {
        return false;
    }
    _poly[index].x = point.x;
    _poly[index].y = point.y;
    _crc32_is_calculated = false;

    return true;
}

bool AP_SwarmROI::calc_poly_centroid(Vector2l &centroid) 
{
    if(_poly_count < 3) {
        //can't have a polygon with < 3 sides
        return false;
    }

    Vector2l min_loc;
    if (min_loc.is_zero()) {
        return false;
    }
    Vector2l max_loc = min_loc;
    for (uint8_t i=0; i<_poly_count; i++) {
        Vector2l new_loc = _poly[i];

        if (new_loc.is_zero()) {
            return false;
        }
        if (new_loc.x < min_loc.x) {
            min_loc.x = new_loc.x;
        }
        if (new_loc.y < min_loc.y) {
            min_loc.y = new_loc.y;
        }
        if (new_loc.x > max_loc.x) {
            max_loc.x = new_loc.x;
        }
        if (new_loc.y > max_loc.y) {
            max_loc.y = new_loc.y;
        }
    }

    centroid.x = (min_loc.x / 2) + (max_loc.x / 2);
    centroid.y = (min_loc.y / 2) + (max_loc.y / 2);

    return true;
}

bool AP_SwarmROI::breached(const Vector2l &pos) const
{
    Location loc;
    loc.lat = pos.x;
    loc.lng = pos.y;
    return breached(loc);
}
bool AP_SwarmROI::breached(const Location &loc) const
{
    if (get_count() == 0) {
        return false;
    }

    const Vector2l pos = Vector2l(loc.lat,loc.lng);
    if (!Polygon_outside(pos, _poly, _poly_count)) {
        return false;
    }

    for (uint16_t i=0; i<_circles_count; i++) {
        Location circle_center;
        circle_center.lat = _circles[i].pt.x;
        circle_center.lng = _circles[i].pt.y;
        if (loc.get_distance(circle_center) < _circles[i].radius) {
            return false;
        }
    }

    return true;
}

void AP_SwarmROI::compute_crc32()
{
    uint32_t crc = 0;
    calc_poly_centroid(_poly_centroid);

    for (uint16_t i=0; i< _poly_count; i++) {
        crc = crc_crc32(crc, (uint8_t*)&_poly[i], sizeof(_poly[0]));
    }
    for (uint16_t i=0; i< _circles_count; i++) {
        crc = crc_crc32(crc, (uint8_t*)&_circles[i], sizeof(_circles[0]));
    }
    if (crc == 0) {
        // don't allow crc == 0
        crc = 1;
    }
    _crc32 = crc;
    _crc32_is_calculated = true;
}


#endif // HAL_AP_SWARMING_ENABLED
