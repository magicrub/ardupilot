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

void AP_SwarmROI::handle_swarm_ROI(const mavlink_message_t &msg)
{
    // TODO
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
