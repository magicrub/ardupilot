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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define SWARM_ROI_POLY_MAX_SIZE               200
#define SWARM_ROI_CIRCLES_MAX_SIZE            20

class AP_SwarmROI {
public:

    // constructor
    AP_SwarmROI();

    struct Circle {
    public:
        Circle() { }
        Circle(Vector2l _pt, float _radius) : pt(_pt), radius(_radius) { }
        Circle(double x, double y, float _radius) : pt(Vector2l(x*1e7,y*1e7)), radius(_radius) { }
        Circle(float x, float y, float _radius) : pt(Vector2l(x*1e7,y*1e7)), radius(_radius) { }
        Circle(int32_t x, int32_t y, float _radius) : pt(Vector2l(x,y)), radius(_radius) { }
        Vector2l pt;
        float radius; // in meters
    };

    uint32_t get_count() const { return _poly_count + _circles_count; }

    bool add(const Location &loc) { return add_poly(loc); }
    bool add(const Vector2l &point) { return add_poly(point); }
    bool add(const Vector2f &point) { return add_poly(Vector2l(point.x*1e7,point.y*1e7)); }
    bool add_poly(const Location &loc) { return add_poly(Vector2l(loc.lat, loc.lng)); }
    bool add_poly(const Vector2l& point);
    bool calc_poly_centroid(Vector2l &centroid); 
    bool set_poly(const uint16_t index, const Vector2l& point);

    bool add(const Circle &circle) { return add_circle(circle); }
    bool add_circle(const Circle &circle) {
        if (_circles_count >= SWARM_ROI_CIRCLES_MAX_SIZE || !_circles.expand_to_hold(_circles_count+1)) {
            return false;
        }
        _circles_count++;
        return set_circle(_circles_count-1, circle);
    }

    bool set_circle(const uint16_t index, const Circle& circle) {
        if (index >= _circles_count) {
            return false;
        }
        _circles[index].pt.x = circle.pt.x;
        _circles[index].pt.y = circle.pt.y;
        _circles[index].radius = circle.radius;
        _crc32_is_calculated = false;
        return true;
    };

    uint32_t get_crc32() {
        if (!_crc32_is_calculated) {
            compute_crc32();
        }
        return _crc32;
    }

    void clear() { _poly_count = 0; _circles_count = 0; _crc32_is_calculated = false; }

    bool handle_swarm_ROI(const mavlink_swarm_vehicle_t &swarm_vehicle);

    bool breached(const Location &loc) const;
    bool breached(const Vector2l &pos) const;
    bool breached(const int32_t lat, const int32_t lng) const { return breached(Vector2l(lat,lng)); }

private:

    void compute_crc32();

    uint32_t _crc32;
    bool _crc32_is_calculated;

    //AP_ExpandingArray<Vector2l> _poly {5};
    Vector2l _poly[SWARM_ROI_POLY_MAX_SIZE];
    Vector2l _poly_centroid;
    uint16_t _poly_count;

    AP_ExpandingArray<Circle> _circles {1};
    uint16_t _circles_count;
};
