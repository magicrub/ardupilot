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

class AP_SwarmAuctions {
public:

    // constructor
    AP_SwarmAuctions() { }

    // periodic update to remove stale vehicles
    void update();

private:
    uint32_t _last_update_ms;
};
