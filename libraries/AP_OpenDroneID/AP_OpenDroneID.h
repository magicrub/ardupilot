#pragma once

/*
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
*/

/*
    Open Drone ID, an implementation of the Remote ID system
    https://www.opendroneid.org/
    http://github.com/opendroneid/

    Tom Pittenger, April 2020
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_OpenDroneID {
public:
    // constructor
    AP_OpenDroneID();

    /* Do not allow copies */
    AP_OpenDroneID(const AP_OpenDroneID &other) = delete;
    AP_OpenDroneID &operator=(const AP_OpenDroneID&) = delete;

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    // periodic task that maintains vehicle_list
    void update(void);

    // get singleton instance
    static AP_OpenDroneID *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_OpenDroneID *_singleton;

    void init();
    void deinit();

    AP_Int8     _enabled;
    bool        _initialized;


};

namespace AP {
    AP_OpenDroneID *OpenDroneID();
};
