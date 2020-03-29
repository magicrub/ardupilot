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
    AP_OpenDroneID.cpp

    Open Drone ID, an implementation of the Remote ID system
    https://www.opendroneid.org/
    http://github.com/opendroneid/

    Tom Pittenger, April 2020
*/

#include "AP_OpenDroneID.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>


extern const AP_HAL::HAL& hal;
AP_OpenDroneID *AP_OpenDroneID::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable OpenDroneID Broadcast
    // @Description: Enable OpenDroneID Broadcast
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE",     0, AP_OpenDroneID, _enabled,    0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_OpenDroneID must be singleton");
    }
    _singleton = this;
}

/*
 * periodic update
 */
void AP_OpenDroneID::update(void)
{
    if (!_enabled) {
        if (_initialized) {
            deinit();
        }
        return;
    }
    if (!_initialized) {
        init();
        return; // return in case we failed the init
    }

    // TODO: do stuff
}

/*
 * Initialization
 */
void AP_OpenDroneID::init(void)
{
    _initialized = true;

    if (!_initialized) {
        // if we failed the init, disable the system so we don't constantly re-init and notify GCS
        _enabled = false;
    }
}

/*
 * De-Initialization
 */
void AP_OpenDroneID::deinit(void)
{
    _initialized = false;
}

AP_OpenDroneID *AP::OpenDroneID()
{
    return AP_OpenDroneID::get_singleton();
}

