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

#include "AP_ADSB_Backend.h"

#if HAL_ADSB_ENABLED
#include <GCS_MAVLink/GCS_config.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if defined(HAL_BUILD_AP_PERIPH)
#include "../Tools/AP_Periph/AP_Periph.h"
extern const AP_HAL::HAL &hal;
#endif

/*
  base class constructor.
*/
AP_ADSB_Backend::AP_ADSB_Backend(AP_ADSB &frontend, uint8_t instance) :
    _frontend(frontend),
    _instance(instance)
{
}

// Init, called once after class is constructed
bool AP_ADSB_Backend::init()
{
#if HAL_GCS_ENABLED
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    return (_port != nullptr);
#elif defined(HAL_BUILD_AP_PERIPH)
    _port = hal.serial(periph.g.adsb_port);
    if (_port == nullptr) {
        return false;
    }
    _frontend.status_msg_received(false);
    return true;
#endif
}

#endif // HAL_ADSB_ENABLED

