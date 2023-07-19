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
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    return (_port != nullptr);
}

#endif // HAL_ADSB_ENABLED

