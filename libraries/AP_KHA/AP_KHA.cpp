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
  temperature calibration library
 */

#include "AP_KHA.h"
#include <stdio.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

#define KHA_DEBUG 0

#if KHA_DEBUG
# define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\r\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define debug(fmt, args ...)
#endif

// table of user settable and learned parameters
const AP_Param::GroupInfo AP_KHA::var_info[] = {

    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_KHA, _param.enabled, 1, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("SER_BCAST", 1, AP_KHA, _param.serial_loopback_broadcast, 0),

    AP_GROUPEND
};

void AP_KHA::init(void)
{
    if (_init.done) {
        return;
    }

    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS;i++) {
        if (_port_loopback[i] == nullptr) {
            _port_loopback[i] = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Loopback, i);
        }
    }

    _init.done = true;
}

void AP_KHA::update(void)
{
    if (!_init.done) {
        init();
        return;
    }

    switch (_param.enabled.get()) {
        default:
        break;
    }

    service_loopback();
}

void AP_KHA::service_loopback()
{
    for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS;i++) {
        if (_port_loopback[i] == nullptr) {
            continue;
        }

        uint32_t nbytes = _port_loopback[i]->available();
        while (nbytes-- > 0) {
            const int16_t data = _port_loopback[i]->read();
            if (data < 0) {
                break;
            }
            
            if (_param.serial_loopback_broadcast) {
                for (uint8_t j=0; j<SERIALMANAGER_NUM_PORTS; j++) {
                    if (_port_loopback[j] == nullptr) {
                        continue;
                    }
                    _port_loopback[j]->write(data+1);
                }
            } else {
                _port_loopback[i]->write(data+1);
            }
        }
    }

}
