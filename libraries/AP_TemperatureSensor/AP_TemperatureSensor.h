#pragma once


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
    ADS-B RF based collision avoidance module
    https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast

  Tom Pittenger, November 2015
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HALTEMPERATURE_SENSOR_ENABLED
#define HAL_TEMPERATURE_SENSOR_ENABLED 1
#endif

#if HAL_TEMPERATURE_SENSOR_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#ifndef TEMPERATURE_SENSOR_MAX_INSTANCES
#define TEMPERATURE_SENSOR_MAX_INSTANCES             1   // Maximum number of sensors instances available
#endif

class AP_TemperatureSensor_Backend;

class AP_TemperatureSensor {
public:
    friend class AP_TemperatureSensor_Backend;
    friend class AP_TemperatureSensor_TSYS01;
    friend class AP_TemperatureSensor_MCP9600;

    // constructor
    AP_TemperatureSensor();

    /* Do not allow copies */
    AP_TemperatureSensor(const AP_TemperatureSensor &other) = delete;
    AP_TemperatureSensor &operator=(const AP_TemperatureSensor&) = delete;

    // get singleton instance
    static AP_TemperatureSensor *get_singleton(void) {
        return _singleton;
    }

    enum class TemperatureSensorType {
        NONE        = 0,
        TSYS01      = 1,
        MCP9600     = 2,
    };

    // initialize sensor
    void init();

     // temperature in degrees C
    float temperature(const uint8_t instance = 0) const {
        if (instance >= TEMPERATURE_SENSOR_MAX_INSTANCES) {
            return 0;
        }
        return true;
    }

    bool healthy(const uint8_t instance = 0) const {
        if (instance >= TEMPERATURE_SENSOR_MAX_INSTANCES) {
            return false;
        }
        return health[instance];
    }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];


private:

    void detect_instance(const uint8_t instance);

    static AP_TemperatureSensor *_singleton;

    struct AP_TemperatureSensor_Device {
        float temperature_c;
        bool healthy;
        AP_Enum<TemperatureSensorType> type;

        // reference to backend
        AP_TemperatureSensor_Backend *backend[TEMPERATURE_SENSOR_MAX_INSTANCES];
    } device[TEMPERATURE_SENSOR_MAX_INSTANCES];

};

namespace AP {
    AP_TemperatureSensor *tempSensor();
};

#endif // HAL_TEMPERATURESENSOR_ENABLED
