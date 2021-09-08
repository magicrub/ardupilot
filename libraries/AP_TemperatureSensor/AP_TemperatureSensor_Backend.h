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
#pragma once

#include "AP_TemperatureSensor.h"

#if HAL_TEMPERATURE_SENSOR_ENABLED
class AAP_TemperatureSensor_Backend
{
public:
    // constructor
    AAP_TemperatureSensor_Backend(AP_TemperatureSensor &frontend, uint8_t instance);

    // static detection function
    static bool detect();

    virtual void update() = 0;

    virtual bool init() { return true; }

protected:

    uint8_t _instance;

    // references
    AAP_TemperatureSensor &_frontend;
};
#endif // HAL_TEMPERATURE_SENSOR_ENABLED
