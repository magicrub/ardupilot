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

// backend driver for EagleTree airspeed sensor
#include <AP_HAL/I2CDevice.h>
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_EagleTree : public AP_Airspeed_Backend
{
public:

    AP_Airspeed_EagleTree(AP_Airspeed &frontend, uint8_t _instance);
    ~AP_Airspeed_EagleTree(void) {}
    
    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure_result) override;

    // temperature not available
    bool get_temperature(float &temperature) override { return false; }

private:
    void timer();
    void measure();
    void collect();

    float pressure;
    float pressure_sum;
    uint32_t press_count;
    
    uint32_t last_sample_time_ms;
    uint32_t measurement_started_ms;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
};
