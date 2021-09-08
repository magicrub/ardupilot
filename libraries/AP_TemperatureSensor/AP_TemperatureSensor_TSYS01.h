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

#include "AP_TemperatureSensor_Backend.h"

#ifndef HAL_TEMPERATURE_SENSOR_TSYS01_ENABLED
#define HAL_TEMPERATURE_SENSOR_TSYS01_ENABLED HAL_TEMPERATURE_SENSOR_ENABLED
#endif

#if HAL_TEMPERATURE_SENSOR_TSYS01_ENABLED
class AP_TemperatureSensor_TSYS01 : public AP_TemperatureSensor_Backend {
public:
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

    void update() override;

    // static detection function
    static bool detect();


    bool init(uint8_t bus);
    float temperature(void) const { return _temperature; } // temperature in degrees C
    bool healthy(void) const { // do we have a valid temperature reading?
        return _healthy;
    }

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;


private:

    float _temperature; // degrees C
    bool _healthy; // we have a valid temperature reading to report
    uint16_t _k[5]; // internal calibration for temperature calculation
    bool _reset(void) const; // reset device
    bool _read_prom(void); // read (relevant) internal calibration registers into _k
    bool _convert(void) const; // begin an ADC conversion (min:7.40ms typ:8.22ms max:9.04ms)
    uint32_t _read_adc(void) const;
    uint16_t _read_prom_word(uint8_t word) const;
    void _timer(void); // update the temperature, called at 20Hz
    void _calculate(uint32_t adc); // calculate temperature using adc reading and internal calibration

};
#endif // HAL_TEMPERATURE_SENSOR_TSYS01_ENABLED

