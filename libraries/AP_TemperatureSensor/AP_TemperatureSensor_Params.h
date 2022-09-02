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

#include <AP_Param/AP_Param.h>

class AP_TemperatureSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_TemperatureSensor_Params(void);

    CLASS_NO_COPY(AP_TemperatureSensor_Params);

    AP_Int8 type;                   // AP_TemperatureSensor::Type, 0=disabled, others see frontend enum TYPE
    AP_Int8 bus;                    // I2C bus number
    AP_Int8 bus_address;            // I2C address
    
    AP_Int8 source;                 // AP_TemperatureSensor::Source, library mapping
    AP_Int32 source_id;             // library instance mapping
};
