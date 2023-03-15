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


   Author: Tom Pittenger

 */

#pragma once

#include "AP_NMEA_XML_config.h"

#if AP_NMEA_XML_ENABLED

#include <AP_Param/AP_Param.h>

class AP_NMEA_XML {

public:

    AP_NMEA_XML() {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_NMEA_XML);

    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_HAL::UARTDriver* _uart;

    bool _init_done;

    struct {
        AP_Float interval_hz;
        uint32_t interval_ms;
        uint32_t last_ms;
        AP_Int8 accuracy_enabled;
    } _pos;

    struct {
        AP_Float interval_hz;
        uint32_t interval_ms;
        uint32_t last_ms;
        AP_Int8 accuracy_enabled;
    } _vel;

    struct {
        AP_Float interval_hz;
        uint32_t interval_ms;
        uint32_t last_ms;
    } _alt;

};

#endif  // !AP_NMEA_XML_ENABLED
