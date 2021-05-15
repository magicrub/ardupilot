/*
   Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.

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

#include "AP_CursorOnTarget.h"

#if AP_CURSORONTARGET_ENABLED

// treat a uart[0+i] as "chan 100+i". This allows us to use chan 0-8ish for MAVLINK_CHAN tunnel
#define AP_CURSORONTARGET_UARTS_BASE_CHAN 100

#include <AP_Math/definitions.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <stdio.h>
#include <time.h>

extern const AP_HAL::HAL& hal;
AP_CursorOnTarget* AP_CursorOnTarget::_singleton;


const AP_Param::GroupInfo AP_CursorOnTarget::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Cursor on Target Enable/Disable
    // @Description: Cursor on Target enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_CursorOnTarget, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SEND_RATE
    // @DisplayName: Rate to send regular position info
    // @Description: Rate to send regular position info
    // @User: Standard
    // @Units: Hz
    // @Range: 0.1 10
    // @Increment: 0.1
    AP_GROUPINFO("SEND_RATE", 1, AP_CursorOnTarget, _send_rate, 1),

    AP_GROUPEND
};


AP_CursorOnTarget::AP_CursorOnTarget()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many AP_CursorOnTargets");
#endif
        return;
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Lazy init
void AP_CursorOnTarget::init()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(_uart); i++) {
        _uart[i] = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CursorOnTarget, i);
        if (_uart[i] == nullptr) {
            break;
        }
        _num_outputs++;
    }
}

void AP_CursorOnTarget::update()
{
    if (!_enabled) {
        return;
    }
    if (!_initialized) {
        init();
        _initialized = true;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // only send at 10Hz
    if ((now_ms - _last_run_ms) < 100) {
        return;
    }
    _last_run_ms = now_ms;
}

void AP_CursorOnTarget::parse_bytes(const uint8_t chan, const uint8_t* data, uint32_t len)
{
    if (!_enabled) {
        return;
    }
    while (len--) {
        parse_byte(chan, *data++);
    }
}

void AP_CursorOnTarget::parse_byte(const uint8_t chan, const uint8_t data)
{
    if (!_enabled) {
        return;
    }
}

#endif // AP_CURSORONTARGET_ENABLED

namespace AP {
AP_CursorOnTarget *CursorOnTarget()
{
    return AP_CursorOnTarget::get_singleton();
}
};

