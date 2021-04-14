/*
   Copyright (C) 2021 Kraus Hamdani Aerospace. All rights reserved.
  
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

   ArduPilot in-house Eelectronic Speed Controller (ESC) for Motors
   by David Maye <David.Maye@krausaerospace.com> and Tom Pittenger <Tom.Pittenger@krausaerospace.com>
*/

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_AP_ESC_ENABLED
#define HAL_AP_ESC_ENABLED (!defined(HAL_BUILD_AP_PERIPH) && !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024) || (defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_AP_ESC))
#endif

#if HAL_AP_ESC_ENABLED

#include <AP_Param/AP_Param.h>

class AP_ESC
{
public:
    //constructor
    AP_ESC() {
        singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_ESC(const AP_ESC &other) = delete;
    AP_ESC &operator=(const AP_ESC&) = delete;

    enum class EscType : uint8_t {
        Disabled            = 0,
        FOC                 = 1,
    };

    // get singleton instance
    static AP_ESC *get_singleton(void) {
        return singleton;
    }

    static const struct AP_Param::GroupInfo var_info[];

    // run-once init
    void init();

    // update slow, usually 10Hz in ap_periph/vehicle thread
    void update();

    // update fast (1Kz) in ap_periph/vehicle thread
    void update_fast();


    // handle incoming RawCommand UAVCAN packets
    void handle_can_rx(uint8_t source_id, const int16_t *rc, uint8_t num_channels);

private:
    static AP_ESC *singleton;

    // tick - main call in it's own thread running as fast as it can ( >1kHz )
    void tick(void);

    uint32_t last_tick1Hz_ms;
    bool initialized;

    AP_Enum<EscType> escType;
    AP_Float debug1;
};

namespace AP {
    AP_ESC *esc();
};

 #endif // HAL_AP_ESC_ENABLED
