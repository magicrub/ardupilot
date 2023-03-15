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

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_NMEA_XML.h"

#if AP_NMEA_XML_ENABLED

#include <AP_Math/definitions.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
// #include <time.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NMEA_XML::var_info[] = {

    // @Param: POS_RATE
    // @DisplayName: NMEA/XML Position rate
    // @Description: NMEA/XML Position rate
    // @Units: Hz
    // @Range: 0 50
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("POS_RATE", 1, AP_NMEA_XML, _pos.interval_hz, 1.0f),

    // @Param: ALT_RATE
    // @DisplayName: NMEA/XML Altitude rate
    // @Description: NMEA/XML Altitude rate
    // @Units: Hz
    // @Range: 0 50
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("ALT_RATE", 2, AP_NMEA_XML, _alt.interval_hz, 4.0f),

    // @Param: VEL_RATE
    // @DisplayName: NMEA/XML Velocity rate
    // @Description: NMEA/XML Velocity rate
    // @Units: Hz
    // @Range: 0 50
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("VEL_RATE", 3, AP_NMEA_XML, _vel.interval_hz, 4.0f),

    // @Param: POS_ACC
    // @DisplayName: NMEA/XML Position Accuracy Enable
    // @Description: NMEA/XML Position Accuracy Enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("POS_ACC", 4, AP_NMEA_XML, _pos.accuracy_enabled, 0),

    // @Param: VEL_ACC
    // @DisplayName: NMEA/XML Velocity Accuracy Enable
    // @Description: NMEA/XML Velocity Accuracy Enable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VEL_ACC", 5, AP_NMEA_XML, _vel.accuracy_enabled, 0),

    AP_GROUPEND
};

void AP_NMEA_XML::update()
{
    if (_uart == nullptr) {
        if (!_init_done) {
            _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NMEA_XML, 0);
            _init_done = true;
        }
        return;
    }

    _pos.interval_ms = (_pos.interval_hz <= 0) ? 0 : 1000.0/_pos.interval_hz;
    _vel.interval_ms = (_vel.interval_hz <= 0) ? 0 : 1000.0/_vel.interval_hz;
    _alt.interval_ms = (_alt.interval_hz <= 0) ? 0 : 1000.0/_alt.interval_hz;

    const uint32_t now_ms = AP_HAL::millis();

    if ((_pos.interval_ms == 0 || (now_ms - _pos.last_ms <= _pos.interval_ms)) &&
        (_vel.interval_ms == 0 || (now_ms - _vel.last_ms <= _vel.interval_ms)) &&
        (_alt.interval_ms == 0 || (now_ms - _alt.last_ms <= _alt.interval_ms)))
    {
        // none of the intervals are ready, come back again..
        return;
    }

    const uint32_t space_required = 60 + 30 + 60; // POS + ALT + VEL
    uint64_t time_usec;
    Location loc;
    Vector3f vel;
    int32_t alt_agl_cm;
    if (_uart->txspace() < space_required ||
        !AP::rtc().get_utc_usec(time_usec) ||
        !AP::ahrs().get_position(loc) ||
        !AP::ahrs().get_velocity_NED(vel) ||
        !loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_agl_cm))
    {
        // we're missing some data. Lets not bother sending anything at all until everything is ready
        return;
    }

    const double time_sec = time_usec / 1000000.0;
    const char* ending = " />\r\n";

    if (_pos.interval_ms > 0 && (now_ms - _pos.last_ms > _pos.interval_ms)) {
        _pos.last_ms = now_ms;

        //<POS tod=“14110.112” lat=”-34.6649” lon=”115.32371862” pu=”1000”/>
        _uart->printf("<POS tod=\"%.3f\" lat=\"%.7f\" lon=\"%.7f\"",
                        time_sec,
                        loc.lat * 1.0e-7f,
                        loc.lng * 1.0e-7f);

        if (_pos.accuracy_enabled) {
            const int32_t pu = 1000;
            _uart->printf(" pu=\"%d\"", (int)pu);
        }

        _uart->write(ending);
    }


    if (_alt.interval_ms > 0 && (now_ms - _alt.last_ms > _alt.interval_ms)) {
        _alt.last_ms = now_ms;

        const int32_t alt_AGL_ft = alt_agl_cm * 0.01f * (1.0f/FEET_TO_METERS);

        // <ALT tod=“10229.417” alt=”18450” />
        _uart->printf("<ALT tod=\"%.3f\" alt=\"%d\"",
                        time_sec,
                        (int)alt_AGL_ft);
        _uart->write(ending);
    }

    if (_vel.interval_ms > 0 && (now_ms - _vel.last_ms > _vel.interval_ms)) {
        _vel.last_ms = now_ms;

        // <VEL tod=“5119.463” vn=”113.367” ve=”97.215” vd=”6.7774” vu=”0.967” />
        _uart->printf("<VEL tod=\"%.3f\" vn=\"%.3f\" ve=\"%.3f\" vd=\"%.3f\"",
                        time_sec,
                        (double)vel.x,
                        (double)vel.y,
                        (double)vel.z);
                        
        if (_vel.accuracy_enabled) {
            const float vu = 0.967;
            _uart->printf(" vu=\"%.3f\"", (double)vu);
        }
        _uart->write(ending);
    }
}

#endif  // AP_NMEA_XML_ENABLED
