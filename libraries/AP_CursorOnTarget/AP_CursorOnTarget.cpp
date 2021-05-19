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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <AP_Common/Location.h>

// treat a uart[0+i] as "chan 100+i". This allows us to use chan 0-8ish for MAVLINK_CHAN tunnel
#define AP_CURSORONTARGET_UARTS_BASE_CHAN 100

#include <AP_Math/definitions.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <stdio.h>
#include <time.h>


#define AP_CURSORONTARGET_DEBUG 0
#if AP_CURSORONTARGET_DEBUG
    #define AP_COT_DEBUG(fmt, args...) { hal.console->printf(fmt, ##args); } while (0)
#else
    #define AP_COT_DEBUG(fmt, args...) 
#endif

#define AP_CURSORONTARGET_PARAM_DEFAULT_OPTIONS (AP_CursorOnTarget::Options::SEND_TO_ADSB)


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

    // @Param: OPTIONS
    // @DisplayName: Cursor on Target options
    // @Description: 0:Enable or disable using the pitch/roll stick control circle mode's radius and rate
    // @Bitmask: 0:Treat inbound vehicles as ADSB vehicles,2:blah,3:blah
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 2, AP_CursorOnTarget, _options, (int32_t)AP_CURSORONTARGET_PARAM_DEFAULT_OPTIONS),

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
    if (_initialized) {
        return;
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(_uart); i++) {
        _uart[i] = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CursorOnTarget, i);
        if (_uart[i] == nullptr) {
            break;
        }
        _num_uarts++;
    }
    _xml.init();

    _initialized = true;
}

void AP_CursorOnTarget::update()
{
    if (!_enabled) {
        return;
    }
    if (!_initialized) {
        init();
        return;
    }

    // -----------------------------
    // read any available data on serial port
    // -----------------------------
    for (uint8_t i = 0; i < AP_CURSORONTARGET_UARTS_MAX; i++) {
        if (_uart[i] == nullptr) {
            continue;
        }
        uint32_t nbytes = MIN(_uart[i]->available(), 1000UL);
        bool check_eof = (nbytes > 0);
        while (nbytes-- > 0) {
            const int16_t data = (uint8_t)_uart[i]->read();
            if (data < 0) {
                break;
            }
            if (data == 0 && yxml_eof(_xml.document) == YXML_OK) {
                // check for EOF inside the array in case there's two strings in this stream
                _xml.init();
            }

            const yxml_ret_t r = yxml_parse(_xml.document, data);
            handle_parsed_xml(_xml.document, r);
        } // while nbytes

        if (check_eof && (yxml_eof(_xml.document) == YXML_OK)) {
            _xml.init();
        }
    }


    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_run_ms < 4000) {
        // ADSB timeout is 5000 ms. This allow a slow refresh without it timing out
        return;
    }
    _last_run_ms = now_ms;

    if (now_ms < 5000) {
        // bootup delay
        return;
    }

    // static bool run_once = false;
    // if (run_once) {
    //     return;
    // }
    // run_once = true;


    const char* line[13] = {
        "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>\n",
        "<event version=\"2.0\"\n",
            "uid=\"K1000ULE\"\n",
            "type=\"a-f-A-M-F-Q\"\n",
            "time=\"2021-03-16T23:41:24.1855285Z\"\n",
            "start=\"2021-03-16T23:41:19.1855285Z\"\n",
            "stale=\"2021-03-16T23:41:29.1855285Z\"\n",
            "how=\"m-g\">\n",
            "<detail>\n",
                "<track course=\"1.23\" speed=\"4.56\" />\n",
            "</detail>\n",
            "<point lat=\"-35.3600000\" lon=\"149.1600000\" hae=\"584.84\" ce=\"1.0\" le=\"1.0\"/>\n",
          //"<point lat=\"-35.3632611\" lon=\"149.1652302\" hae=\"584.84\" ce=\"1.0\" le=\"1.0\"/>\n",
        "</event>\n"
    };

    for (uint8_t i=0; i<13; i++) {
        parse_string(0, line[i],strlen(line[i]));
    }

}

void AP_CursorOnTarget::parse_string(const uint8_t chan, const char* data, uint32_t len)
{
    if (!_enabled) {
        return;
    }
    while (len--) {
        if (!*data && yxml_eof(_xml.document) == YXML_OK) {
            // check for EOF inside the array in case there's two strings in this stream
            _xml.init();
        }
        
        const yxml_ret_t r = yxml_parse(_xml.document, *data++);
        handle_parsed_xml(_xml.document, r);
    }

    if (yxml_eof(_xml.document) == YXML_OK) {
            _xml.init();
    }
}

void AP_CursorOnTarget::handle_parsed_xml(yxml_t *x, yxml_ret_t r)
{
    //const uint16_t      element_current_count = _xml.elements_index;
    const CoTXML::State element_current = (_xml.elements_len > 0) ? _xml.elements[_xml.elements_len-1] : CoTXML::State::Unknown_Elem;
    CoTXML::State       element_new = CoTXML::State::Unknown_Elem;

	switch(r) {
	case YXML_PICONTENT:
	case YXML_CONTENT:
    case YXML_PISTART:
    case YXML_PIEND:
	case YXML_OK:
        // not handled
        return;

    default:
        // error condition. reset everything to generic unknown
        memset(_xml.elements, (uint8_t)CoTXML::State::Unknown, sizeof(_xml.elements));
        _xml.elements_len = 0;
        _xml.state_data = CoTXML::State::Unknown;
        AP_COT_DEBUG("\n* * * * * * * Unhandled YXML state %d * * * * * * *\n", (int)r);
        return;

 	case YXML_ELEMSTART:
        if (_xml.elements_len == 0) {
            AP_COT_DEBUG("\n*** XML DOCUMENT STARTING *** %s\n\n", x->elem);
            if (strcmp(x->elem, "event") == 0) {
                // this is the first thing we'll see at the start of a CoT msg.
                element_new = CoTXML::State::Event;
            }

        } else if (element_current == CoTXML::State::Event) {
            if (strcmp(x->elem, "detail") == 0) {
                element_new = CoTXML::State::Event_Detail;
            } else if (strcmp(x->elem, "point") == 0) {
                element_new = CoTXML::State::Event_Point;
            }

        } else if (element_current == CoTXML::State::Event_Detail) {
            if (strcmp(x->elem, "track") == 0) {
                element_new = CoTXML::State::Event_Detail_Track;
            }
        }

        if (element_new == CoTXML::State::Unknown_Elem) {
            AP_COT_DEBUG("******* ELEM Unhandled: ");
        }
        if (_xml.elements_len < AP_CURSORONTARGET_ELEMENT_DEPTH_MAX) {
            _xml.elements[_xml.elements_len] = element_new;
            _xml.elements_len++;
        } else {
            AP_COT_DEBUG("******* ELEM OVERFLOW: ");
        }

        AP_COT_DEBUG("(%2u,%2u) Elem START: %s\n", (unsigned)element_current, (unsigned)element_new, x->elem);
        break;

 	case YXML_ELEMEND:
        if (_xml.elements_len > 0) {
            _xml.elements_len--;

            if (_xml.elements_len > 0) {
                element_new = _xml.elements[_xml.elements_len-1];
                AP_COT_DEBUG("(%2u,%2u) Elem END\n", (unsigned)element_current, (unsigned)element_new);
            } else {
                AP_COT_DEBUG("\n*** XML DOCUMENT END ***\n");
                handle_end_of_document();
            }
        } else {
            AP_COT_DEBUG("******* ELEM UnderFLOW %u\n", (unsigned)element_current);
        }
        break;

    case YXML_ATTRSTART:
        _xml.data_buf_index = 0;
        _xml.state_data = CoTXML::State::Unknown_Attr; // this should get overritten if we successfully parse somethign we understand
        if (element_current == CoTXML::State::Event) {
            if (strcmp(x->attr, "version") == 0) {
                _xml.state_data = CoTXML::State::Event_Version;
            } else if (strcmp(x->attr, "uid") == 0) {
               _xml.state_data = CoTXML::State::Event_UID;
            } else if (strcmp(x->attr, "type") == 0) {
                _xml.state_data = CoTXML::State::Event_Type;
            } else if (strcmp(x->attr, "time") == 0) {
                _xml.state_data = CoTXML::State::Event_Time;
            } else if (strcmp(x->attr, "start") == 0) {
                _xml.state_data = CoTXML::State::Event_Start;
            } else if (strcmp(x->attr, "stale") == 0) {
                _xml.state_data = CoTXML::State::Event_Stale;
            } else if (strcmp(x->attr, "how") == 0) {
                _xml.state_data = CoTXML::State::Event_HOW;
            }

        } else if (element_current == CoTXML::State::Event_Detail_Track) {
            if (strcmp(x->attr, "course") == 0) {
                _xml.state_data = CoTXML::State::Event_Detail_Track_Course;
            } else if (strcmp(x->attr, "speed") == 0) {
                _xml.state_data = CoTXML::State::Event_Detail_Track_Speed;
            }

        } else if (element_current == CoTXML::State::Event_Point) {
            if (strcmp(x->attr, "lat") == 0) {
                _xml.state_data = CoTXML::State::Event_Point_Lat;
            } else if (strcmp(x->attr, "lon") == 0) {
                _xml.state_data = CoTXML::State::Event_Point_Lon;
            } else if (strcmp(x->attr, "hae") == 0) {
                _xml.state_data = CoTXML::State::Event_Point_HAE;
            } else if (strcmp(x->attr, "ce") == 0) {
                _xml.state_data = CoTXML::State::Event_Point_CE;
            } else if (strcmp(x->attr, "le") == 0) {
                _xml.state_data = CoTXML::State::Event_Point_LE;
            }
        }
        AP_COT_DEBUG("(%2u,%2u) AttrSTART: %s, ", (unsigned)element_current, (unsigned)_xml.state_data, x->attr);
        break;

    case YXML_ATTRVAL:
        AP_COT_DEBUG("%s", x->data);
        if (_xml.data_buf_index < sizeof(_xml.data_buf)-1) {
            // leave room for null termination
            _xml.data_buf[_xml.data_buf_index++] = *x->data;
        }
        break;

    case YXML_ATTREND:
        if (_xml.data_buf_index < sizeof(_xml.data_buf)) {
            // null terminate so we can do string processing
            _xml.data_buf[_xml.data_buf_index++] = 0;
        }
        handle_attribute(_xml.state_data, _xml.data_buf);
        AP_COT_DEBUG(", (%2u) AttrEND:%s=%s (len=%u)\n", (unsigned)_xml.state_data, x->attr, _xml.data_buf, _xml.data_buf_index);
        break;
    }
}

void AP_CursorOnTarget::handle_attribute(const CoTXML::State state, char* buf)
{
    switch (state) {
    case CoTXML::State::Event_Version:
        _xml.cache.version = strtof(buf, NULL);
        AP_COT_DEBUG("\n** CACHE.VERSION: %.1f\n", (double)_xml.cache.version);
        break;
    case CoTXML::State::Event_UID:
        strncpy(_xml.cache.uid, buf, sizeof(_xml.cache.uid)-1);
        AP_COT_DEBUG("\n** CACHE.UID: %s\n", _xml.cache.uid);
        break;

    case CoTXML::State::Event_Type:
        strncpy(_xml.cache.type, buf, sizeof(_xml.cache.type)-1);
        AP_COT_DEBUG("\n** CACHE.TYPE: %s\n", _xml.cache.type);
        break;

    case CoTXML::State::Event_Time:
    case CoTXML::State::Event_Start:
    case CoTXML::State::Event_Stale:
    case CoTXML::State::Event_HOW:
    case CoTXML::State::Event_Detail_Track_Course:
        _xml.cache.track_course = strtof(buf, NULL);
        AP_COT_DEBUG("\n** CACHE.TRACK_COURSE: %.2f\n", (double)_xml.cache.track_course);
        break;

    case CoTXML::State::Event_Detail_Track_Speed:            
        _xml.cache.track_speed = strtof(buf, NULL);
        AP_COT_DEBUG("\n** CACHE.TRACK_SPEED: %.2f\n", (double)_xml.cache.track_speed);
        break;
        
    case CoTXML::State::Event_Point_Lat:
        _xml.cache.loc.lat = strtof(buf, NULL) * 1e7f;
        AP_COT_DEBUG("\n** CACHE.LOC.LAT: %d\n", (int)_xml.cache.loc.lat);
        break;
    case CoTXML::State::Event_Point_Lon:
        _xml.cache.loc.lng = strtof(buf, NULL) * 1e7f;
        AP_COT_DEBUG("\n** CACHE.LOC.LNG: %d\n", (int)_xml.cache.loc.lng);
        break;
    case CoTXML::State::Event_Point_HAE:
        _xml.cache.loc.alt = strtof(buf, NULL) * 1e2f;
        AP_COT_DEBUG("\n** CACHE.LOC_ALT: %fm\n", (int)_xml.cache.loc.alt * 0.01);
        break;
    case CoTXML::State::Event_Point_CE:
    case CoTXML::State::Event_Point_LE:
        break;

    default:
        break;

    } // switch state
}

// detected end of document. Now lets act on all the cached values
void AP_CursorOnTarget::handle_end_of_document()
{
    update_adsb();
}


void AP_CursorOnTarget::update_adsb()
{
#if HAL_ADSB_ENABLED
    const char* type_uav = "a-f-A-M-F-Q";
    if (strncmp(_xml.cache.type, type_uav, strlen(type_uav)) != 0) {
        // for now we only accept this exact type
        return;
    }


    AP_ADSB *adsb = AP::ADSB();
    if (adsb == nullptr || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    AP_ADSB::adsb_vehicle_t vehicle {};

    vehicle.info.ICAO_address = 100;

    vehicle.info.tslc = 1;
    vehicle.info.lat =_xml.cache.loc.lat;
    vehicle.info.lon = _xml.cache.loc.lng;
    vehicle.info.altitude = _xml.cache.loc.alt * 10;
    vehicle.info.heading = wrap_360_cd(_xml.cache.track_course * 100.0f);
    vehicle.info.hor_velocity = _xml.cache.track_speed * 100.0f;
    vehicle.info.ver_velocity = 0;
    vehicle.info.emitter_type = ADSB_EMITTER_TYPE_UAV;
    vehicle.info.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
    vehicle.info.squawk = 1200;

    for (uint8_t i=0; i<9; i++) {
        vehicle.info.callsign[i] = _xml.cache.uid[i];
    }

    if (vehicle.info.altitude != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_ALTITUDE;
    }
    if (vehicle.info.lat || vehicle.info.lon) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (vehicle.info.heading != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (vehicle.info.hor_velocity != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (vehicle.info.callsign[0] != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (vehicle.info.squawk != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (vehicle.info.ver_velocity != 0) {
        vehicle.info.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }

    if (true) {
         vehicle.info.flags |= ADSB_FLAGS_SIMULATED;
    }
    if (true) {
        vehicle.info.flags |= ADSB_FLAGS_BARO_VALID;
    }

    vehicle.last_update_ms = AP_HAL::native_millis() - (vehicle.info.tslc * 1000);

    AP_COT_DEBUG("CoT sending info to ADSB");
    adsb->handle_adsb_vehicle(vehicle);
#endif
}

#endif // AP_CURSORONTARGET_ENABLED

namespace AP {
AP_CursorOnTarget *CursorOnTarget()
{
    return AP_CursorOnTarget::get_singleton();
}
};

