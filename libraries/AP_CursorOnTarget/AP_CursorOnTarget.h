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


#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_CURSORONTARGET_ENABLED
#define AP_CURSORONTARGET_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_ADSB/AP_ADSB.h>



#ifndef AP_CURSORONTARGET_UARTS_MAX
#define AP_CURSORONTARGET_UARTS_MAX 1
#endif

#define AP_CURSORONTARGET_XML_STACK_SIZE (1024)
#define AP_CURSORONTARGET_ELEMENT_DEPTH_MAX (10)

#if AP_CURSORONTARGET_ENABLED
#include "yxml.h"
#endif

class AP_CursorOnTarget {

public:
    AP_CursorOnTarget();

    /* Do not allow copies */
    AP_CursorOnTarget(const AP_CursorOnTarget &other) = delete;
    AP_CursorOnTarget &operator=(const AP_CursorOnTarget&) = delete;

    static AP_CursorOnTarget *get_singleton() { return _singleton; }

    // update - should be called at least 10Hz
    void update();

    // indicate whether this module is enabled or not
    bool enabled() const { return _enabled; }

    void parse_string(const uint8_t chan, const char* data, uint32_t len);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_CursorOnTarget* _singleton;
    
        enum class Options : int32_t {
        SEND_TO_ADSB = (1<<0),
    };

    // lazy init
    void init();

    // xml parsing
    void        y_printchar(char c);
    void        y_printstring(const char *str);
    void        y_printtoken(yxml_t *x, const char *str);
    void        handle_parsed_xml(yxml_t *x, yxml_ret_t r);
    void        update_adsb();

    struct CoTXML {
        public:
        
        struct Cache {
            float version;
            float track_course;
            float track_speed;
            char uid[30];
            char type[20];
            struct Location loc;

            void reset() {
                version = 0;
                track_course = 0;
                track_speed = 0;
                memset(&uid, 0, sizeof(uid));
                memset(&type, 0, sizeof(type));
                loc = Location();
            }

        } cache;

        void init() {
            yxml_init(document, stack, sizeof(stack));
            cache.reset();
        }
	    yxml_t document[1];

        enum class State {
            Unknown = 0,
            Unknown_Elem,
            Unknown_Attr,
            Event = 3,
            Event_Version = 4,
            Event_UID,
            Event_Type,
            Event_Time,
            Event_Start,
            Event_Stale,
            Event_HOW,
            Event_Detail = 11,
            Event_Detail_Track = 12,
            Event_Detail_Track_Course,
            Event_Detail_Track_Speed,
            Event_Point = 15,
            Event_Point_Lat,
            Event_Point_Lon,
            Event_Point_HAE,
            Event_Point_CE,
            Event_Point_LE
        };

        State state_data;
        State elements[AP_CURSORONTARGET_ELEMENT_DEPTH_MAX];
        uint16_t elements_len;

        char data_buf[100];
        uint16_t data_buf_index;
        

    private:
        char stack[AP_CURSORONTARGET_XML_STACK_SIZE];
    } _xml;


    void        handle_attribute(const CoTXML::State state, char* buf);
    void        handle_end_of_document();

    bool        _initialized;
    uint8_t     _num_uarts;
    AP_HAL::UARTDriver* _uart[AP_CURSORONTARGET_UARTS_MAX];
    uint32_t    _last_run_ms;

    // parameters
    AP_Int8     _enabled;
    AP_Float    _send_rate;
    AP_Int32    _options;

};

namespace AP {
    AP_CursorOnTarget *CursorOnTarget();
};

