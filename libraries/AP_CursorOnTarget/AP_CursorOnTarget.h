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

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

#ifndef AP_CURSORONTARGET_ENABLED
#define AP_CURSORONTARGET_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef AP_CURSORONTARGET_UARTS_MAX
#define AP_CURSORONTARGET_UARTS_MAX 1
#endif

#define AP_CURSORONTARGET_XML_STACK_SIZE (4*1024)

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

    void parse_bytes(const uint8_t chan, const char* data, uint32_t len);
    void parse_byte(const uint8_t chan, const char data);

    static const struct AP_Param::GroupInfo        var_info[];

private:
    static AP_CursorOnTarget* _singleton;
    
    // lazy init
    void init();

    // xml parsing
    void        y_printchar(char c);
    void        y_printstring(const char *str);
    void        y_printtoken(yxml_t *x, const char *str);
    void        handle_parsed_xml(yxml_t *x, yxml_ret_t r);
    char        _xml_stack[AP_CURSORONTARGET_XML_STACK_SIZE];
	yxml_t      _xml_document[1];
	int32_t     _xml_indata;


    bool        _initialized;
    uint8_t     _num_uarts;
    AP_HAL::UARTDriver* _uart[AP_CURSORONTARGET_UARTS_MAX];
    uint32_t    _last_run_ms;

    // parameters
    AP_Int8     _enabled;
    AP_Float    _send_rate;

};

namespace AP {
    AP_CursorOnTarget *CursorOnTarget();
};

