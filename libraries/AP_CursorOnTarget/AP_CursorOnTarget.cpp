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


#define AP_COT_XML_BITMASK_VERSION          (1 << 0)
#define AP_COT_XML_BITMASK_UID              (1 << 1)
#define AP_COT_XML_BITMASK_TYPE             (1 << 2)
#define AP_COT_XML_BITMASK_TIME             (1 << 3)
#define AP_COT_XML_BITMASK_START            (1 << 4)
#define AP_COT_XML_BITMASK_STALE            (1 << 5)
#define AP_COT_XML_BITMASK_HOW              (1 << 6)
#define AP_COT_XML_BITMASK_TRACK__          (1 << 7)
#define AP_COT_XML_BITMASK_POINT__          (1 << 8)



#define AP_COT_XML_BITMASK_TRACK_COURSE     (1 << 0)

#define AP_COT_XML_BITMASK_POINT_LAT        (1 << 0)
#define AP_COT_XML_BITMASK_POINT_LON        (1 << 1)
#define AP_COT_XML_BITMASK_POINT_HAE        (1 << 2)
#define AP_COT_XML_BITMASK_POINT_CE         (1 << 3)
#define AP_COT_XML_BITMASK_POINT_LE         (1 << 4)


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
    yxml_init(_xml_document, _xml_stack, sizeof(_xml_stack));
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

    const uint32_t now_ms = AP_HAL::millis();

    // only send at 10Hz
    if (_last_run_ms && (now_ms - _last_run_ms) < 5000) {
        return;
    }
    _last_run_ms = now_ms;


    static bool run_once = false;

    if (run_once) {
        return;
    }
   run_once = true;


    hal.console->printf("\n***********\n");
    hal.console->printf("PARSING XML - START\n");
    hal.console->printf("***********\n");

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
                "<track course=\"0.00\" speed=\"0.00\" />\n",
            "</detail>\n",
            "<point lat=\"-35.3632611\" lon=\"149.1652302\" hae=\"584.84\" ce=\"1.0\" le=\"1.0\"/>\n",
        "</event>\n"
    };

    for (uint8_t i=0; i<13; i++) {
        parse_string(0, line[i],strlen(line[i]));
    }


//    y_printtoken(_xml_document,  < 0 ? "error\n" : "ok\n");

    hal.console->printf("\n***********\n");
    hal.console->printf("PARSING XML -- DONE\n");
    hal.console->printf("***********\n");

}

void AP_CursorOnTarget::parse_string(const uint8_t chan, const char* data, uint32_t len)
{
    if (!_enabled) {
        return;
    }
    while (len--) {
        if (!*data && yxml_eof(_xml_document) == YXML_OK) {
            // check for EOF inside the array in case there's two strings in this stream
            yxml_init(_xml_document, _xml_stack, sizeof(_xml_stack));
        }
        
        const yxml_ret_t r = yxml_parse(_xml_document, *data++);
        handle_parsed_xml(_xml_document, r);
    }

    if (yxml_eof(_xml_document) == YXML_OK) {
        yxml_init(_xml_document, _xml_stack, sizeof(_xml_stack));
    }
}

void AP_CursorOnTarget::handle_parsed_xml(yxml_t *x, yxml_ret_t r)
{
	switch(r) {
	case YXML_OK:
    default:
        return;
// 	case YXML_ELEMSTART:

    case YXML_ATTRSTART:
        if (strcmp(x->attr, "event") == 0) {
            _xml_state = XML_state::Event;
        } else (_xml_state ==  XML_state::Event) {
            if (strcmp(x->attr, "uid") == 0) {
                _xml_state = XML_state::Event_UID;
            } else if (strcmp(x->attr, "type") == 0) {
                _xml_state = XML_state::Event_Type;
            } else if (strcmp(x->attr, "time") == 0) {
                _xml_state = XML_state::Event_Time;
            } else if (strcmp(x->attr, "start") == 0) {
                _xml_state = XML_state::Event_Start;
            } else if (strcmp(x->attr, "stale") == 0) {
                _xml_state = XML_state::Event_Stale;
            } else if (strcmp(x->attr, "how") == 0) {
                _xml_state = XML_state::Event_HOW;
            } else if (strcmp(x->attr, "track") == 0) {
                _xml_state = XML_state::Event_Track;
        } else if (strcmp(x->attr, "point") == 0) {
            _xml_state = XML_state::Point;
        } else {
            _xml_state = XML_state::Unknown;
            break;
        }
        if (_xml_state != XML_state::Unknown) {
            hal.console->printf ("%u Found %s: ", (unsigned)_xml_state, x->attr);
        }
        break;
        Track,
        Track_Course,
        Point,
        Point_Lat,
        Point_Lon,
        Point_HAE,
        Point_CE,
        Point_LE

    case YXML_ATTRVAL:
        if (_xml_state == XML_state::Unknown) {
            break;
        }
        hal.console->printf ("%s", x->data);
        break;

    case YXML_ATTREND:
        if (_xml_state == XML_state::Unknown) {
            break;
        }
        hal.console->printf ("\n");
        _xml_state = XML_state::Unknown;
        break;

        // if (_xml_state == XML_state::UID) {
        //     _xml_parse_state.expect_event = 0;
        //     hal.console->printf ("Found %s:  %s\n", x->attr, x->data);
        // } else if (_xml_parse_state.expect_event == AP_COT_XML_BITMASK_TYPE && strcmp(x->attr, "type") == 0) {
        //     _xml_parse_state.expect_event = 0;
        //     hal.console->printf ("Found Type:  %s\n", x->data);
        // }
        break;
    }

}

// void AP_CursorOnTarget::handle_parsed_xml(yxml_t *x, yxml_ret_t r)
// {
//  static int32_t indata;
// 	int32_t nextdata = 0;

// 	switch(r) {
// 	case YXML_OK:
//         nextdata = indata;
// 		break;
// 	case YXML_ELEMSTART:
// 		y_printtoken(x, "elemstart ");
// 		y_printstring(x->elem);
// 		if(yxml_symlen(x, x->elem) != strlen(x->elem))
// 			y_printtoken(x, "assertfail: elem lengths don't match");
// 		if(r & YXML_CONTENT)
// 			y_printtoken(x, "content");
// 		break;
// 	case YXML_ELEMEND:
// 		y_printtoken(x, "elemend");
// 		break;
// 	case YXML_ATTRSTART:
// 		y_printtoken(x, "attrstart ");
// 		y_printstring(x->attr);
// 		if(yxml_symlen(x, x->attr) != strlen(x->attr))
// 			y_printtoken(x, "assertfail: attr lengths don't match");
// 		break;
// 	case YXML_ATTREND:
// 		y_printtoken(x, "attrend");
// 		break;
// 	case YXML_PICONTENT:
// 	case YXML_CONTENT:
// 	case YXML_ATTRVAL:
// 		if(!indata)
// 			y_printtoken(x, r == YXML_CONTENT ? "content " : r == YXML_PICONTENT ? "picontent " : "attrval ");
// 		y_printstring(x->data);
// 		nextdata = 1;
// 		break;
// 	case YXML_PISTART:
// 		y_printtoken(x, "pistart ");
// 		y_printstring(x->pi);
// 		if(yxml_symlen(x, x->pi) != strlen(x->pi))
// 			y_printtoken(x, "assertfail: pi lengths don't match");
// 		break;
// 	case YXML_PIEND:
// 		y_printtoken(x, "piend");
// 		break;
// 	default:
// 		y_printtoken(x, "error\n");
// 		//exit(0);
// 	}
// 	indata = nextdata;
// }

void AP_CursorOnTarget::y_printchar(char c)
{
	if(c == '\x7F' || (c >= 0 && c < 0x20))
		hal.console->printf("\\x%02x", c);
	else
		hal.console->printf("%c", c);
}
void AP_CursorOnTarget::y_printstring(const char *str)
{
	while(*str) {
		y_printchar(*str);
		str++;
	}
}
void AP_CursorOnTarget::y_printtoken(yxml_t *x, const char *str)
{
	// puts("");
	// if(verbose)
	// 	printf("t%03"PRIu64" l%03"PRIu32" b%03"PRIu64": ", x->total, x->line, x->byte);
	hal.console->printf("%s\n", str);
}


#endif // AP_CURSORONTARGET_ENABLED

namespace AP {
AP_CursorOnTarget *CursorOnTarget()
{
    return AP_CursorOnTarget::get_singleton();
}
};

