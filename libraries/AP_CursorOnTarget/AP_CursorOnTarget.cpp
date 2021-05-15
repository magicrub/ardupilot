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
        _num_uarts++;
    }
    yxml_init(_xml_document, _xml_stack, sizeof(_xml_stack));
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
    // if ((now_ms - _last_run_ms) < 100) {
    //     return;
    // }
    // _last_run_ms = now_ms;

    if ((now_ms - _last_run_ms) < 5000) {
        return;
    }

static bool run_once = false;

    if (!run_once) {
        run_once = true;
        hal.console->printf("\n***********\n");
        hal.console->printf("PARSING XML\n");
        hal.console->printf("***********\n");


// const char* test_str[0] = "<?xml version=\"1.0\" encoding=\"utf-8\" standalone=\"yes\"?>"
// const char* test_str[0] = <event
// const char* test_str[0] =   version=\"2.0\"
// const char* test_str[0] =   uid=\"K1000\"
// const char* test_str[0] =   type=\"a-f-A-M-F-Q\"
// const char* test_str[0] =   time=\"2021-05-15T21:02:43.6593362Z\"
// const char* test_str[0] =   start=\"2021-05-15T21:02:38.6593362Z\"
// const char* test_str[0] =   stale=\"2021-05-15T21:02:48.6593362Z\"
// const char* test_str[0] =   how=\"m-g\">
// const char* test_str[0] =   <detail>
// const char* test_str[0] =     <track
// const char* test_str[0] =       course=\"0.00\"
// const char* test_str[0] =       speed=\"0.00\" />
// const char* test_str[0] =   </detail>
// const char* test_str[0] =   <point
// const char* test_str[0] =     lat=\"0.0000000\"
// const char* test_str[0] =     lng=\"0.0000000\"
// const char* test_str[0] =     hae=\" 0.00\"
// const char* test_str[0] =     ce=\"1.0\"
// const char* test_str[0] =     le=\"1.0\" />
// const char* test_str[0] = </event>"";

//     const char* line[13] = {
// "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>",
// "<event version=\"2.0\"",
// "uid=\"K1000ULE_Direct\"",
// "type=\"a-f-A-M-F-Q\"",
// "time=\"2021-03-16T23:41:24.1855285Z\"",
// "start=\"2021-03-16T23:41:19.1855285Z\"",
// "stale=\"2021-03-16T23:41:29.1855285Z\"",
// "how=\"m-g\">",
// "<detail>",
// "<track course=\"0.00\" speed=\"0.00\" />",
// "</detail>",
// "<point lat=\"-35.3632611\" lon=\"149.1652302\" hae=\"584.84\" ce=\"1.0\" le=\"1.0\"",
// "</event>",
//         };

        // const char* line1 = "<?xml version='1.0' encoding='UTF-8' standalone='yes'?>\n";
        // const char* line2 = "<event version=\"2.0\"\n";
        // const char* line3 = "uid=\"K1000ULE_Direct\"\n";
        // const char* line4 = "type=\"a-f-A-M-F-Q\"\n";
        // const char* line5 = "time=\"2021-03-16T23:41:24.1855285Z\"\n";
        // const char* line6 = "start=\"2021-03-16T23:41:19.1855285Z\"\n";
        // const char* line7 = "stale=\"2021-03-16T23:41:29.1855285Z\"\n";
        // const char* line8 = "how=\"m-g\">\n";
        // const char* line9 = "<detail>\n";
        // const char* line10 = "<track course=\"0.00\" speed=\"0.00\" />\n";
        // const char* line11 = "</detail>\n";
        // const char* line12 = "<point lat=\"-35.3632611\" lon=\"149.1652302\" hae=\"584.84\" ce=\"1.0\" le=\"1.0\"\n";
        // const char* line13 = "</event>\n";


const char* line1 = "<a>ZOMFG! Element	content!";
const char* line2 = "";
const char* line3 = "<entities>&amp;&lt;&gt;&apos;&quot;</entities>";
const char* line4 = "<refs>&#x20;&#33;&#x0020;&#0033;&#xe9;&#x2603;&#x1F431;</refs>";
const char* line5 = "<![CDATA[CDATA!]]>";
const char* line6 = "<![CDATA[[[CD<a/> <!-- no comment -->&amp;<?NotaPI?>&notaref;";
const char* line7 = "]x]]y]]]z]]]]>";
const char* line8 = "</a>";

        parse_bytes(0, line1,strlen(line1));
        parse_bytes(0, line2,strlen(line2));
        parse_bytes(0, line3,strlen(line3));
        parse_bytes(0, line4,strlen(line4));
        parse_bytes(0, line5,strlen(line5));
        parse_bytes(0, line6,strlen(line6));
        parse_bytes(0, line7,strlen(line7));
        parse_bytes(0, line8,strlen(line8));
        // parse_bytes(0, line9,strlen(line9));
        // parse_bytes(0, line10,strlen(line10));
        // parse_bytes(0, line11,strlen(line11));
        // parse_bytes(0, line12,strlen(line12));
        // parse_bytes(0, line13,strlen(line13));


        // const char* line1 = "<:A_S0mewhat-Longer.Name></:A_S0mewhat-Longer.Name>";
        // parse_bytes(0, line1,strlen(line1));

        y_printtoken(_xml_document, yxml_eof(_xml_document) < 0 ? "error\n" : "ok\n");

    }

}

void AP_CursorOnTarget::parse_bytes(const uint8_t chan, const char* data, uint32_t len)
{
    if (!_enabled) {
        return;
    }
    while (len--) {
        parse_byte(chan, *data++);
    }
}

void AP_CursorOnTarget::parse_byte(const uint8_t chan, const char data)
{
    if (!_enabled) {
        return;
    }

    const yxml_ret_t r = yxml_parse(_xml_document, data);
    handle_parsed_xml(_xml_document, r);

    //y_printres(x, r);

	//y_printtoken(x, yxml_eof(x) < 0 ? "error\n" : "ok\n");
}

void AP_CursorOnTarget::handle_parsed_xml(yxml_t *x, yxml_ret_t r)
{
	int32_t nextdata = 0;

	switch(r) {
	case YXML_OK:
        nextdata = _xml_indata;
		break;
	case YXML_ELEMSTART:
		y_printtoken(x, "elemstart ");
		y_printstring(x->elem);
		if(yxml_symlen(x, x->elem) != strlen(x->elem))
			y_printtoken(x, "assertfail: elem lengths don't match");
		if(r & YXML_CONTENT)
			y_printtoken(x, "content");
		break;
	case YXML_ELEMEND:
		y_printtoken(x, "elemend");
		break;
	case YXML_ATTRSTART:
		y_printtoken(x, "attrstart ");
		y_printstring(x->attr);
		if(yxml_symlen(x, x->attr) != strlen(x->attr))
			y_printtoken(x, "assertfail: attr lengths don't match");
		break;
	case YXML_ATTREND:
		y_printtoken(x, "attrend");
		break;
	case YXML_PICONTENT:
	case YXML_CONTENT:
	case YXML_ATTRVAL:
		if(!_xml_indata)
			y_printtoken(x, r == YXML_CONTENT ? "content " : r == YXML_PICONTENT ? "picontent " : "attrval ");
		y_printstring(x->data);
		nextdata = 1;
		break;
	case YXML_PISTART:
		y_printtoken(x, "pistart ");
		y_printstring(x->pi);
		if(yxml_symlen(x, x->pi) != strlen(x->pi))
			y_printtoken(x, "assertfail: pi lengths don't match");
		break;
	case YXML_PIEND:
		y_printtoken(x, "piend");
		break;
	default:
		y_printtoken(x, "error\n");
		exit(0);
	}
	_xml_indata = nextdata;
}

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

