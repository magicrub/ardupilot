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
/*
  temperature calibration library
 */

#include "AP_KHA.h"
//#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define KHA_DEBUG 0

#if KHA_DEBUG
# define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\r\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define debug(fmt, args ...)
#endif

// table of user settable and learned parameters
const AP_Param::GroupInfo AP_KHA::var_info[] = {

  //AP_GROUPINFO("XXXXXXXXXXXX", XX, AP_KHA, _type, 0), // max name char length: KHA_ + 4

    AP_GROUPINFO("TYPE"        ,  1, AP_KHA, _type, (int8_t)KHA_Vehicle_Type_t::MAIM),

    AP_GROUPINFO("JSON_IP0"    , 11, AP_KHA, _ahrs.json.eth.addr.ip[0], 239),
    AP_GROUPINFO("JSON_IP1"    , 12, AP_KHA, _ahrs.json.eth.addr.ip[1], 2),
    AP_GROUPINFO("JSON_IP2"    , 13, AP_KHA, _ahrs.json.eth.addr.ip[2], 3),
    AP_GROUPINFO("JSON_IP3"    , 14, AP_KHA, _ahrs.json.eth.addr.ip[3], 1),
    AP_GROUPINFO("JSON_PORT"   , 15, AP_KHA, _ahrs.json.eth.addr.port, 6969),
    AP_GROUPINFO("JSON_EN"     , 16, AP_KHA, _ahrs.json.eth.enabled_at_boot, 1),



     AP_GROUPINFO("MAINT_PROTO" , 17, AP_KHA, _maint.protocol, (int8_t)KHA_MAIM_Routing::NONE),

     AP_GROUPINFO("AVION_PROTO" , 18, AP_KHA, _avionics.protocol, (int8_t)KHA_MAIM_Routing::NONE),


    AP_GROUPINFO("P1_CON_IP1"   , 19, AP_KHA, _payload[0].console.eth.addr.ip[0], 255),
    AP_GROUPINFO("P1_CON_IP1"   , 20, AP_KHA, _payload[0].console.eth.addr.ip[1], 255),
    AP_GROUPINFO("P1_CON_IP2"   , 21, AP_KHA, _payload[0].console.eth.addr.ip[2], 255),
    AP_GROUPINFO("P1_CON_IP3"   , 22, AP_KHA, _payload[0].console.eth.addr.ip[3], 255),
    AP_GROUPINFO("P1_CON_PORT"  , 23, AP_KHA, _payload[0].console.eth.addr.port, 11222),
    AP_GROUPINFO("P1_CON_EN"    , 24, AP_KHA, _payload[0].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P1_POW_EN"    , 25, AP_KHA, _payload[0].power.enabled_at_boot, 1),

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 2
    AP_GROUPINFO("P2_CON_IP1"   , 26, AP_KHA, _payload[1].console.eth.addr.ip[0], 255),
    AP_GROUPINFO("P2_CON_IP1"   , 27, AP_KHA, _payload[1].console.eth.addr.ip[1], 255),
    AP_GROUPINFO("P2_CON_IP2"   , 28, AP_KHA, _payload[1].console.eth.addr.ip[2], 255),
    AP_GROUPINFO("P2_CON_IP3"   , 29, AP_KHA, _payload[1].console.eth.addr.ip[3], 255),
    AP_GROUPINFO("P2_CON_PORT"  , 30, AP_KHA, _payload[1].console.eth.addr.port, 11222),
    AP_GROUPINFO("P2_CON_EN"    , 31, AP_KHA, _payload[1].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P2_POW_EN"    , 32, AP_KHA, _payload[1].power.enabled_at_boot, 1),
#endif // AP_KHA_MAIM_PAYLOAD_COUNT_MAX

    AP_GROUPEND
};

AP_KHA::AP_KHA(void)
{
    _singleton = this;

    // load parameter defaults
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_KHA::init(void)
{
    if (_init.done) {
        return;
    }

    _ahrs.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_AHRS_Sensor, 0);

    for (uint8_t i=0; i<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; i++) {
        _payload[i].state.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Payload_State, i);
        _payload[i].console.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Payload_Console, i);
    }

    _maint.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Maint, 0);
    _avionics.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Avionics, 0);


    if (_system.pps.in.pin >= 0) {
        hal.gpio->pinMode(_system.pps.in.pin, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(
                _system.pps.in.pin,
                FUNCTOR_BIND_MEMBER(&AP_KHA::pps_pin_irq_handler,
                                    void,
                                    uint8_t,
                                    bool,
                                    uint32_t),
                AP_HAL::GPIO::INTERRUPT_BOTH)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "KHA: Failed to attach to pin %u", _system.pps.in.pin);
        }
        _system.pps.in.state = hal.gpio->read(_system.pps.in.pin);
    }


    _avionics.uart.bytes_in.len = 0;
    _maint.uart.bytes_in.len = 0;
    _ahrs.uart.bytes_in.len = 0;

    _init.done = true;
}

void AP_KHA::update(void)
{
    if (!_init.done) {
        init();
        return;
    }

    // switch (_type.get()) {
    //     default:
    //     break;
    // }

    service_input_uarts();

    service_output_uart(_maint.protocol, _maint.uart.port);
    service_output_uart(_avionics.protocol, _avionics.uart.port);
    
    for (uint8_t i=0; i<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; i++) {
        service_output_uart(_maint.protocol, _payload[i].state.uart.port);
        service_output_uart(_maint.protocol, _payload[i].state.uart.port);
    }
}

void AP_KHA::service_input_uarts()
{
    if (_avionics.uart.port != nullptr) {
        _avionics.uart.bytes_in.len = _avionics.uart.port->read(_avionics.uart.bytes_in.data, sizeof(_avionics.uart.bytes_in.data));
    }

    if (_maint.uart.port != nullptr) {
        _maint.uart.bytes_in.len = _maint.uart.port->read(_maint.uart.bytes_in.data, sizeof(_maint.uart.bytes_in.data));
    }

    if (_ahrs.uart.port != nullptr) {
        _ahrs.uart.bytes_in.len = _ahrs.uart.port->read(_ahrs.uart.bytes_in.data, sizeof(_ahrs.uart.bytes_in.data));
    }

    for (uint8_t i=0; i<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; i++) {
        if (_payload[i].state.uart.port != nullptr) {
            _payload[i].state.uart.bytes_in.len = _payload[i].state.uart.port->read(_payload[i].state.uart.bytes_in.data, sizeof(_payload[i].state.uart.bytes_in.data));
        }
        if (_payload[i].console.uart.port != nullptr) {
            _payload[i].console.uart.bytes_in.len = _payload[i].console.uart.port->read(_payload[i].console.uart.bytes_in.data, sizeof(_payload[i].console.uart.bytes_in.data));
        }
    }
}

void AP_KHA::service_output_uart(const KHA_MAIM_Routing protocol, AP_HAL::UARTDriver *&port)
{
    if (port == nullptr) {
        return;
    }

    uint8_t index;

    switch (protocol) {
    case KHA_MAIM_Routing::PAYLOAD1_STATE:
    case KHA_MAIM_Routing::PAYLOAD2_STATE:
    case KHA_MAIM_Routing::PAYLOAD3_STATE:
    case KHA_MAIM_Routing::PAYLOAD4_STATE:
        index = (uint8_t)protocol - (uint8_t)KHA_MAIM_Routing::PAYLOAD1_STATE;
        if (_payload[index].state.uart.bytes_in.len > 0) {
            port->write(_payload[index].state.uart.bytes_in.data, _payload[index].state.uart.bytes_in.len);
        }
        break;

    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL:
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_SERIAL:
    case KHA_MAIM_Routing::PAYLOAD3_CONSOLE_SERIAL:
    case KHA_MAIM_Routing::PAYLOAD4_CONSOLE_SERIAL:
        index = (uint8_t)protocol - (uint8_t)KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL;
        if (_payload[index].console.uart.bytes_in.len > 0) {
            port->write(_payload[index].console.uart.bytes_in.data, _payload[index].console.uart.bytes_in.len);
        }
        break;

    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_IP:
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_IP:
    case KHA_MAIM_Routing::PAYLOAD3_CONSOLE_IP:
    case KHA_MAIM_Routing::PAYLOAD4_CONSOLE_IP:
        break;

    case KHA_MAIM_Routing::MAINT:
        if (_maint.uart.bytes_in.len > 0) {
            port->write(_maint.uart.bytes_in.data, _maint.uart.bytes_in.len);
        }
        break;

    case KHA_MAIM_Routing::AVIONICS:
        if (_avionics.uart.bytes_in.len > 0) {
            port->write(_avionics.uart.bytes_in.data, _avionics.uart.bytes_in.len);
        }
        break;

    case KHA_MAIM_Routing::AHRS_Bytes:
        if (_ahrs.uart.bytes_in.len > 0) {
            port->write(_ahrs.uart.bytes_in.data, _ahrs.uart.bytes_in.len);
        }
        break;

    case KHA_MAIM_Routing::LOOPBACK:
    case KHA_MAIM_Routing::AHRS_JSON:
    case KHA_MAIM_Routing::AHRS_Packets:
    case KHA_MAIM_Routing::SLCAN:
    case KHA_MAIM_Routing::MAVLink:
    case KHA_MAIM_Routing::NONE:
        // not yet supported
        break;
    }
}

void AP_KHA::pps_pin_irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us)
{
    _system.pps.in.state = pin_value;
    if (pin_value) {
        _system.pps.in.timestamp_high_us = timestamp_us;
    } else {
        _system.pps.in.timestamp_low_us = timestamp_us;
    }
}

// singleton instance
AP_KHA *AP_KHA::_singleton;

namespace AP
{

AP_KHA *kha()
{
    return AP_KHA::get_singleton();
}

}
