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

  //             "XX_TWELVE_XX"
  //AP_GROUPINFO("XXXXXXXXXXXX", XX, AP_KHA, _type, 0), // max name char length: KHA_ + 4

    AP_GROUPINFO("TYPE"        ,  1, AP_KHA, _type, (uint8_t)KHA_Vehicle_Type_t::MAIM),
    AP_GROUPINFO("ZERO_PIN"    ,  2, AP_KHA, _system.zeroize.pin, 20),
    AP_GROUPINFO("NO_EXT_GPIO" ,  3, AP_KHA, _ignore_uavcan_gpio_relay_commands, 0),
    AP_GROUPINFO("ROUTE_MAINT" ,  4, AP_KHA, _maint.route, (uint8_t)KHA_MAIM_Routing::NONE),
    AP_GROUPINFO("ROUTE_AVION" ,  5, AP_KHA, _avionics.route, (uint8_t)KHA_MAIM_Routing::NONE),
    AP_GROUPINFO("ROUTE_P1_CON",  6, AP_KHA, _payload[0].console.route, (uint8_t)KHA_MAIM_Routing::PAYLOAD1_CONSOLE_IP),
    AP_GROUPINFO("ROUTE_P2_CON",  7, AP_KHA, _payload[1].console.route, (uint8_t)KHA_MAIM_Routing::PAYLOAD2_CONSOLE_IP),
    AP_GROUPINFO("ROUTE_AHRS"  ,  8, AP_KHA, _ahrs.route, (uint8_t)KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL),

    AP_GROUPINFO("JSON_IP0"    , 11, AP_KHA, _ahrs.json.eth.addr.ip[0], 239),
    AP_GROUPINFO("JSON_IP1"    , 12, AP_KHA, _ahrs.json.eth.addr.ip[1], 2),
    AP_GROUPINFO("JSON_IP2"    , 13, AP_KHA, _ahrs.json.eth.addr.ip[2], 3),
    AP_GROUPINFO("JSON_IP3"    , 14, AP_KHA, _ahrs.json.eth.addr.ip[3], 1),
    AP_GROUPINFO("JSON_PORT"   , 15, AP_KHA, _ahrs.json.eth.addr.port, 6969),
    AP_GROUPINFO("JSON_EN"     , 16, AP_KHA, _ahrs.json.eth.enabled_at_boot, 0),

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 1
    AP_GROUPINFO("P1_CON_IP1"  , 20, AP_KHA, _payload[0].console.eth.addr.ip[0], 255),
    AP_GROUPINFO("P1_CON_IP1"  , 21, AP_KHA, _payload[0].console.eth.addr.ip[1], 255),
    AP_GROUPINFO("P1_CON_IP2"  , 22, AP_KHA, _payload[0].console.eth.addr.ip[2], 255),
    AP_GROUPINFO("P1_CON_IP3"  , 23, AP_KHA, _payload[0].console.eth.addr.ip[3], 255),
    AP_GROUPINFO("P1_CON_PORT" , 24, AP_KHA, _payload[0].console.eth.addr.port, 11222),
    AP_GROUPINFO("P1_CON_EN"   , 25, AP_KHA, _payload[0].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P1_POW_V_PIN", 26, AP_KHA, _payload[0].power.valid_pin, 1),
    AP_GROUPINFO("P1_POW_E_PIN", 27, AP_KHA, _payload[0].power.enable_pin, 3),
    AP_GROUPINFO("P1_POW_ENABL", 28, AP_KHA, _payload[0].power.enabled_at_boot, 0),
#endif

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 2
    AP_GROUPINFO("P2_CON_IP1"  , 30, AP_KHA, _payload[1].console.eth.addr.ip[0], 255),
    AP_GROUPINFO("P2_CON_IP1"  , 31, AP_KHA, _payload[1].console.eth.addr.ip[1], 255),
    AP_GROUPINFO("P2_CON_IP2"  , 32, AP_KHA, _payload[1].console.eth.addr.ip[2], 255),
    AP_GROUPINFO("P2_CON_IP3"  , 33, AP_KHA, _payload[1].console.eth.addr.ip[3], 255),
    AP_GROUPINFO("P2_CON_PORT" , 34, AP_KHA, _payload[1].console.eth.addr.port, 11223),
    AP_GROUPINFO("P2_CON_EN"   , 35, AP_KHA, _payload[1].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P2_POW_V_PIN", 36, AP_KHA, _payload[1].power.valid_pin, 2),
    AP_GROUPINFO("P2_POW_E_PIN", 37, AP_KHA, _payload[1].power.enable_pin, 4),
    AP_GROUPINFO("P2_POW_ENABL", 38, AP_KHA, _payload[1].power.enabled_at_boot, 0),
#endif // AP_KHA_MAIM_PAYLOAD_COUNT_MAX

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 3
    // 40-49
#endif

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 4
    // 50-59
#endif


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

    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        _payload[p].state.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Payload_State, p);
        _payload[p].console.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_KHA_MAIM_Payload_Console, p);
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



    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        hal.gpio->pinMode(_payload[p].power.valid_pin, HAL_GPIO_INPUT);
        hal.gpio->pinMode(_payload[p].power.enable_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_payload[p].power.enable_pin, _payload[p].power.enabled_at_boot);
        _payload[p].power.valid = hal.gpio->read(_payload[p].power.valid_pin);
    }


    hal.gpio->pinMode(_system.zeroize.pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_system.zeroize.pin, 0);


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


    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        _payload[p].power.valid = hal.gpio->read(_payload[p].power.valid_pin);
    }


    // clear memory buffers by setting all lengths to zero
    _avionics.uart.bytes_in.len = _avionics.uart.bytes_out.len = 0;
    _maint.uart.bytes_in.len = _maint.uart.bytes_out.len = 0;
    _ahrs.uart.bytes_in.len = _ahrs.uart.bytes_out.len = 0;
    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        _payload[p].state.uart.bytes_in.len = _payload[p].state.uart.bytes_out.len = 0;
        _payload[p].console.uart.bytes_in.len = _payload[p].console.uart.bytes_out.len = 0;
    }

    service_input_uarts();

    service_router();

    service_output_uarts();
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

    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        if (_payload[p].state.uart.port != nullptr) {
            _payload[p].state.uart.bytes_in.len = _payload[p].state.uart.port->read(_payload[p].state.uart.bytes_in.data, sizeof(_payload[p].state.uart.bytes_in.data));
        }

        if (_payload[p].console.uart.port != nullptr) {
            _payload[p].console.uart.bytes_in.len = _payload[p].console.uart.port->read(_payload[p].console.uart.bytes_in.data, sizeof(_payload[p].console.uart.bytes_in.data));
        }
    }
}


void AP_KHA::service_router()
{
    uint16_t len = 0;

    // if (_avionics.uart.port != nullptr) {
    //     _avionics.uart.bytes_in.len = _avionics.uart.port->read(_avionics.uart.bytes_in.data, sizeof(_avionics.uart.bytes_in.data));
    // }

    // if (_maint.uart.port != nullptr) {
    //     _maint.uart.bytes_in.len = _maint.uart.port->read(_maint.uart.bytes_in.data, sizeof(_maint.uart.bytes_in.data));
    // }
    
    // copy ahrs.in -> state[p].out
    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_payload[p].state.uart.bytes_out.data));
        memcpy(_payload[p].state.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _payload[p].state.uart.bytes_out.len += len;
    }

    // // copy state[p].in -> ahrs.out
    // for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
    //     // if more than one channel writes to "_ahrs.uart.bytes_out", then we need to append it so we don't clobber the data
    //     // This is likely ruining ther packets anyway so this is best done as locked access of one-on-one at a time
    //     const uint16_t offset = _ahrs.uart.bytes_out.len;
    //     const uint16_t len = MIN(_payload[p].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data) - offset);
    //     memcpy(&_ahrs.uart.bytes_out.data[offset], _payload[p].state.uart.bytes_in.data, len);
    //     _payload[p].state.uart.bytes_out.len += len;
    // }

    switch ((KHA_MAIM_Routing)_ahrs.route.get()) {
    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_SERIAL:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::AVIONICS:
        len = MIN(_avionics.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _avionics.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    default:
        break;
    }


    switch ((KHA_MAIM_Routing)_maint.route.get()) {
    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_SERIAL:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::AHRS_Bytes:
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::AVIONICS:
        len = MIN(_avionics.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _avionics.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    default:
        break;
    }

    switch ((KHA_MAIM_Routing)_avionics.route.get()) {
    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_SERIAL:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_SERIAL:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::AHRS_Bytes:
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    default:
        break;
    }



    switch ((KHA_MAIM_Routing)_payload[0].console.route.get()) {
    case KHA_MAIM_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_payload[0].console.uart.bytes_out.data));
        memcpy(_payload[0].console.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _payload[0].console.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD1_CONSOLE_IP:
        break;
    default:
        break;
    }

    switch ((KHA_MAIM_Routing)_payload[1].console.route.get()) {
    case KHA_MAIM_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_payload[1].console.uart.bytes_out.data));
        memcpy(_payload[1].console.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _payload[1].console.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD2_CONSOLE_IP:
        break;
    default:
        break;
    }

}


void AP_KHA::service_output_uarts()
{
    if (_avionics.uart.port != nullptr && _avionics.uart.bytes_out.len > 0) {
        _avionics.uart.port->write(_avionics.uart.bytes_out.data, _avionics.uart.bytes_out.len);
    }

    if (_maint.uart.port != nullptr && _maint.uart.bytes_out.len > 0) {
        _maint.uart.port->write(_maint.uart.bytes_out.data, _maint.uart.bytes_out.len);
    }

    if (_ahrs.uart.port != nullptr && _ahrs.uart.bytes_out.len > 0) {
        _ahrs.uart.port->write(_ahrs.uart.bytes_out.data, _ahrs.uart.bytes_out.len);
    }

    for (uint8_t p=0; p<AP_KHA_MAIM_PAYLOAD_COUNT_MAX; p++) {
        if (_payload[p].state.uart.port != nullptr && _payload[p].state.uart.bytes_out.len > 0) {
            _payload[p].state.uart.port->write(_payload[p].state.uart.bytes_out.data, _payload[p].state.uart.bytes_out.len);
        }
        if (_payload[p].console.uart.port != nullptr && _payload[p].console.uart.bytes_out.len > 0) {
            _payload[p].console.uart.port->write(_payload[p].console.uart.bytes_out.data, _payload[p].console.uart.bytes_out.len);
        }
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


void AP_KHA::set_gpio(const uint32_t index, const bool value)
{
    if (_ignore_uavcan_gpio_relay_commands) {
        return;
    }

    switch (index) {
    case 1:
        hal.gpio->write(_payload[0].power.enable_pin, value);
        break;
    case 2:
        hal.gpio->write(_payload[1].power.enable_pin, value);
        break;
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
