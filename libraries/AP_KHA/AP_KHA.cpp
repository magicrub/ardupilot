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
#include <AP_Networking/AP_Networking.h>

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

    AP_GROUPINFO("JSON_IP1"    , 11, AP_KHA, _ahrs.json.eth.addr.ip[0], 239),
    AP_GROUPINFO("JSON_IP2"    , 12, AP_KHA, _ahrs.json.eth.addr.ip[1], 2),
    AP_GROUPINFO("JSON_IP3"    , 13, AP_KHA, _ahrs.json.eth.addr.ip[2], 3),
    AP_GROUPINFO("JSON_IP4"    , 14, AP_KHA, _ahrs.json.eth.addr.ip[3], 1),
    AP_GROUPINFO("JSON_PORT"   , 15, AP_KHA, _ahrs.json.eth.addr.port, 6969),
    AP_GROUPINFO("JSON_EN"     , 16, AP_KHA, _ahrs.json.eth.enabled_at_boot, 0),
    AP_GROUPINFO("JSON_RATE"   , 17, AP_KHA, _ahrs.json.eth.interval_ms, 10),

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 1
    AP_GROUPINFO("P1_CON_IP1"  , 20, AP_KHA, _payload[0].console.eth.addr.ip[0], 239),
    AP_GROUPINFO("P1_CON_IP2"  , 21, AP_KHA, _payload[0].console.eth.addr.ip[1], 2),
    AP_GROUPINFO("P1_CON_IP3"  , 22, AP_KHA, _payload[0].console.eth.addr.ip[2], 3),
    AP_GROUPINFO("P1_CON_IP4"  , 23, AP_KHA, _payload[0].console.eth.addr.ip[3], 2),
    AP_GROUPINFO("P1_CON_PORT" , 24, AP_KHA, _payload[0].console.eth.addr.port, 7000),
    AP_GROUPINFO("P1_CON_EN"   , 25, AP_KHA, _payload[0].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P1_POW_V_PIN", 26, AP_KHA, _payload[0].power.valid_pin, 1),
    AP_GROUPINFO("P1_POW_E_PIN", 27, AP_KHA, _payload[0].power.enable_pin, 3),
    AP_GROUPINFO("P1_POW_ENABL", 28, AP_KHA, _payload[0].power.enabled_at_boot, 0),
    AP_GROUPINFO("P1_CON_RATE" , 29, AP_KHA, _payload[0].console.eth.interval_ms, 10),
#endif

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 2
    AP_GROUPINFO("P2_CON_IP1"  , 30, AP_KHA, _payload[1].console.eth.addr.ip[0], 239),
    AP_GROUPINFO("P2_CON_IP2"  , 31, AP_KHA, _payload[1].console.eth.addr.ip[1], 2),
    AP_GROUPINFO("P2_CON_IP3"  , 32, AP_KHA, _payload[1].console.eth.addr.ip[2], 3),
    AP_GROUPINFO("P2_CON_IP4"  , 33, AP_KHA, _payload[1].console.eth.addr.ip[3], 3),
    AP_GROUPINFO("P2_CON_PORT" , 34, AP_KHA, _payload[1].console.eth.addr.port, 7001),
    AP_GROUPINFO("P2_CON_EN"   , 35, AP_KHA, _payload[1].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P2_POW_V_PIN", 36, AP_KHA, _payload[1].power.valid_pin, 2),
    AP_GROUPINFO("P2_POW_E_PIN", 37, AP_KHA, _payload[1].power.enable_pin, 4),
    AP_GROUPINFO("P2_POW_ENABL", 38, AP_KHA, _payload[1].power.enabled_at_boot, 0),
    AP_GROUPINFO("P2_CON_RATE" , 39, AP_KHA, _payload[1].console.eth.interval_ms, 10),
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
        
        bool enabled;
        if (_payload[p].power.enabled_at_boot == 2) {
            // keep current enabled state
            hal.gpio->pinMode(_payload[p].power.enable_pin, HAL_GPIO_INPUT);
            enabled = hal.gpio->read(_payload[p].power.enable_pin);
        } else {
            enabled = _payload[p].power.enabled_at_boot;
        }
        hal.gpio->pinMode(_payload[p].power.enable_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_payload[p].power.enable_pin, enabled);

        _payload[p].console.eth.enabled = _payload[p].console.eth.enabled_at_boot;
    }

    hal.gpio->pinMode(_system.zeroize.pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_system.zeroize.pin, 0);

    _ahrs.json.eth.enabled = _ahrs.json.eth.enabled_at_boot;



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
    case KHA_MAIM_Routing::PAYLOAD1_STATE:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case KHA_MAIM_Routing::PAYLOAD2_STATE:
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

char* AP_KHA::get_udp_out_ip(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return convert_ip_to_str(stream_id, _ahrs.json.eth.addr);
        case 1: return convert_ip_to_str(stream_id, _payload[0].console.eth.addr);
        case 2: return convert_ip_to_str(stream_id, _payload[1].console.eth.addr);
    }
    return nullptr;
}

uint16_t AP_KHA::get_udp_out_port(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return _ahrs.json.eth.addr.port;
        case 1: return _payload[0].console.eth.addr.port;
        case 2: return _payload[1].console.eth.addr.port;
    }
    return 0;
}

char* AP_KHA::get_udp_out_name(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return (char*)"AHRS JSON";
        case 1: return (char*)"Payload 1 Console";
        case 2: return (char*)"Payload 2 Console";
    }
    return nullptr;
}

uint32_t AP_KHA::get_udp_out_interval_ms(const uint32_t stream_id)
{
    int32_t interval_ms;
    switch (stream_id) {
        case 0: interval_ms =_ahrs.json.eth.interval_ms; break;
        case 1: interval_ms =_payload[0].console.eth.interval_ms; break;
        case 2: interval_ms =_payload[1].console.eth.interval_ms; break;
        default: interval_ms = 1000; break;
    }
    return (uint32_t)constrain_int32(interval_ms, 1, 60000);
}

char* AP_KHA::convert_ip_to_str(const uint32_t stream_id, const KHA_IP_PORT_t addr)
{
    const uint32_t ip = IP4_ADDR_VALUE((int)addr.ip[0].get(),(int)addr.ip[1].get(),(int)addr.ip[2].get(),(int)addr.ip[3].get());
    if (ip == _ip_str_last[stream_id]) {
        return _ip_str[stream_id];
    }
    _ip_str_last[stream_id] = ip;

    hal.util->snprintf(_ip_str[stream_id], sizeof(_ip_str[stream_id]), "%d.%d.%d.%d", (int)addr.ip[0].get(),(int)addr.ip[1].get(),(int)addr.ip[2].get(),(int)addr.ip[3].get());
    return _ip_str[stream_id];
}

char* AP_KHA::get_udp_out_data_str(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return get_json_str();
        case 1: return _payload[0].console.eth.enabled ? (char*)"Payload 1 Console payload data" : nullptr;
        case 2: return _payload[1].console.eth.enabled ? (char*)"Payload 1 Console payload data" : nullptr;
    }
    return nullptr;
}

char* AP_KHA::get_json_str()
{
    if (!_ahrs.json.eth.enabled) {
        return nullptr;
    }

    const uint32_t now_ms = AP_HAL::millis();
    for (uint8_t i=0; i<ARRAY_SIZE(_ahrs.json.msgs); i++) {
        if (now_ms - _ahrs.json.msgs[i].last_ms < _ahrs.json.msgs[i].interval_ms) {
            continue;
        }
        _ahrs.json.msgs[i].last_ms = now_ms;

        switch (_ahrs.json.msgs[i].name) {
        case KHA_JSON_Msg::MAIM_VER:
            return (char*)R"({"class":"MAIM_VER","sw":"1.0","dev":"SBG ELLIPSE-N","devhw":"2.4","devsw":"6.5"})";

        case KHA_JSON_Msg::STATUS:
            return (char*)R"({"class":"STATUS","general":"7F","com":"17FFFFFF","aiding":"3FFF","utc":"64","imu":"17E","mag":"0C5","sol":"1234CC7","vel":"C3","pos":"FFABC","alt":"3"})";

        case KHA_JSON_Msg::IMUNAV:
            return (char*)R"({"class":"IMUNAV","veln":-175.135,"vele":-22.0,"veld":-4.234})";

        case KHA_JSON_Msg::PRESSURE:
            return (char*)R"({"class":"PRESSURE","pressure":101325.0,"alt":0.0})";

        case KHA_JSON_Msg::TPV:
            return (char*)R"({"class":"TPV","time":"2017-05-15T10:30:43.123Z","ept":500, "track":123.45,"lat":12.12345,"lon":-12.12345,"alt":12345.12, "mode":3,"epx":12.12,"epy":12.12,"epv":12.12,"climb":-4.234, "epd":12.345,"epc":12.345})";

        case KHA_JSON_Msg::ATT:
            return (char*)R"({"class":"ATT","acc_x":3.123,"acc_y":2.123,"acc_z":-1.456,"gyro_x":1.456, "gyro_y":2.789,"gyro_z":3.567,"temp":12.12,"mag_x":123.456,"mag_y":234.789, "mag_z":24.223,"roll":3.001,"pitch":-0.345,"yaw":-2.789,"heading":123.45})";

        case KHA_JSON_Msg::SKY:
            return (char*)R"({"class":"SKY","time":"2017-05-15T10:30:43.123Z","hdop":6.3})";

        case KHA_JSON_Msg::ADDL:
            return (char*)R"({"class\":"ADDL\",\"up\":1345786201,\"tow\":375218453,"und":3.7,"gveln":-175.135,"gvele":-22.0,"gveld":-4.234,"epn":4.75,"epe":1.66,"epd":0.37,"nsv":7})";
        }
        return nullptr;
    }
    return nullptr;
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
