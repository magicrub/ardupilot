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

#include "AP_ModPayload.h"
#if AP_MODPAYLOAD_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_Networking/AP_Networking.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Common/AP_FWVersion.h>

#include "AP_AHRS/AP_AHRS.h"
#include "AP_ExternalAHRS/AP_ExternalAHRS.h"
#include "AP_ExternalAHRS/AP_ExternalAHRS_SBG.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RTC/AP_RTC.h>

extern const AP_HAL::HAL& hal;

// singleton instance
AP_ModPayload *AP_ModPayload::_singleton;

#define ModPayload_DEBUG 0

#if ModPayload_DEBUG
# define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\r\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
# define debug(fmt, args ...)
#endif

// table of user settable and learned parameters
const AP_Param::GroupInfo AP_ModPayload::var_info[] = {

  //             "XX_TWELVE_XX"
  //AP_GROUPINFO("XXXXXXXXXXXX", XX, AP_ModPayload, _type, 0), // max name char length: MP_ + 5

    AP_GROUPINFO_FLAGS("TYPE"  ,  0, AP_ModPayload, _type, (uint8_t)ModPayload_Vehicle_Type_t::DISABLED, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("ZERO_PIN"    ,  2, AP_ModPayload, _system.zeroize.pin, 20),
    AP_GROUPINFO("NO_EXT_GPIO" ,  3, AP_ModPayload, _ignore_uavcan_gpio_relay_commands, 0),
    AP_GROUPINFO("ROUTE_MAINT" ,  4, AP_ModPayload, _maint.route, (uint8_t)ModPayload_Routing::NONE),
    AP_GROUPINFO("ROUTE_AVION" ,  5, AP_ModPayload, _avionics.route, (uint8_t)ModPayload_Routing::NONE),
    AP_GROUPINFO("ROUTE_P1_CON",  6, AP_ModPayload, _payload[0].console.route, (uint8_t)ModPayload_Routing::PAYLOAD1_CONSOLE_IP),
    AP_GROUPINFO("ROUTE_P2_CON",  7, AP_ModPayload, _payload[1].console.route, (uint8_t)ModPayload_Routing::PAYLOAD2_CONSOLE_IP),
    AP_GROUPINFO("ROUTE_AHRS"  ,  8, AP_ModPayload, _ahrs.route, (uint8_t)ModPayload_Routing::PAYLOAD1_CONSOLE_SERIAL),

    AP_GROUPINFO("JSON_IP1"    , 11, AP_ModPayload, _ahrs.json.eth.addr.ip[0], 239),
    AP_GROUPINFO("JSON_IP2"    , 12, AP_ModPayload, _ahrs.json.eth.addr.ip[1], 255),
    AP_GROUPINFO("JSON_IP3"    , 13, AP_ModPayload, _ahrs.json.eth.addr.ip[2], 1),
    AP_GROUPINFO("JSON_IP4"    , 14, AP_ModPayload, _ahrs.json.eth.addr.ip[3], 1),
    AP_GROUPINFO("JSON_PORT"   , 15, AP_ModPayload, _ahrs.json.eth.addr.port, 6969),
    AP_GROUPINFO("JSON_EN"     , 16, AP_ModPayload, _ahrs.json.eth.enabled_at_boot, 0),
    AP_GROUPINFO("JSON_RATE"   , 17, AP_ModPayload, _ahrs.json.eth.interval_ms, 100),

#if AP_MODPAYLOAD_PAYLOAD_COUNT_MAX >= 1
    AP_GROUPINFO("P1_CON_IP1"  , 20, AP_ModPayload, _payload[0].console.eth.addr.ip[0], 239),
    AP_GROUPINFO("P1_CON_IP2"  , 21, AP_ModPayload, _payload[0].console.eth.addr.ip[1], 2),
    AP_GROUPINFO("P1_CON_IP3"  , 22, AP_ModPayload, _payload[0].console.eth.addr.ip[2], 3),
    AP_GROUPINFO("P1_CON_IP4"  , 23, AP_ModPayload, _payload[0].console.eth.addr.ip[3], 2),
    AP_GROUPINFO("P1_CON_PORT" , 24, AP_ModPayload, _payload[0].console.eth.addr.port, 7000),
    AP_GROUPINFO("P1_CON_EN"   , 25, AP_ModPayload, _payload[0].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P1_POW_V_PIN", 26, AP_ModPayload, _payload[0].power.valid_pin, 1),
    AP_GROUPINFO("P1_POW_E_PIN", 27, AP_ModPayload, _payload[0].power.enable_pin, 3),
    AP_GROUPINFO("P1_POW_ENABL", 28, AP_ModPayload, _payload[0].power.enabled_at_boot, 1),
    AP_GROUPINFO("P1_CON_RATE" , 29, AP_ModPayload, _payload[0].console.eth.interval_ms, 100),
#endif

#if AP_MODPAYLOAD_PAYLOAD_COUNT_MAX >= 2
    AP_GROUPINFO("P2_CON_IP1"  , 30, AP_ModPayload, _payload[1].console.eth.addr.ip[0], 239),
    AP_GROUPINFO("P2_CON_IP2"  , 31, AP_ModPayload, _payload[1].console.eth.addr.ip[1], 2),
    AP_GROUPINFO("P2_CON_IP3"  , 32, AP_ModPayload, _payload[1].console.eth.addr.ip[2], 3),
    AP_GROUPINFO("P2_CON_IP4"  , 33, AP_ModPayload, _payload[1].console.eth.addr.ip[3], 3),
    AP_GROUPINFO("P2_CON_PORT" , 34, AP_ModPayload, _payload[1].console.eth.addr.port, 7001),
    AP_GROUPINFO("P2_CON_EN"   , 35, AP_ModPayload, _payload[1].console.eth.enabled_at_boot, 0),
    AP_GROUPINFO("P2_POW_V_PIN", 36, AP_ModPayload, _payload[1].power.valid_pin, 2),
    AP_GROUPINFO("P2_POW_E_PIN", 37, AP_ModPayload, _payload[1].power.enable_pin, 4),
    AP_GROUPINFO("P2_POW_ENABL", 38, AP_ModPayload, _payload[1].power.enabled_at_boot, 1),
    AP_GROUPINFO("P2_CON_RATE" , 39, AP_ModPayload, _payload[1].console.eth.interval_ms, 100),
#endif // AP_MODPAYLOAD_PAYLOAD_COUNT_MAX

#if AP_MODPAYLOAD_PAYLOAD_COUNT_MAX >= 3
    // 40-49
#endif

#if AP_MODPAYLOAD_PAYLOAD_COUNT_MAX >= 4
    // 50-59
#endif

    AP_GROUPEND
};

AP_ModPayload::AP_ModPayload(void)
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ModPayload must be singleton");
    }
#endif
    _singleton = this;
}

void AP_ModPayload::init(void)
{
    if (_init.done || !is_enabled()) {
        return;
    }

    _ahrs.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ModPayload_AHRS_Sensor, 0);

    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
        _payload[p].state.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ModPayload_Payload_State, p);
        _payload[p].console.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ModPayload_Payload_Console, p);
    }

    _maint.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ModPayload_Maint, 0);
    _avionics.uart.port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ModPayload_Avionics, 0);


    if (_system.pps.in.pin >= 0) {
        hal.gpio->pinMode(_system.pps.in.pin, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(
                _system.pps.in.pin,
                FUNCTOR_BIND_MEMBER(&AP_ModPayload::pps_pin_irq_handler,
                                    void,
                                    uint8_t,
                                    bool,
                                    uint32_t),
                AP_HAL::GPIO::INTERRUPT_BOTH)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ModPayload: Failed to attach to pin %u", (unsigned)_system.pps.in.pin);
        }
        _system.pps.in.state = hal.gpio->read(_system.pps.in.pin);
    }

    _avionics.uart.bytes_in.len = 0;
    _maint.uart.bytes_in.len = 0;
    _ahrs.uart.bytes_in.len = 0;

    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
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

bool AP_ModPayload::is_enabled()
{
    switch ((ModPayload_Vehicle_Type_t)_type.get()) {
        case ModPayload_Vehicle_Type_t::MAIM:
            // this is the only support vehicle
            return true;

        case ModPayload_Vehicle_Type_t::DISABLED:
        case ModPayload_Vehicle_Type_t::Plane:
        case ModPayload_Vehicle_Type_t::Plane_VTOL:
        default:
            // not supported, do nothing
            break;
    }
    return false;
}

void AP_ModPayload::update(void)
{
    if (!is_enabled()) {
        return;
    }
    if (!_init.done) {
        init();
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    housekeeping_system(now_ms);

    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
        housekeeping_payload(now_ms, p);
    }


    service_input_uarts(now_ms);

    service_router(now_ms);

    service_output_uarts(now_ms);

    service_json_out(now_ms);
}

void AP_ModPayload::housekeeping_system(const uint32_t now_ms)
{
    _avionics.uart.bytes_in.len = _avionics.uart.bytes_out.len = 0;
    _maint.uart.bytes_in.len = _maint.uart.bytes_out.len = 0;
    _ahrs.uart.bytes_in.len = _ahrs.uart.bytes_out.len = 0;

    update_power(now_ms, _system.power.data, (uint8_t)ModPayload_POWER_TARGET::SYSTEM);
}

void AP_ModPayload::update_power(const uint32_t now_ms, ModPayload_Power_t &power_data, const uint8_t battery_instance)
{
    if (now_ms - power_data.last_ms < 100) {
        return;
    }
    power_data.last_ms = now_ms;

    if (!AP::battery().healthy(battery_instance)) {
        power_data.voltage =  power_data.voltage_smoothed = 0;
        power_data.current =  power_data.current_smoothed = 0;
    } else {
        const float LPF_coef = 0.80;

        const float voltage = AP::battery().voltage(battery_instance);
        power_data.voltage = MAX(0,voltage);
        power_data.voltage_smoothed = (LPF_coef*power_data.voltage_smoothed) + ((1.0f-LPF_coef)*power_data.voltage);

        float current;
        if (!AP::battery().current_amps(current, battery_instance)) {
            power_data.current = power_data.current_smoothed = 0;
        } else {
            power_data.current = MAX(0,current);
            power_data.current_smoothed = (LPF_coef*power_data.current_smoothed) + ((1.0f-LPF_coef)*power_data.current);
        }
    }
}

void AP_ModPayload::housekeeping_payload(const uint32_t now_ms, const uint8_t index)
{
    if (index >= AP_MODPAYLOAD_PAYLOAD_COUNT_MAX) {
        return;
    }
    _payload[index].power.valid = hal.gpio->read(_payload[index].power.valid_pin);

    // clear memory buffers by setting all lengths to zero
    _payload[index].state.uart.bytes_in.len = _payload[index].state.uart.bytes_out.len = 0;
    _payload[index].console.uart.bytes_in.len = _payload[index].console.uart.bytes_out.len = 0;

    update_power(now_ms, _payload[index].power.data, (uint8_t)(ModPayload_POWER_TARGET::PAYLOAD1)+index);
}

void AP_ModPayload::service_input_uarts(const uint32_t now_ms)
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

    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
        if (_payload[p].state.uart.port != nullptr) {
            _payload[p].state.uart.bytes_in.len = _payload[p].state.uart.port->read(_payload[p].state.uart.bytes_in.data, sizeof(_payload[p].state.uart.bytes_in.data));
        }

        if (_payload[p].console.uart.port != nullptr) {
            _payload[p].console.uart.bytes_in.len = _payload[p].console.uart.port->read(_payload[p].console.uart.bytes_in.data, sizeof(_payload[p].console.uart.bytes_in.data));
        }
    }
}


void AP_ModPayload::service_router(const uint32_t now_ms)
{
    uint16_t len = 0;

    // if (_avionics.uart.port != nullptr) {
    //     _avionics.uart.bytes_in.len = _avionics.uart.port->read(_avionics.uart.bytes_in.data, sizeof(_avionics.uart.bytes_in.data));
    // }

    // if (_maint.uart.port != nullptr) {
    //     _maint.uart.bytes_in.len = _maint.uart.port->read(_maint.uart.bytes_in.data, sizeof(_maint.uart.bytes_in.data));
    // }
    
    // copy ahrs.in -> state[p].out
    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_payload[p].state.uart.bytes_out.data));
        memcpy(_payload[p].state.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _payload[p].state.uart.bytes_out.len += len;
    }

    // // copy state[p].in -> ahrs.out
    // for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
    //     // if more than one channel writes to "_ahrs.uart.bytes_out", then we need to append it so we don't clobber the data
    //     // This is likely ruining ther packets anyway so this is best done as locked access of one-on-one at a time
    //     const uint16_t offset = _ahrs.uart.bytes_out.len;
    //     const uint16_t len = MIN(_payload[p].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data) - offset);
    //     memcpy(&_ahrs.uart.bytes_out.data[offset], _payload[p].state.uart.bytes_in.data, len);
    //     _payload[p].state.uart.bytes_out.len += len;
    // }

    switch ((ModPayload_Routing)_ahrs.route.get()) {
    case ModPayload_Routing::PAYLOAD1_STATE:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::PAYLOAD2_STATE:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::AVIONICS:
        len = MIN(_avionics.uart.bytes_in.len, sizeof(_ahrs.uart.bytes_out.data));
        memcpy(_ahrs.uart.bytes_out.data, _avionics.uart.bytes_in.data, len);
        _ahrs.uart.bytes_out.len += len;
        break;
    default:
        break;
    }


    switch ((ModPayload_Routing)_maint.route.get()) {
    case ModPayload_Routing::PAYLOAD1_CONSOLE_SERIAL:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::PAYLOAD2_CONSOLE_SERIAL:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::AHRS_Byte_Passthrough:
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::AVIONICS:
        len = MIN(_avionics.uart.bytes_in.len, sizeof(_maint.uart.bytes_out.data));
        memcpy(_maint.uart.bytes_out.data, _avionics.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    default:
        break;
    }

    switch ((ModPayload_Routing)_avionics.route.get()) {
    case ModPayload_Routing::PAYLOAD1_CONSOLE_SERIAL:
        len = MIN(_payload[0].state.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _payload[0].state.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::PAYLOAD2_CONSOLE_SERIAL:
        len = MIN(_payload[1].state.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _payload[1].state.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::AHRS_Byte_Passthrough:
        len = MIN(_ahrs.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _ahrs.uart.bytes_in.data, len);
        _maint.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_avionics.uart.bytes_out.data));
        memcpy(_avionics.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _avionics.uart.bytes_out.len += len;
        break;
    default:
        break;
    }



    switch ((ModPayload_Routing)_payload[0].console.route.get()) {
    case ModPayload_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_payload[0].console.uart.bytes_out.data));
        memcpy(_payload[0].console.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _payload[0].console.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::PAYLOAD1_CONSOLE_IP:
        // len = MIN(_payload[0].console.eth.buf_in.available(), sizeof(_payload[0].console.uart.bytes_out.data));
        // _payload[0].console.uart.bytes_out.len = _payload[0].console.eth.buf_in.read(_payload[0].console.uart.bytes_out.data, len);

        // len = MIN(_payload[0].console.uart.bytes_in.len, _payload[0].console.eth.buf_in.space());
        // _payload[0].console.eth.buf_out.write(_payload[0].console.uart.bytes_in.data, len);
        break;
    default:
        break;
    }

    switch ((ModPayload_Routing)_payload[1].console.route.get()) {
    case ModPayload_Routing::MAINT:
        len = MIN(_maint.uart.bytes_in.len, sizeof(_payload[1].console.uart.bytes_out.data));
        memcpy(_payload[1].console.uart.bytes_out.data, _maint.uart.bytes_in.data, len);
        _payload[1].console.uart.bytes_out.len += len;
        break;
    case ModPayload_Routing::PAYLOAD2_CONSOLE_IP:
        // len = MIN(_payload[1].console.eth.buf_in.available(), sizeof(_payload[1].console.uart.bytes_out.data));
        // _payload[1].console.uart.bytes_out.len = _payload[1].console.eth.buf_in.read(_payload[1].console.uart.bytes_out.data, len);

        // len = MIN(_payload[1].console.uart.bytes_in.len, _payload[1].console.eth.buf_in.space());
        // _payload[1].console.eth.buf_out.write(_payload[1].console.uart.bytes_in.data, len);
        break;
    default:
        break;
    }

}

void AP_ModPayload::service_output_uarts(const uint32_t now_ms)
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

    for (uint8_t p=0; p<AP_MODPAYLOAD_PAYLOAD_COUNT_MAX; p++) {
        if (_payload[p].state.uart.port != nullptr && _payload[p].state.uart.bytes_out.len > 0) {
            _payload[p].state.uart.port->write(_payload[p].state.uart.bytes_out.data, _payload[p].state.uart.bytes_out.len);
        }
        if (_payload[p].console.uart.port != nullptr && _payload[p].console.uart.bytes_out.len > 0) {
            _payload[p].console.uart.port->write(_payload[p].console.uart.bytes_out.data, _payload[p].console.uart.bytes_out.len);
        }
    }
}

void AP_ModPayload::pps_pin_irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us)
{
    if (pin != _system.pps.in.pin) {
        return;
    }

    _system.pps.in.state = pin_value;
    if (pin_value) {
        _system.pps.in.timestamp_high_us = timestamp_us;
    } else {
        _system.pps.in.timestamp_low_us = timestamp_us;
    }
}


void AP_ModPayload::set_enable(const uint32_t index, const bool value)
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

bool AP_ModPayload::get_enable(const uint32_t index) const
{
    bool result = false;
    switch (index) {
    case 1:
        result = hal.gpio->read(_payload[0].power.enable_pin);
        break;
    case 2:
        result = hal.gpio->read(_payload[1].power.enable_pin);
        break;
    }
    return result;
}

char* AP_ModPayload::get_udp_out_ip(const uint32_t stream_id)
{
    // switch (stream_id) {
    //     case 0: return convert_ip_to_str(stream_id, _ahrs.json.eth.addr);
    //     case 1: return convert_ip_to_str(stream_id, _payload[0].console.eth.addr);
    //     case 2: return convert_ip_to_str(stream_id, _payload[1].console.eth.addr);
    // }
    return (char*)"1.2.3.4";
}

uint16_t AP_ModPayload::get_udp_out_port(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return _ahrs.json.eth.addr.port;
        case 1: return _payload[0].console.eth.addr.port;
        case 2: return _payload[1].console.eth.addr.port;
    }
    return 1234;
}

char* AP_ModPayload::get_udp_out_name(const uint32_t stream_id)
{
    switch (stream_id) {
        case 0: return (char*)"AHRS JSON";
        case 1: return (char*)"Payload 1 Console";
        case 2: return (char*)"Payload 2 Console";
    }
    return (char*)"Unknown Name";
}

uint32_t AP_ModPayload::get_udp_out_interval_ms(const uint32_t stream_id)
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

// char* AP_ModPayload::convert_ip_to_str(const uint32_t stream_id)
// {
// #if AP_NETWORKING_ENABLED
//     const uint32_t ip = IP4_ADDR_VALUE((int)addr.ip[0].get(),(int)addr.ip[1].get(),(int)addr.ip[2].get(),(int)addr.ip[3].get());
//     if (ip == _ip_str_last[stream_id]) {
//         return _ip_str[stream_id];
//     }
//     _ip_str_last[stream_id] = ip;

//     hal.util->snprintf(_ip_str[stream_id], sizeof(_ip_str[stream_id]), "%d.%d.%d.%d", (int)addr.ip[0].get(),(int)addr.ip[1].get(),(int)addr.ip[2].get(),(int)addr.ip[3].get());
//     return _ip_str[stream_id];
// #else
//     return nullptr;
// #endif
// }

// char* AP_ModPayload::get_udp_out_data_str(const uint32_t stream_id)
// {
//     uint32_t len;
//     switch (stream_id) {
//         case 0:
//             return get_json_str();

//      // case 1: return _payload[0].console.eth.enabled ? (char*)"Payload 1 Console payload data" : nullptr;
//         case 1:
//             if ((ModPayload_Routing)_payload[0].console.route.get() != ModPayload_Routing::PAYLOAD1_CONSOLE_IP) {
//                 break;
//             }
//             len = _payload[0].console.eth.buf_out.available();
//             if (len == 0) {
//                 return nullptr;
//             }
//             // TODO: copy _payload[0].console.eth.buf_out somewhere
//             return nullptr;

//      // case 2: return _payload[1].console.eth.enabled ? (char*)"Payload 2 Console payload data" : nullptr;
//         case 2:
//             if ((ModPayload_Routing)_payload[1].console.route.get() != ModPayload_Routing::PAYLOAD2_CONSOLE_IP) {
//                 break;
//             }
//             len = _payload[1].console.eth.buf_out.available();
//             if (len == 0) {
//                 return nullptr;
//             }
//             // TODO: copy _payload[1].console.eth.buf_out somewhere
//             return nullptr;
//     }
//     return nullptr;
// }

// uint32_t AP_ModPayload::get_udp_out_data(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max)
// {
//     uint32_t len;
//     switch (stream_id) {
//         case 0:
//             len = MIN(get_next_json_str(),data_len_max);
//             memcpy(data, _ahrs.json.bytes_out.data, len);
//             return len;

//      // case 1: return _payload[0].console.eth.enabled ? (char*)"Payload 1 Console payload data" : nullptr;
//         case 1:
//             // if ((ModPayload_Routing)_payload[0].console.route.get() != ModPayload_Routing::PAYLOAD1_CONSOLE_IP) {
//             //     break;
//             // }
//             // len = _payload[0].console.eth.buf_out.available();
//             // if (len == 0) {
//             //     return nullptr;
//             // }
//             // TODO: copy _payload[0].console.eth.buf_out somewhere
//             return 0;

//      // case 2: return _payload[1].console.eth.enabled ? (char*)"Payload 2 Console payload data" : nullptr;
//         case 2:
//             // if ((ModPayload_Routing)_payload[1].console.route.get() != ModPayload_Routing::PAYLOAD2_CONSOLE_IP) {
//             //     break;
//             // }
//             // len = _payload[1].console.eth.buf_out.available();
//             // if (len == 0) {
//             //     return nullptr;
//             // }
//             // TODO: copy _payload[1].console.eth.buf_out somewhere
//             return 0;
//     }
//     return 0;
// }

void AP_ModPayload::service_json_out(const uint32_t now_ms)
{
    if (!_ahrs.json.eth.enabled) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_ahrs.json.msgs); i++) {
        if (now_ms - _ahrs.json.msgs[i].last_ms < _ahrs.json.msgs[i].interval_ms) {
            continue;
        }
        _ahrs.json.msgs[i].last_ms = now_ms;
        generate_and_send_json(_ahrs.json.msgs[i].name);

        // only send one message at a time.
        return;
    }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=1700"
void AP_ModPayload::generate_and_send_json(const ModPayload_JSON_Msg msg_name)
{
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_AHRS)
    return;
#endif

    auto &ahrs = AP::ahrs();
    if (!ahrs.initialised()) {
        return;
    }

    Vector3f mag = Vector3f();
    if (!ahrs.get_mag_field_NED(mag)) {
        mag = AP::compass().get_field();
    }

    Vector3f accel = Vector3f();
#if HAL_INS_ENABLED
    accel = AP::ins().get_accel();
#endif
    const Vector3f gyro = ahrs.get_gyro();

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BARO)
    const temperature = AP::baro().healthy() ? AP::baro().get_temperature() : 0;
#else
    const float temperature = 0;
#endif

    //const float temperature = AP::ins().get_temperature(0);
    //const float temperature = AP::sbg().get_temperature(0);

    unsigned hw = 0;
    unsigned fw = 0;
#if HAL_EXTERNAL_AHRS_SBG_ENABLED
    if (AP::sbg() != nullptr) {
        hw = AP::sbg()->get_version_hardwareRev();
        fw = AP::sbg()->get_version_firmwareRev();
    }
#endif

    char utc_str[50] {};
    const uint16_t utc_str_len = AP::rtc().get_system_clock_utc_string(utc_str, sizeof(utc_str), true);
    (void)utc_str_len;

    float vert_pos_rate = 0;
    UNUSED_RESULT(ahrs.get_vert_pos_rate(vert_pos_rate));

    Location loc = Location();
    UNUSED_RESULT(ahrs.get_location(loc));

    float UNKNOWN_VALUE = 0.0f;

    const Vector3f gps_velocity = AP::gps().velocity();
    Vector3f gps_velocity_error = Vector3f();
    UNUSED_RESULT(AP::gps().horizontal_accuracy(gps_velocity_error.x));
    UNUSED_RESULT(AP::gps().horizontal_accuracy(gps_velocity_error.y));
    UNUSED_RESULT(AP::gps().vertical_accuracy(gps_velocity_error.z));

    switch (msg_name) {
    case ModPayload_JSON_Msg::MAIM_VER: {
        //return (char*)R"({"class":"MAIM_VER","sw":"1.0","dev":"SBG ELLIPSE-N","devhw":"2.4","devsw":"6.5"})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"MAIM_VER\",\"sw\":\"%s\",\"dev\":\"SBG ELLIPSE-N\",\"devhw\":\"%u.%u.%u.%u.%u\",\"devsw\":\"%u.%u.%u.%u.%u\"}",
            AP::fwversion().fw_string,   // sw
            (hw>>31) & 0x1, (hw>>24) & 0x7F, (hw>>16) & 0xFF, (hw>>8)  & 0xFF, hw & 0x00FF,   // devhw
            (fw>>31) & 0x1, (fw>>28) & 0x07, (fw>>22) & 0x3F, (fw>>16) & 0x3F, fw & 0xFFFF    // devsw
            );
        }
        break;

    case ModPayload_JSON_Msg::STATUS:
        // return (char*)R"({"class":"STATUS","general":"7F","com":"17FFFFFF","aiding":"3FFF","utc":"64","imu":"17E","mag":"0C5","sol":"1234CC7","vel":"C3","pos":"FFABC","alt":"3"})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"STATUS\",\"general\":\"%X\",\"com\":\"%X\",\"aiding\":\"%X\",\"utc\":\"%X\",\"imu\":\"%X\",\"mag\":\"%X\",\"sol\":\"%X\",\"vel\":\"%X\",\"pos\":\"%X\",\"alt\":\"%X\"}",
                0x7F,           // general
                0x17FFFFFF,     // com
                0x3FFF,         // aiding
                0x64,           // utc
                0x17E,          // imu
                0x0C5,          // mag
                0x1234CC7,      // sol
                0xC3,           // vel
                0xFFABC,        // pos
                0x3            // alt 
            );
        break;

    case ModPayload_JSON_Msg::IMUNAV: {
        Vector3f vel = Vector3f();
        AP::ahrs().get_velocity_NED(vel);
    //     return (char*)R"({"class":"IMUNAV","veln":-175.135,"vele":-22.0,"veld":-4.234})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data), 
           "{\"class\":\"IMUNAV\",\"veln\":\"%.3f\",\"vele\":\"%.3f\",\"veld\":\"%.3f\"}",
            (double)vel.x,
            (double)vel.y,
            (double)vel.z);
        }
        break;

    case ModPayload_JSON_Msg::PRESSURE:
    //     return (char*)R"({"class":"PRESSURE","pressure":101325.0,"alt":0.0})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"PRESSURE\",\"pressure\":\"%.1f\",\"alt\":\"%.3f\"}",
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BARO)
            (double)AP::baro().get_pressure(), (double)AP::baro().get_altitude());
#else
            0.0, 0.0);
#endif
        break;

    case ModPayload_JSON_Msg::TPV:
    //     return (char*)R"({"class":"TPV","time":"2017-05-15T10:30:43.123Z","ept":500, "track":123.45,"lat":12.12345,"lon":-12.12345,"alt":12345.12, "mode":3,"epx":12.12,"epy":12.12,"epv":12.12,"climb":-4.234, "epd":12.345,"epc":12.345})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"TPV\",\"time\":\"%s\",\"ept\":%u,\"track\":%.2f,\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f,\"mode\":%u,\"epx\":%.2f,\"epy\":%.2f,\"epv\":%.2f,\"climb\":%.3f,\"epd\":%.3f,\"epc\":%.3f}",
            utc_str,
            (unsigned)UNKNOWN_VALUE, // ept - Estimated Timestamp Error, nanoseconds
            (double)wrap_360(degrees(ahrs.groundspeed_vector().angle())),
            (double)(loc.lat*1.0e-7), // lat
            (double)(loc.lng*1.0e-7), // lng
            (double)(loc.alt*1.0e-2), // alt
            (unsigned)constrain_int16(AP::gps().status(), 0, 3), // mode
            (double)AP::gps().get_hdop()*0.01f, // epx
            (double)AP::gps().get_hdop()*0.01f, // epy
            (double)AP::gps().get_vdop()*0.01f, // epv
            (double)vert_pos_rate, // climb
            (double)UNKNOWN_VALUE, // epd - Direction Error Estimate, Degrees (0 - 360)
            (double)UNKNOWN_VALUE); // epc - Climb/Sink Error Estimate, meters per second (Down - Positive)
        break;

    case ModPayload_JSON_Msg::ATT:
    //     return (char*)R"({"class":"ATT","acc_x":3.123,"acc_y":2.123,"acc_z":-1.456,"gyro_x":1.456,"gyro_y":2.789,"gyro_z":3.567,"temp":12.12,"mag_x":123.456,"mag_y":234.789,"mag_z":24.223,"roll":3.001,"pitch":-0.345,"yaw":-2.789,"heading":123.45})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"ATT\",\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f,\"temp\":%.2f,\"mag_x\":%.3f,\"mag_y\":%.3f,\"mag_z\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,\"heading\":%.2f}",
            (double)accel.x,
            (double)accel.y,
            (double)accel.z,
            (double)gyro.x,
            (double)gyro.y,
            (double)gyro.z,
            (double)temperature,
            (double)mag.x,
            (double)mag.y,
            (double)mag.z,
            (double)degrees(ahrs.roll),
            (double)degrees(ahrs.pitch),
            (double)degrees(ahrs.yaw),
            (double)degrees(ahrs.groundspeed_vector().angle()));
        break;

    case ModPayload_JSON_Msg::SKY:
    //     return (char*)R"({"class":"SKY","time":"2017-05-15T10:30:43.123Z","hdop":6.3})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data), 
            "{\"class\":\"SKY\",\"time\":\"%s\",\"hdop\":%.1f}",
            utc_str,
            (double)AP::gps().get_hdop()*0.01f);
        break;

    case ModPayload_JSON_Msg::ADDL:
    //     return (char*)R"({"class\":"ADDL\",\"up\":1345786201,\"tow\":375218453,"und":3.7,"gveln":-175.135,"gvele":-22.0,"gveld":-4.234,"epn":4.75,"epe":1.66,"epd":0.37,"nsv":7})";
        _ahrs.json.bytes_out.len = hal.util->snprintf((char*)_ahrs.json.bytes_out.data, sizeof(_ahrs.json.bytes_out.data),
            "{\"class\":\"ADDL\",\"up\":%u,\"tow\":%u,\"und\":%.1f,\"gveln\":%.3f,\"gvele\":%.1f,\"gveld\":%.3f,\"epn\":%.2f,\"epe\":%.2f,\"epd\":%.2f,\"nsv\":%u}",
            (unsigned)AP_HAL::micros64(), // up
            (unsigned)AP::gps().time_week_ms(), // tow
            (double)UNKNOWN_VALUE, // und - Undulation - Altitude difference between the geoid and the Ellipsoid, Meters (WGS-84 Altitude - MSL Altitude)
            (double)gps_velocity.x, // gveln
            (double)gps_velocity.y, // gvele
            (double)gps_velocity.z, // gveld
            (double)gps_velocity_error.x, // epn
            (double)gps_velocity_error.y, // epe
            (double)gps_velocity_error.y, // epd
            (unsigned)AP::gps().num_sats());// nsv
        break;
    } // switch

    if (_ahrs.json.bytes_out.len == 0) {
        return;
    }

    // TODO: send _ahrs.json.bytes_out.data
}

#pragma GCC diagnostic pop

float AP_ModPayload::get_voltage(const uint8_t instance)
{
    switch (instance) {
        case 0:
            return _system.power.data.voltage_smoothed;
        case 1:
        case 2:
            return _payload[instance-1].power.data.voltage_smoothed;
    }
    return 0;
}

float AP_ModPayload::get_current(const uint8_t instance)
{
    switch (instance) {
        case 0:
            return _system.power.data.current_smoothed;
        case 1:
        case 2:
            return _payload[instance-1].power.data.current_smoothed;
    }
    return 0;
}


AP_ModPayload *AP::mod_payload()
{
    return AP_ModPayload::get_singleton();
}
#endif // #if AP_MODPAYLOAD_ENABLED

