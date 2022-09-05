#pragma once
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
  temperature calibration library. This monitors temperature changes
  and opportunistically calibrates sensors when the vehicle is still
 */

#include <AP_HAL/AP_HAL.h>

#ifndef AP_MODPAYLOAD_ENABLED
#define AP_MODPAYLOAD_ENABLED 0
#endif

#if AP_MODPAYLOAD_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define AP_MODPAYLOAD_PAYLOAD_COUNT_MAX 2

#if AP_MODPAYLOAD_PAYLOAD_COUNT_MAX <= 0
#error "AP_MODPAYLOAD_PAYLOAD_COUNT_MAX must be at least 1"
#endif

class AP_ModPayload
{
public:
    // constructor.  This remains because construction of Copter's g2
    // object becomes problematic if we don't have at least one object
    // to initialise
    AP_ModPayload();

    /* Do not allow copies */
    AP_ModPayload(const AP_ModPayload &other) = delete;
    AP_ModPayload &operator=(const AP_ModPayload&) = delete;

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    static AP_ModPayload *get_singleton(void) { return _singleton; }

    void update(void);

    void init();
    bool is_enabled();

    enum class ModPayload_POWER_TARGET : uint8_t {
        SYSTEM      = 0,
        PAYLOAD1    = 1,
        PAYLOAD2    = 2,
        PAYLOAD3    = 3,
        PAYLOAD4    = 4,
    };

    float get_voltage(const uint8_t instance);
    float get_current(const uint8_t instance);
    

    char* get_udp_out_ip(const uint32_t stream_id);
    char* get_udp_out_name(const uint32_t stream_id);
    uint16_t get_udp_out_port(const uint32_t stream_id);
    uint32_t get_udp_out_interval_ms(const uint32_t stream_id);

    void set_enable(const uint32_t index, const bool value);
    bool get_enable(const uint32_t index) const;

    enum class ModPayload_Routing : uint8_t {
        NONE                    = 0,
        PAYLOAD1_STATE          = 1,
        PAYLOAD1_CONSOLE_SERIAL = 2,
        PAYLOAD1_CONSOLE_IP     = 3,
        PAYLOAD2_STATE          = 4,
        PAYLOAD2_CONSOLE_SERIAL = 5,
        PAYLOAD2_CONSOLE_IP     = 6,
        PAYLOAD3_STATE          = 7,
        PAYLOAD3_CONSOLE_SERIAL = 8,
        PAYLOAD3_CONSOLE_IP     = 9,
        PAYLOAD4_STATE          = 10,
        PAYLOAD4_CONSOLE_SERIAL = 11,
        PAYLOAD4_CONSOLE_IP     = 12,
        AVIONICS                = 13,
        MAINT                   = 14,
        AHRS_Byte_Passthrough   = 15,
        AHRS_Packet_Parsing     = 16,
        AHRS_JSON               = 17,
        SLCAN                   = 18,
        MAVLink                 = 19,
        LOOPBACK                = 20,
    };

    enum class ModPayload_Vehicle_Type_t : uint8_t {
        DISABLED        = 0,
        MAIM            = 1,
        Plane           = 2,
        Plane_VTOL      = 3,
    };

    struct ModPayload_IP_t {
        AP_Int16 ip[4];
    };
    struct ModPayload_IP_PORT_t {
        AP_Int16 ip[4];
        AP_Int32 port;
    };

private:


    enum class ModPayload_JSON_Msg : uint8_t {
        MAIM_VER,
        STATUS,
        IMUNAV,
        PRESSURE,
        TPV,
        ATT,
        SKY,
        ADDL,
    };

    struct ModPayload_JSON_Msg_Interval {
        const ModPayload_JSON_Msg name;
        const uint32_t interval_ms;
        uint32_t last_ms;
    };

    

    struct ModPayload_Uart {
        struct {
            uint8_t data[256];
            uint16_t len;
        } bytes_in;
        struct {
            uint8_t data[256];
            uint16_t len;
        } bytes_out;
        AP_HAL::UARTDriver *port;
    };

    struct ModPayload_Power_t {
        float voltage;
        float current;
        float voltage_smoothed;
        float current_smoothed;
        uint32_t last_ms;
    };

    struct {
        struct {
            struct {
                ModPayload_IP_PORT_t addr;
                AP_Int8 enabled_at_boot;
                bool enabled;
                AP_Int32 interval_ms;
            } eth;
            ModPayload_Uart uart;
            AP_Enum<ModPayload_Routing> route;
        } console;

        struct {
            ModPayload_Uart uart;
            AP_Enum<ModPayload_Routing> route;
        } state;

        struct {
            bool valid;
            AP_Int8 valid_pin;
            AP_Int8 enable_pin;
            AP_Int8 enabled_at_boot;
            bool enabled;
            ModPayload_Power_t data;
        } power;
    } _payload[AP_MODPAYLOAD_PAYLOAD_COUNT_MAX];

    struct {
        ModPayload_Uart uart;
        AP_Enum<ModPayload_Routing> route; // Who talks to Avionics/Maint? Maint, Avionics, State 1, State2, Console1, or Console2
    } _maint, _avionics;

    struct {
        struct {
            struct {
                ModPayload_IP_PORT_t addr;
                AP_Int8 enabled_at_boot;
                bool enabled;
                AP_Int32 interval_ms;
            } eth;
            struct {
                uint8_t data[1500]; // Ethernet MTU. See MOD spec 1.1.4.2.1
                uint16_t len;
            } bytes_out;
            uint32_t timer_ms;
            
            ModPayload_JSON_Msg_Interval msgs[8] = {
                {ModPayload_JSON_Msg::MAIM_VER, 1000, 0},
                {ModPayload_JSON_Msg::STATUS, 1000, 0},
                {ModPayload_JSON_Msg::IMUNAV, 100, 0},
                {ModPayload_JSON_Msg::PRESSURE, 1000, 0},
                {ModPayload_JSON_Msg::TPV, 1000, 0},
                {ModPayload_JSON_Msg::ATT, 100, 0},
                {ModPayload_JSON_Msg::SKY, 1000, 0},
                {ModPayload_JSON_Msg::ADDL, 1000, 0} };
        } json;
        ModPayload_Uart uart;
        AP_Enum<ModPayload_Routing> route; // Who talks to AHRS? Maint, Avionics, State 1 or State2
    } _ahrs;

    struct {
        struct {
            struct {
                uint32_t timestamp_high_us;
                uint32_t timestamp_low_us;
                bool state;
                AP_Int8 pin;
            } in;
        } pps;
        
        struct {
            ModPayload_Power_t data;
        } power;

        struct {
            AP_Int8 pin;
            bool active;
        } zeroize;
    } _system;

    AP_Int8 _ignore_uavcan_gpio_relay_commands;

    static AP_ModPayload *_singleton;
    
    void service_input_uarts(const uint32_t now_ms);
    void service_output_uarts(const uint32_t now_ms);
    void service_router(const uint32_t now_ms);
    // char* convert_ip_to_str(const uint32_t stream_id, const MP_IP_PORT_t addr);

    void service_json_out(const uint32_t now_ms);
    void generate_and_send_json(const ModPayload_JSON_Msg msg_name);

    void housekeeping_system(const uint32_t now_ms);
    static void update_power(const uint32_t now_ms, ModPayload_Power_t &power_data, const uint8_t battery_instance);
    void housekeeping_payload(const uint32_t now_ms, const uint8_t index);

    uint32_t get_udp_out_data(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max);

    void pps_pin_irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us);

    struct {
        uint32_t timer_ms;
        bool done;
        uint8_t state;
    } _init;

    AP_Enum<ModPayload_Vehicle_Type_t> _type;

    uint32_t _ip_str_last[10];
    char _ip_str[10][256];
};


namespace AP {
    AP_ModPayload *mod_payload();
};

#endif // #if AP_MODPAYLOAD_ENABLED
