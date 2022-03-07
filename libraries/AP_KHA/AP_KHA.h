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
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define AP_KHA_MAIM_PAYLOAD_COUNT_MAX 2

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX <= 0
#error "AP_KHA_MAIM_PAYLOAD_COUNT_MAX must be at least 1"
#endif

class AP_KHA
{
public:
    // constructor.  This remains because construction of Copter's g2
    // object becomes problematic if we don't have at least one object
    // to initialise
    AP_KHA();

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    static AP_KHA *get_singleton(void) { return _singleton; }

    void update(void);

    void init();

    /* Do not allow copies */
    AP_KHA(const AP_KHA &other) = delete;
    AP_KHA &operator=(const AP_KHA&) = delete;

    char* get_json_str() { return (char*)"This is a test of const string function"; }
    
    void set_gpio(const uint32_t index, const bool value);

    enum class KHA_MAIM_Routing : uint8_t {
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
        AHRS_Bytes              = 15,
        AHRS_Packets            = 16,
        AHRS_JSON               = 17,
        SLCAN                   = 18,
        MAVLink                 = 19,
        LOOPBACK                = 20,
    };

    enum class KHA_Vehicle_Type_t : uint8_t {
        MAIM            = 0,
        Plane           = 1,
        Plane_VTOL      = 2,
    };


    struct KHA_IP_t {
        AP_Int16 ip[4];
    };
    struct KHA_IP_PORT_t {
        AP_Int16 ip[4];
        AP_Int32 port;
    };

    struct KHA_Uart {
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

    struct {
        struct {
            struct {
                KHA_IP_PORT_t addr;
                AP_Int8 enabled_at_boot;
                bool enabled;
            } eth;
            KHA_Uart uart;
            AP_Enum<KHA_MAIM_Routing> route;
        } console;

        struct {
            KHA_Uart uart;
            AP_Enum<KHA_MAIM_Routing> route;
        } state;

        struct {
            bool valid;
            AP_Int8 valid_pin;
            AP_Int8 enable_pin;
            AP_Int8 enabled_at_boot;
            bool enabled;
            float voltage;
            float current;
        } power;
    } _payload[AP_KHA_MAIM_PAYLOAD_COUNT_MAX];

    struct {
        KHA_Uart uart;
        AP_Enum<KHA_MAIM_Routing> route; // Who talks to Avionics/Maint? Maint, Avionics, State 1, State2, Console1, or Console2
    } _maint, _avionics;

    struct {
        struct {
            struct {
                KHA_IP_PORT_t addr;
                AP_Int8 enabled_at_boot;
                bool enabled;
            } eth;
            struct {
                uint8_t data[256];
                uint16_t len;
            } bytes_out;
            uint32_t timer_ms;
        } json;
        KHA_Uart uart;
        AP_Enum<KHA_MAIM_Routing> route; // Who talks to AHRS? Maint, Avionics, State 1 or State2
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
            float voltage;
            float current;
        } power;

        struct {
            AP_Int8 pin;
            bool active;
        } zeroize;
    } _system;

    AP_Int8 _ignore_uavcan_gpio_relay_commands;

private:
    static AP_KHA *_singleton;
    
    void service_input_uarts();
    void service_output_uarts();
    void service_router();

    void pps_pin_irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp_us);

    struct {
        uint32_t timer_ms;
        bool done;
        uint8_t state;
    } _init;

    AP_Enum<KHA_Vehicle_Type_t> _type;

};

namespace AP {
    AP_KHA *kha(void);
};
