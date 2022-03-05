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
    
    enum class KHA_MAIM_Maint_Serial_Protocol_t : uint8_t {
        NONE            = 0,
        PAYLOAD1        = 1,
        PAYLOAD2        = 2,
        AIRCRAFT        = 3,
        AHRS            = 4,
        SLCAN           = 5,
    } ;

    enum class KHA_Vehicle_Type_t : uint8_t {
        MAIM            = 0,
        Plane           = 1,
        Plane_VTOL      = 2,
    } ;


    struct KHA_IP_t {
        AP_Int16 ip[4];
    };
    struct KHA_IP_PORT_t {
        AP_Int16 ip[4];
        AP_Int32 port;
    };

    AP_Enum<KHA_Vehicle_Type_t> _type;

    struct KHA_MAIM_t {

        struct KHA_MAIM_Payload_t {
            struct {
                KHA_IP_PORT_t eth_out;
                AP_Int8 eth_out_enabled_at_boot;
                bool eth_out_enabled;
                AP_HAL::UARTDriver *port;
            } console;

            struct {
                AP_Int8 enabled_at_boot;
                bool enabled;
                float voltage;
                float current;
            } power;

            AP_HAL::UARTDriver *state_port;
            bool zeroize;
            uint32_t pps_start_ms;
            uint32_t pps_on_ms;
            uint32_t pps_off_ms;
        } payload[AP_KHA_MAIM_PAYLOAD_COUNT_MAX];

        struct KHA_MAIM_Maint_t {
            AP_Enum<KHA_MAIM_Maint_Serial_Protocol_t> protocol;
            AP_HAL::UARTDriver *port;
        } maint;

        struct KHA_MAIM_Avionics_t {
            AP_Enum<KHA_MAIM_Maint_Serial_Protocol_t> protocol;
            AP_HAL::UARTDriver *port;
        } avionics;

        struct {
            KHA_IP_PORT_t eth_out;
            AP_Int8 enabled_at_boot;
            bool enabled;
            uint32_t timer_ms;
        } state_json;
    } _maim;

private:
    static AP_KHA *_singleton;

    struct {
        uint32_t timer_ms;
        bool done;
        uint8_t state;
    } _init;

};

namespace AP {
    AP_KHA *kha(void);
};
