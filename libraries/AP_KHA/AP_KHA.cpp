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
#include <stdio.h>

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


    AP_GROUPINFO("IP0"         ,  2, AP_KHA, _maim.my_ip.ip[0], 172),
    AP_GROUPINFO("IP1"         ,  3, AP_KHA, _maim.my_ip.ip[1], 20),
    AP_GROUPINFO("IP2"         ,  4, AP_KHA, _maim.my_ip.ip[2], 13),
    AP_GROUPINFO("IP3"         ,  5, AP_KHA, _maim.my_ip.ip[3], 90),

    AP_GROUPINFO("NETMASK"     ,  6, AP_KHA, _maim.my_netmask, 16),

    AP_GROUPINFO("GWP0"        ,  7, AP_KHA, _maim.my_gateway.ip[0], 172),
    AP_GROUPINFO("GW1"         ,  8, AP_KHA, _maim.my_gateway.ip[1], 20),
    AP_GROUPINFO("GW2"         ,  9, AP_KHA, _maim.my_gateway.ip[2], 1),
    AP_GROUPINFO("GW3"         , 10, AP_KHA, _maim.my_gateway.ip[3], 1),


    AP_GROUPINFO("JSON_IP0"    , 11, AP_KHA, _maim.state_json.eth_out.ip[0], 239),
    AP_GROUPINFO("JSON_IP1"    , 12, AP_KHA, _maim.state_json.eth_out.ip[1], 2),
    AP_GROUPINFO("JSON_IP2"    , 13, AP_KHA, _maim.state_json.eth_out.ip[2], 3),
    AP_GROUPINFO("JSON_IP3"    , 14, AP_KHA, _maim.state_json.eth_out.ip[3], 1),
    AP_GROUPINFO("JSON_PORT"   , 15, AP_KHA, _maim.state_json.eth_out.port, 6969),
    AP_GROUPINFO("JSON_ENABLE" , 16, AP_KHA, _maim.state_json.enabled_at_boot, 1),



    AP_GROUPINFO("MAINT_PROTO" , 17, AP_KHA, _maim.maint.protocol, (int8_t)KHA_MAIM_Maint_Serial_Protocol_t::NONE),

    AP_GROUPINFO("AVION_PROTO" , 18, AP_KHA, _maim.avionics.protocol, (int8_t)KHA_MAIM_Maint_Serial_Protocol_t::NONE),


    AP_GROUPINFO("P1_C_IP1"    , 19, AP_KHA, _maim.payload[0].console.eth_out.ip[0], 255),
    AP_GROUPINFO("P1_C_IP1"    , 20, AP_KHA, _maim.payload[0].console.eth_out.ip[1], 255),
    AP_GROUPINFO("P1_C_IP2"    , 21, AP_KHA, _maim.payload[0].console.eth_out.ip[2], 255),
    AP_GROUPINFO("P1_C_IP3"    , 22, AP_KHA, _maim.payload[0].console.eth_out.ip[3], 255),
    AP_GROUPINFO("P1_C_PORT"   , 23, AP_KHA, _maim.payload[0].console.eth_out.port, 11222),
    AP_GROUPINFO("P1_C_ENABLE" , 24, AP_KHA, _maim.payload[0].console.eth_out_enabled_at_boot, 0),
    AP_GROUPINFO("P1_P_ENABLE" , 25, AP_KHA, _maim.payload[0].power.enabled_at_boot, 1),

#if AP_KHA_MAIM_PAYLOAD_COUNT_MAX >= 2
    AP_GROUPINFO("P2_C_IP1"    , 26, AP_KHA, _maim.payload[1].console.eth_out.ip[0], 255),
    AP_GROUPINFO("P2_C_IP1"    , 27, AP_KHA, _maim.payload[1].console.eth_out.ip[1], 255),
    AP_GROUPINFO("P2_C_IP2"    , 28, AP_KHA, _maim.payload[1].console.eth_out.ip[2], 255),
    AP_GROUPINFO("P2_C_IP3"    , 29, AP_KHA, _maim.payload[1].console.eth_out.ip[3], 255),
    AP_GROUPINFO("P2_C_PORT"   , 30, AP_KHA, _maim.payload[1].console.eth_out.port, 11222),
    AP_GROUPINFO("P2_C_ENABLE" , 31, AP_KHA, _maim.payload[1].console.eth_out_enabled_at_boot, 0),
    AP_GROUPINFO("P2_P_ENABLE" , 32, AP_KHA, _maim.payload[1].power.enabled_at_boot, 1),
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

    // for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS;i++) {
    //     if (_port_loopback[i] == nullptr) {
    //         _port_loopback[i] = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Loopback, i);
    //     }
    // }

    _init.done = true;
}

void AP_KHA::update(void)
{
    if (!_init.done) {
        init();
        return;
    }

    switch (_type.get()) {
        default:
        break;
    }

    //service_loopback();
}

// void AP_KHA::service_loopback()
// {
//     for (uint8_t i=0; i<SERIALMANAGER_NUM_PORTS;i++) {
//         if (_port_loopback[i] == nullptr) {
//             continue;
//         }

//         uint32_t nbytes = _port_loopback[i]->available();
//         while (nbytes-- > 0) {
//             const int16_t data = _port_loopback[i]->read();
//             if (data < 0) {
//                 break;
//             }
            
//             if (_param.serial_loopback_broadcast) {
//                 for (uint8_t j=0; j<SERIALMANAGER_NUM_PORTS; j++) {
//                     if (_port_loopback[j] == nullptr) {
//                         continue;
//                     }
//                     _port_loopback[j]->write(data+1);
//                 }
//             } else {
//                 _port_loopback[i]->write(data+1);
//             }
//         }
//     }

// }

// singleton instance
AP_KHA *AP_KHA::_singleton;

namespace AP
{

AP_KHA *kha()
{
    return AP_KHA::get_singleton();
}

}
