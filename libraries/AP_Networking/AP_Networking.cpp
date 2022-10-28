
#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED

#include "AP_Networking_Serial2UDP.h"


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    #include <hal_mii.h>
    #include <lwip/sockets.h>

#else
    #include <arpa/inet.h>
    // #include <sys/ioctl.h>
    // #include <sys/types.h>
    #include <sys/socket.h>
    // #include <netinet/in.h>
    // #include <netinet/tcp.h>
    // #include <sys/select.h>
    // #include <termios.h>
    // #include <sys/time.h>

#endif

#include <GCS_MAVLink/GCS.h>

#ifndef LWIP_DHCP
#define LWIP_DHCP 0
//#define NET_ADDRESS_STATIC 222
#endif

#ifndef AP_NETWORKING_DEFAULT_IP_ADDR0
#define AP_NETWORKING_DEFAULT_IP_ADDR0     192
#define AP_NETWORKING_DEFAULT_IP_ADDR1     168
#define AP_NETWORKING_DEFAULT_IP_ADDR2       0
#define AP_NETWORKING_DEFAULT_IP_ADDR3      34
#endif

#ifndef AP_NETWORKING_DEFAULT_GW_ADDR0
#define AP_NETWORKING_DEFAULT_GW_ADDR0     192
#define AP_NETWORKING_DEFAULT_GW_ADDR1     168
#define AP_NETWORKING_DEFAULT_GW_ADDR2       0
#define AP_NETWORKING_DEFAULT_GW_ADDR3       1
#endif

#ifndef AP_NETWORKING_DEFAULT_NM_ADDR
#define AP_NETWORKING_DEFAULT_NM_ADDR       24
#endif

#ifndef AP_NETWORKING_DEFAULT_DHCP_ENABLE
#define AP_NETWORKING_DEFAULT_DHCP_ENABLE   LWIP_DHCP
#endif

#ifndef AP_NETWORKING_DEFAULT_MAC_ADDR0
    #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        #define AP_NETWORKING_DEFAULT_MAC_ADDR0   LWIP_ETHADDR_0
        #define AP_NETWORKING_DEFAULT_MAC_ADDR1   LWIP_ETHADDR_1
        #define AP_NETWORKING_DEFAULT_MAC_ADDR2   LWIP_ETHADDR_2
        #define AP_NETWORKING_DEFAULT_MAC_ADDR3   LWIP_ETHADDR_3
        #define AP_NETWORKING_DEFAULT_MAC_ADDR4   LWIP_ETHADDR_4
        #define AP_NETWORKING_DEFAULT_MAC_ADDR5   LWIP_ETHADDR_5
    #else
        #define AP_NETWORKING_DEFAULT_MAC_ADDR0   1
        #define AP_NETWORKING_DEFAULT_MAC_ADDR1   2
        #define AP_NETWORKING_DEFAULT_MAC_ADDR2   3
        #define AP_NETWORKING_DEFAULT_MAC_ADDR3   4
        #define AP_NETWORKING_DEFAULT_MAC_ADDR4   5
        #define AP_NETWORKING_DEFAULT_MAC_ADDR5   6
    #endif
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Networking::var_info[] = {

    // @Param: _ENABLED
    // @DisplayName: Enable Networking
    // @Description: Enable Networking
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLED",  0, AP_Networking, _param.enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _IPADDR0
    // @DisplayName: IP Address MSB
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPADDR0", 1,  AP_Networking,    _param.ipaddr[0],   AP_NETWORKING_DEFAULT_IP_ADDR0),

    // @Param: _IPADDR1
    // @DisplayName: IP Address 2nd byte
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPADDR1", 2,  AP_Networking,    _param.ipaddr[1],   AP_NETWORKING_DEFAULT_IP_ADDR1),

    // @Param: _IPADDR2
    // @DisplayName: IP Address 3rd byte
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPADDR2", 3,  AP_Networking,    _param.ipaddr[2],   AP_NETWORKING_DEFAULT_IP_ADDR2),

    // @Param: _IPADDR3
    // @DisplayName: IP Address LSB
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_IPADDR3", 4,  AP_Networking,    _param.ipaddr[3],   AP_NETWORKING_DEFAULT_IP_ADDR3),

    // @Param: _NETMASK
    // @DisplayName: IP Subnet mask
    // @Description: Allows setting static subnet mask. The value is a count of consecutive bits. Examples: 24 = 255.255.255.0, 16 = 255.255.0.0
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("_NETMASK", 5,  AP_Networking,    _param.netmask,   AP_NETWORKING_DEFAULT_NM_ADDR),

    // @Param: _DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Range: 0:Disable 1:Enable
    // @User: Advanced
    AP_GROUPINFO("_DHCP", 6,  AP_Networking,    _param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),

    // @Param: _GWADDR0
    // @DisplayName: Gateway IP Address MSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR0", 7,  AP_Networking,    _param.gwaddr[0],   AP_NETWORKING_DEFAULT_GW_ADDR0),

    // @Param: _GWADDR1
    // @DisplayName: Gateway IP Address 2nd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR1", 8,  AP_Networking,    _param.gwaddr[1],   AP_NETWORKING_DEFAULT_GW_ADDR1),

    // @Param: _GWADDR2
    // @DisplayName: Gateway IP Address 3rd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR2", 9,  AP_Networking,    _param.gwaddr[2],   AP_NETWORKING_DEFAULT_GW_ADDR2),

    // @Param: _GWADDR3
    // @DisplayName: Gateway IP Address LSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_GWADDR3", 10,  AP_Networking,    _param.gwaddr[3],   AP_NETWORKING_DEFAULT_GW_ADDR3),

    // @Param: _MACADDR0
    // @DisplayName: MAC Address MSbyte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR0", 11,  AP_Networking,    _param.macaddr[0],  AP_NETWORKING_DEFAULT_MAC_ADDR0),

    // @Param: _MACADDR1
    // @DisplayName: MAC Address 2nd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR1", 12,  AP_Networking,    _param.macaddr[1],   AP_NETWORKING_DEFAULT_MAC_ADDR1),

    // @Param: _MACADDR2
    // @DisplayName: MAC Address 3rd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR2", 13,  AP_Networking,    _param.macaddr[2],   AP_NETWORKING_DEFAULT_MAC_ADDR2),

    // @Param: _MACADDR3
    // @DisplayName: MAC Address 4th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR3", 14,  AP_Networking,    _param.macaddr[3],   AP_NETWORKING_DEFAULT_MAC_ADDR3),

    // @Param: _MACADDR4
    // @DisplayName: MAC Address 5th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR4", 15,  AP_Networking,    _param.macaddr[4],   AP_NETWORKING_DEFAULT_MAC_ADDR4),

    // @Param: _MACADDR5
    // @DisplayName: MAC Address LSb
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("_MACADDR5", 16,  AP_Networking,    _param.macaddr[5],   AP_NETWORKING_DEFAULT_MAC_ADDR5),

    // @Group: 1_
    // @Path: AP_TemperatureSensor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 30, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[0], "1_", 31, AP_Networking, backend_var_info[0]),

#if AP_NETWORKING_MAX_INSTANCES >= 2
    // @Group: 2_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 32, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[1], "2_", 33, AP_Networking, backend_var_info[1]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 3
    // @Group: 3_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 34, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[2], "3_", 35, AP_Networking, backend_var_info[2]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 4
    // @Group: 4_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 36, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[3], "4_", 37, AP_Networking, backend_var_info[3]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 5
    // @Group: 5_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 38, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[4], "5_", 39, AP_Networking, backend_var_info[4]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 6
    // @Group: 6_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 40, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[5], "6_", 41, AP_Networking, backend_var_info[5]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 7
    // @Group: 7_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 42, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[6], "7_", 43, AP_Networking, backend_var_info[6]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 8
    // @Group: 8_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 44, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[7], "8_", 45, AP_Networking, backend_var_info[7]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 9
    // @Group: 9_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 46, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[8], "9_", 47, AP_Networking, backend_var_info[8]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 10
    // @Group: A_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[9], "A_", 48, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[9], "A_", 49, AP_Networking, backend_var_info[9]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 11
    // @Group: B_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[10], "B_", 50, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[10], "B_", 51, AP_Networking, backend_var_info[10]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 12
    // @Group: C_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[11], "C", 52, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[11], "C_", 53, AP_Networking, backend_var_info[11]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 13
    // @Group: D_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[12], "D_", 54, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[12], "D_", 55, AP_Networking, backend_var_info[12]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 14
    // @Group: E_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[13], "E_", 56, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[13], "E_", 57, AP_Networking, backend_var_info[13]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 15
    // @Group: F_
    // @Path: AP_Networking_Params.cpp
    AP_SUBGROUPINFO(_params[14], "F_", 58, AP_Networking, AP_Networking_Params),
    AP_SUBGROUPVARPTR(_drivers[14], "F_", 59, AP_Networking, backend_var_info[14]),
#endif

#if AP_NETWORKING_MAX_INSTANCES >= 16
    #error "AP_NETWORKING_MAX_INSTANCES value is too large. Value must be <= 15."
#endif

    AP_GROUPEND
};
const AP_Param::GroupInfo *AP_Networking::backend_var_info[AP_NETWORKING_MAX_INSTANCES];

AP_Networking::AP_Networking(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Networking must be singleton");
    }
#endif

    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Networking::init()
{
    // If we skip this init, everything locks up. Sooo.. for
    // now we need to run init() regardless of enable param
    // if (!_param.enabled) {
    //     return;
    // }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    _activeSettings.ip = get_ip_param();
    _activeSettings.nm = get_netmask_param();
    _activeSettings.gw = get_gateway_param();
    net_addr_mode_t addrMode = NET_ADDRESS_STATIC;

    const uint8_t localMACAddress[6] = {(uint8_t)_param.macaddr[0].get(),
                                        (uint8_t)_param.macaddr[1].get(),
                                        (uint8_t)_param.macaddr[2].get(),
                                        (uint8_t)_param.macaddr[3].get(),
                                        (uint8_t)_param.macaddr[4].get(),
                                        (uint8_t)_param.macaddr[5].get() };

#if LWIP_DHCP
    if (get_dhcp_enabled()) {
        _activeSettings.ip = 0;
        _activeSettings.nm = 0;
        _activeSettings.gw = 0;
        addrMode = NET_ADDRESS_DHCP;
    }
#else
    set_dhcp_enable(false);
#endif

    struct lwipthread_opts netOptions = { (uint8_t *) localMACAddress,
                                        _activeSettings.ip,
                                        _activeSettings.nm,
                                        _activeSettings.gw,
                                        addrMode };

    lwipInit(&netOptions);

    apply_errata_for_mac_KSZ9896C();
#endif

    // create each instance
    for (uint8_t instance = 0; instance < AP_NETWORKING_MAX_INSTANCES; instance++) {
        switch (get_type(instance)) {
#if AP_NETWORKING_SERIAL2UDP_ENABLED
            case AP_Networking_Params::Type::Serial2UDP:
                _drivers[instance] = new AP_Networking_Serial2UDP(*this, _state[instance], _params[instance]);
                break;
#endif

            case AP_Networking_Params::Type::None:
            default:
                break;
        }

        // call init function for each backend
        if (_drivers[instance] != nullptr) {
            // if the backend has some local parameters then make those available in the tree
            if (_state[instance].var_info != nullptr) {
                backend_var_info[instance] = _state[instance].var_info;
                AP_Param::load_object_from_eeprom(_drivers[instance], backend_var_info[instance]);

                // param count could have changed
                AP_Param::invalidate_count();
            }
            _drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            _num_instances = instance + 1;
        }
    }

#if AP_NETWORKING_HAS_THREAD
    const uint32_t interval_ms = 1;
    _dev->register_periodic_callback(interval_ms * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Networking::thread, void));
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Initialized.");
    _init.done = true;
}

void AP_Networking::check_for_config_changes()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (_activeSettings.announce_ms == 0) {
        // do nothing on the first run so we print on update() running once already
        _activeSettings.announce_ms = now_ms;
        return;
    }
    if (now_ms - _activeSettings.announce_ms < 1000) {
        // Never announce changes any faster than 1 sec
        return;
    }

    const uint32_t ip = lwipGetIp();
    const uint32_t nm = lwipGetNetmask();
    const uint32_t gw = lwipGetGateway();

    if (_activeSettings.once &&
        ip == _activeSettings.ip &&
        nm == _activeSettings.nm &&
        gw == _activeSettings.gw)
    {
        // nothing changed and we're already printed at least once. Nothing to do.
        return;
    }

    _activeSettings.ip = ip;
    _activeSettings.nm = nm;
    _activeSettings.gw = gw;
    _activeSettings.once = true;
    _activeSettings.announce_ms = now_ms;


    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: IP      %s", get_ip_active_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Mask    %s", get_netmask_active_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Gateway %s", get_gateway_active_str());

    can_printf("NET: IP %s", get_ip_active_str());
    can_printf("NET: Mask %s", get_netmask_active_str());
    can_printf("NET: Gateway %s", get_gateway_active_str());
}


void AP_Networking::apply_errata_for_mac_KSZ9896C()
{
#if defined(BOARD_PHY_ID) && defined(MII_KSZ9896C_ID) && BOARD_PHY_ID == MII_KSZ9896C_ID
    /// Apply Erratas
    for (uint8_t phy = 0; phy < 32; phy++) {
        if (!(ETHD1.phyaddrmask & (1 << phy))) {
            continue;
        }

        // Hardware Design Checklist
        // https://ww1.microchip.com/downloads/en/DeviceDoc/KSZ989x-KSZ956x-KSZ9477-Hardware-Design-Checklist-00004151.pdf
        // 6.4: 10/100 Mbps Ethernet Only
        mii_write(&ETHD1, phy, 0x00, 0x3100); // disable 1000Gbps, enable auto-negotiate for 10/100
        mii_write(&ETHD1, phy, 0x09, 0x0400); // disable 1000Gbps announcements
        mii_write(&ETHD1, phy, 0x00, 0x3100 | (1 << 9)); // restart auto-negotiate

        // Erratas:
        // http://ww1.microchip.com/downloads/en/DeviceDoc/80000757A.pdf
        const uint16_t mmd[22][3] = {
            //[MMD], [register],[data]

            // module 1: Register settings are needed to improve PHY receive performance
            {0x01, 0x6F, 0xDD0B},
            {0x01, 0x8F, 0x6032},
            {0x01, 0x9D, 0x248C},
            {0x01, 0x75, 0x0060},
            {0x01, 0xD3, 0x7777},
            {0x1C, 0x06, 0x3008},
            {0x1C, 0x08, 0x2001},

            // module 2: Transmit waveform amplitude can be improved
            {0x1C, 0x04F, 0x00D0},

            // module 3: Energy Efficient Ethernet (EEE) feature select must be manually disabled
            {0x07, 0x03C, 0x0000},

            // module 4: Toggling PHY Powerdown can cause errors or link failures in adjacent PHYs
            #if STM32_MAC_ETH1_CHANGE_PHY_STATE
            #error "MII_KSZ9896C_ID Errata module 4 requires STM32_MAC_ETH1_CHANGE_PHY_STATE = FALSE"
            #endif

            // module 6: Register settings are required to meet data sheet supply current specifications
            {0x1C, 0x013, 0x6EFF},
            {0x1C, 0x014, 0xE6FF},
            {0x1C, 0x015, 0x6EFF},
            {0x1C, 0x016, 0xE6FF},
            {0x1C, 0x017, 0x00FF},
            {0x1C, 0x018, 0x43FF},
            {0x1C, 0x019, 0xC3FF},
            {0x1C, 0x01A, 0x6FFF},
            {0x1C, 0x01B, 0x07FF},
            {0x1C, 0x01C, 0x0FFF},
            {0x1C, 0x01D, 0xE7FF},
            {0x1C, 0x01E, 0xEFFF},
            {0x1C, 0x020, 0xEEEE},
        };

        
        for (uint8_t i=0; i<22; i++) {
            // Write MMD - Device Address 2h, Register 00h = 0010h to enable single-LED mode.
            // 1. Write the PHY MMD Setup Register with 0002h // Set up register address for MMD – Device Address 2h.
            // 2. Write the PHY MMD Data Register with 0000h // Select Register 00h of MMD – Device Address 2h.
            // 3. Write the PHY MMD Setup Register with 4002h // Select register data for MMD – Device Address 2h, Reg. 00h.
            // 4. Write the PHY MMD Data Register with 0010h // Write value 0010h to MMD – Device Address 2h, Reg. 00h.
            
            const uint16_t deviceAddress = (mmd[i][0] & 0x001F);
            mii_write(&ETHD1, phy, 0x0D, 0x0000 | deviceAddress);
            mii_write(&ETHD1, phy, 0x0E, mmd[i][1]);

            mii_write(&ETHD1, phy, 0x0D, 0x4000 | deviceAddress);
            mii_write(&ETHD1, phy, 0x0E, mmd[i][2]);
        }
    }
#endif
}


void AP_Networking::update()
{
    if (!_param.enabled) {
        return;
    }
    if (!_init.done) {
        return;
    }
    check_for_config_changes();

    for (uint8_t i=0; i<_num_instances; i++) {
        if (_drivers[i] != nullptr && get_type(i) != AP_Networking_Params::Type::None) {
            _drivers[i]->update();
        }
    }
}

AP_Networking_Params::Type AP_Networking::get_type(const uint8_t instance) const
{
    if (instance >= AP_NETWORKING_MAX_INSTANCES) {
        return AP_Networking_Params::Type::None;
    }
    return (AP_Networking_Params::Type)_params[instance].type.get();
}

uint32_t AP_Networking::convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount)
{
    if (netmask_bitcount > 32) {
        return 0;
    }

    uint32_t netmask_ip = 0;
    for (uint32_t i=0; i<netmask_bitcount; i++) {
        netmask_ip |= (1UL << i);
    }
    return netmask_ip;
}

uint8_t AP_Networking::convert_netmask_ip_to_bitcount(const uint32_t netmask_ip)
{
    uint32_t netmask_bitcount = 0;
    for (uint32_t i=0; i<32; i++) {
        // note, netmask LSB is IP MSB
        if ((netmask_ip & (1UL<<i)) == 0) {
            break;
        }
        netmask_bitcount++;
    }
    return netmask_bitcount;
}

uint32_t AP_Networking::convert_str_to_ip(char* ip_str)
{
    // struct sockaddr_in antelope[2];
    // inet_pton(AF_INET, "10.0.0.2", &(antelope[0].sin_addr));
    // return inet_addr(ip_str);

    uint32_t ip = 0;
    inet_pton(AF_INET, ip_str, &ip);
    return ip;
}

char* AP_Networking::convert_ip_to_str(const uint8_t ip[4])
{
    static char _str_buffer[20];
    if (hal.util->snprintf(_str_buffer, sizeof(_str_buffer), "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]) == 0) {
        _str_buffer[0] = '\0';
    }
    return _str_buffer;
}
char* AP_Networking::convert_ip_to_str(const uint32_t ip)
{
    uint8_t ip_array[4];
        ip_array[3] = ((ip >> 24) & 0xff);
        ip_array[2] = ((ip >> 16) & 0xff);
        ip_array[1] = ((ip >> 8) & 0xff);
        ip_array[0] = (ip & 0xff);

    return convert_ip_to_str(ip_array);
}

// periodic callback in our own networking thread at 1000 Hz
#if AP_NETWORKING_HAS_THREAD
void AP_Networking::thread()
{
    // TODO: do awesome stuff at 1kHz in our own dedicated thread!
}
#endif

// send a UDP packet and return with bytes sent or an error code
int32_t AP_Networking::send_udp(struct udp_pcb *pcb, const ip4_addr_t &ip4_addr, const uint16_t port, const uint8_t* data, uint16_t data_len)
{
    if (data == nullptr || pcb == nullptr) {
        return ERR_ARG;
    }
    if (data_len == 0) {
        return 0;
    }
    
    //data_len = MIN(data_len, AP_NETWORKING_MTU_SIZE);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_len , PBUF_RAM);
    if (p == nullptr) {
        return ERR_MEM;
    }

    ip_addr_t dst;
    ip_addr_copy_from_ip4(dst, ip4_addr);

    memcpy(p->payload, data, data_len);
    const err_t err = udp_sendto(pcb, p, &dst, port);
    pbuf_free(p);

    return err == ERR_OK ? data_len : err;
}

AP_Networking *AP_Networking::_singleton;
namespace AP { 
    AP_Networking &network() {
        return *AP_Networking::get_singleton();
    }
}

#endif // AP_NETWORKING_ENABLED
