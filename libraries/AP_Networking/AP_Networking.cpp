
#include "AP_Networking.h"

#if HAL_ENABLE_NETWORKING

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    #include <hal_mii.h>
    #include <AP_Scripting/luasocket/src/inet.h>

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

const AP_Param::GroupInfo AP_Networking::var_info[] = {

    AP_GROUPINFO_FLAGS("ENABLED"  ,  0, AP_Networking, _param.enabled, 0, AP_PARAM_FLAG_ENABLE),

      // @Group: IPADDR0
    // @DisplayName: IP Address MSB
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IPADDR0", 1,  AP_Networking,    _param.ipaddr[0],   AP_NETWORKING_DEFAULT_IP_ADDR0),

    // @Group: IPADDR1
    // @DisplayName: IP Address 2nd byte
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IPADDR1", 2,  AP_Networking,    _param.ipaddr[1],   AP_NETWORKING_DEFAULT_IP_ADDR1),

    // @Group: IPADDR2
    // @DisplayName: IP Address 3rd byte
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IPADDR2", 3,  AP_Networking,    _param.ipaddr[2],   AP_NETWORKING_DEFAULT_IP_ADDR2),

    // @Group: IPADDR3
    // @DisplayName: IP Address LSB
    // @Description: Allows setting static IP address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("IPADDR3", 4,  AP_Networking,    _param.ipaddr[3],   AP_NETWORKING_DEFAULT_IP_ADDR3),

    // @Group: NETMASK
    // @DisplayName: IP Subnet mask
    // @Description: Allows setting static subnet mask
    // @Range: 0 32
    // @User: Advanced
    AP_GROUPINFO("NETMASK", 5,  AP_Networking,    _param.netmask,   AP_NETWORKING_DEFAULT_NM_ADDR),

    // @Group: DHCP
    // @DisplayName: DHCP client
    // @Description: Enable/Disable DHCP client
    // @Range: 0:Disable 1:Enable
    // @User: Advanced
    AP_GROUPINFO("DHCP", 6,  AP_Networking,    _param.dhcp,   AP_NETWORKING_DEFAULT_DHCP_ENABLE),

    // @Group: GWADDR0
    // @DisplayName: Gateway IP Address MSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("GWADDR0", 7,  AP_Networking,    _param.gwaddr[0],   AP_NETWORKING_DEFAULT_GW_ADDR0),

    // @Group: GWADDR1
    // @DisplayName: Gateway IP Address 2nd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("GWADDR1", 8,  AP_Networking,    _param.gwaddr[1],   AP_NETWORKING_DEFAULT_GW_ADDR1),

    // @Group: GWADDR2
    // @DisplayName: Gateway IP Address 3rd byte
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("GWADDR2", 9,  AP_Networking,    _param.gwaddr[2],   AP_NETWORKING_DEFAULT_GW_ADDR2),

    // @Group: GWADDR3
    // @DisplayName: Gateway IP Address LSB
    // @Description: Allows setting static GW address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("GWADDR3", 10,  AP_Networking,    _param.gwaddr[3],   AP_NETWORKING_DEFAULT_GW_ADDR3),

    // @Group: LWIP_MACADDR0
    // @DisplayName: MAC Address MSbyte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR0", 11,  AP_Networking,    _param.macaddr[0],  AP_NETWORKING_DEFAULT_MAC_ADDR0),

    // @Group: MACADDR1
    // @DisplayName: MAC Address 2nd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR1", 12,  AP_Networking,    _param.macaddr[1],   AP_NETWORKING_DEFAULT_MAC_ADDR1),

    // @Group: MACADDR2
    // @DisplayName: MAC Address 3rd byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR2", 13,  AP_Networking,    _param.macaddr[2],   AP_NETWORKING_DEFAULT_MAC_ADDR2),

    // @Group: MACADDR3
    // @DisplayName: MAC Address 4th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR3", 14,  AP_Networking,    _param.macaddr[3],   AP_NETWORKING_DEFAULT_MAC_ADDR3),

    // @Group: MACADDR4
    // @DisplayName: MAC Address 5th byte
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR4", 15,  AP_Networking,    _param.macaddr[4],   AP_NETWORKING_DEFAULT_MAC_ADDR4),

    // @Group: MACADDR5
    // @DisplayName: MAC Address LSb
    // @Description: Allows setting MAC address
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MACADDR5", 16,  AP_Networking,    _param.macaddr[5],   AP_NETWORKING_DEFAULT_MAC_ADDR5),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

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
    uint32_t ip = get_ip_param();
    uint32_t netmask = get_netmask_param();
    uint32_t gateway = get_gateway_param();
    net_addr_mode_t addrMode = NET_ADDRESS_STATIC;

    const uint8_t localMACAddress[6] = {(uint8_t)_param.macaddr[0].get(),
                                        (uint8_t)_param.macaddr[1].get(),
                                        (uint8_t)_param.macaddr[2].get(),
                                        (uint8_t)_param.macaddr[3].get(),
                                        (uint8_t)_param.macaddr[4].get(),
                                        (uint8_t)_param.macaddr[5].get() };

#if LWIP_DHCP
    if (get_dhcp_enabled()) {
        ip = 0;
        gateway = 0;
        netmask = 0;
        addrMode = NET_ADDRESS_DHCP;
    }
#else
    set_dhcp_enable(false);
#endif

    struct lwipthread_opts netOptions = { (uint8_t *) localMACAddress,
                                        ip,
                                        netmask,
                                        gateway,
                                        addrMode };

    lwipInit(&netOptions);

    apply_errata_for_mac_KSZ9896C();
#endif


#if AP_NETWORKING_HAS_THREAD
    const uint32_t interval_ms = 1;
    _dev->register_periodic_callback(interval_ms * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Networking::thread, void));
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Initialized.");
#if LWIP_DHCP
    if (get_dhcp_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Waiting for DHCP IP assignment...");
    } else
#endif
    {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: IP      %s", get_ip_active_str());
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Mask    %s", get_netmask_active_str());
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"NET: Gateway %s", get_gateway_active_str());

        can_printf("NET: IP      %s", get_ip_active_str());
    }

    _init.done = true;
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

    // TODO: add awesome stuff!
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
        ip_array[0] = ((ip >> 24) & 0xff);
        ip_array[1] = ((ip >> 16) & 0xff);
        ip_array[2] = ((ip >> 8) & 0xff);
        ip_array[3] = (ip & 0xff);

    return convert_ip_to_str(ip_array);
}

// periodic callback in our own networking thread at 1000 Hz
#if AP_NETWORKING_HAS_THREAD
void AP_Networking::thread()
{
    // TODO: do awesome stuff at 1kHz in our own dedicated thread!
}
#endif

AP_Networking *AP_Networking::_singleton;
namespace AP { 
    AP_Networking *network() {
        return AP_Networking::get_singleton();
    }
}

#endif // HAL_ENABLE_NETWORKING
