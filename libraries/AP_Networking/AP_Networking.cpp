
#include "AP_Networking.h"

#if HAL_ENABLE_NETWORKING
#include <AP_HAL/AP_HAL.h>
#include <lwip/ip4_addr.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <lwip/ip4_addr.h>
#include <hal_mii.h>
#include <GCS_MAVLink/GCS.h>


#ifndef AP_NETWORKING_DEFAULT_IP_ADDR0
#define AP_NETWORKING_DEFAULT_IP_ADDR0     192
#define AP_NETWORKING_DEFAULT_IP_ADDR1     168
#define AP_NETWORKING_DEFAULT_IP_ADDR2       0
#define AP_NETWORKING_DEFAULT_IP_ADDR3     193
#endif

#ifndef AP_NETWORKING_DEFAULT_GW_ADDR0
#define AP_NETWORKING_DEFAULT_GW_ADDR0     192
#define AP_NETWORKING_DEFAULT_GW_ADDR1     168
#define AP_NETWORKING_DEFAULT_GW_ADDR2       0
#define AP_NETWORKING_DEFAULT_GW_ADDR3       1
#endif

#ifndef AP_NETWORKING_DEFAULT_NM_ADDR
#define AP_NETWORKING_DEFAULT_NM_ADDR     24
#endif

#ifndef AP_NETWORKING_DEFAULT_DHCP_ENABLE
#define AP_NETWORKING_DEFAULT_DHCP_ENABLE 1
#endif

#ifndef AP_NETWORKING_DEFAULT_MAC_ADDR0
#define AP_NETWORKING_DEFAULT_MAC_ADDR0   LWIP_ETHADDR_0
#define AP_NETWORKING_DEFAULT_MAC_ADDR1   LWIP_ETHADDR_1
#define AP_NETWORKING_DEFAULT_MAC_ADDR2   LWIP_ETHADDR_2
#define AP_NETWORKING_DEFAULT_MAC_ADDR3   LWIP_ETHADDR_3
#define AP_NETWORKING_DEFAULT_MAC_ADDR4   LWIP_ETHADDR_4
#define AP_NETWORKING_DEFAULT_MAC_ADDR5   LWIP_ETHADDR_5
#endif

const AP_Param::GroupInfo AP_Networking::var_info[] = {
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
//     // initialise LWIP

    const uint8_t localMACAddress[6] = {(uint8_t)_param.macaddr[0].get(),
                                        (uint8_t)_param.macaddr[1].get(),
                                        (uint8_t)_param.macaddr[2].get(),
                                        (uint8_t)_param.macaddr[3].get(),
                                        (uint8_t)_param.macaddr[4].get(),
                                        (uint8_t)_param.macaddr[5].get() };

    uint32_t netmask = 0;
    uint32_t ip, gateway;
    net_addr_mode_t addrMode;

#if LWIP_DHCP
    if (_param.dhcp) {
        ip = 0;
        gateway = 0;
        addrMode = NET_ADDRESS_DHCP;
    } else
#endif    
    {
        for (uint32_t i=0; i<_param.netmask; i++) {
            netmask |= (1UL << i);
        }
        ip = IP4_ADDR_VALUE(_param.ipaddr[0], _param.ipaddr[1],_param.ipaddr[2], _param.ipaddr[3]),
        gateway = IP4_ADDR_VALUE(_param.gwaddr[0], _param.gwaddr[1], _param.gwaddr[2], _param.gwaddr[3]),
        addrMode = NET_ADDRESS_STATIC;
    }

    struct lwipthread_opts netOptions = { (uint8_t *) localMACAddress,
                                        ip,
                                        netmask,
                                        gateway,
                                        addrMode };

    lwipInit(&netOptions);

    apply_errata_for_mac_KSZ9896C();


#if AP_NETWORKING_HAS_THREAD
    const uint32_t interval_ms = 1;
    _dev->register_periodic_callback(interval_ms * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Networking::thread, void));
#endif

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
        mii_write(&ETHD1, phy, 0x00, mii_read(&ETHD1, phy, 0x00) & ~0x0020); // clear bit 6
        mii_write(&ETHD1, phy, 0x12, 0x0800);
        mii_write(&ETHD1, phy, 0x00, 0x3100 | (1 << 9));


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
    if (!_init.done) {
        return;
    }

    // TODO: add awesome stuff!
}


// periodic callback in our own networking thread at 1Hz for 
#if AP_NETWORKING_HAS_THREAD
void AP_Networking::thread()
{

}
#endif

AP_Networking *AP_Networking::_singleton;
namespace AP { 
    AP_Networking *network() {
        return AP_Networking::get_singleton();
    }
}

#endif // HAL_ENABLE_NETWORKING
