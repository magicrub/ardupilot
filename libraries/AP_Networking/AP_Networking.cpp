
#include "AP_Networking.h"

#if HAL_ENABLE_NETWORKING
#include <AP_HAL/AP_HAL.h>
#include <lwip/ip4_addr.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#ifndef AP_NETWORKING_DEFAULT_IP_ADDR0
#define AP_NETWORKING_DEFAULT_IP_ADDR0     192
#define AP_NETWORKING_DEFAULT_IP_ADDR1     168
#define AP_NETWORKING_DEFAULT_IP_ADDR2       1
#define AP_NETWORKING_DEFAULT_IP_ADDR3      10
#endif

#ifndef AP_NETWORKING_DEFAULT_GW_ADDR0
#define AP_NETWORKING_DEFAULT_GW_ADDR0     192
#define AP_NETWORKING_DEFAULT_GW_ADDR1     168
#define AP_NETWORKING_DEFAULT_GW_ADDR2       1
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


#define AP_NETWORKING_PHY_I2C_ADDR        (0xBE >> 1)
// 1011_1110 <write>
// 1011_1111 <read>

#define AP_NETWORKING_PHY_I2C_BUS         0

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
    AP_Param::setup_object_defaults(this, var_info);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Networking must be singleton");
    }
#endif
    _singleton = this;
}


void AP_Networking::init()
{

    // // set mac address
    // for (uint8_t i = 0; i < ARRAY_SIZE(_param.macaddr); i++) {
    //     macaddress[i] = _param.macaddr[i];
    // }
    // lwip_opts.macaddress = macaddress;

    // ip4_addr_t ipaddr, gw;
    // IP4_ADDR(&ipaddr, _param.ipaddr[0], _param.ipaddr[1], _param.ipaddr[2], _param.ipaddr[3]);
    // lwip_opts.address = ipaddr.addr;
    // // create a subnet mask based on the configured netmask
    // for (uint8_t i=0; i<_param.netmask; i++) {
    //     lwip_opts.netmask = lwip_opts.netmask | (0x10000000UL >> i);
    // }
    // //set gateway
    // IP4_ADDR(&gw, _param.gwaddr[0],_param.gwaddr[1], _param.gwaddr[2], _param.gwaddr[3]);
    // lwip_opts.gateway = gw.addr;

    // // set DHCP option
    // if (_param.dhcp) {
    //     lwip_opts.addrMode = NET_ADDRESS_DHCP;
    // } else {
    //     lwip_opts.addrMode = NET_ADDRESS_STATIC;
    // }

    // lwipInit(&lwip_opts);


    lwipInit(NULL);

    _dev = std::move(hal.i2c_mgr->get_device(0, AP_NETWORKING_PHY_I2C_ADDR));

    if (!_dev) {
        _init.failed = true;
        return;
    }

    hal.gpio->write(66, 0);

    const uint32_t interval_ms = 100;
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->register_periodic_callback(interval_ms * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Networking::thread, void));
}

void AP_Networking::reset()
{
#ifdef HAL_GPIO_PIN_GPIO_ETH_RESET_N

    const uint32_t now_ms = AP_HAL::millis();

    switch (_init.reset_state) {
        case 0:
            // reset
            palWriteLine(HAL_GPIO_PIN_GPIO_ETH_RESET_N, 0);
            _init.timer_ms = now_ms;
            _init.reset_state++;
            return;

        case 1:
            // hold in reset
            if (now_ms - _init.timer_ms >= 200) {
                // resume
                palWriteLine(HAL_GPIO_PIN_GPIO_ETH_RESET_N, 1);
                _init.timer_ms = now_ms;
                _init.reset_state++;
            }
            return;

        case 2:
            // wait for boot-up delay
            if (now_ms - _init.timer_ms >= 200) {
                // carry on...
                _init.reset_state++;
            }
            return;
      }
#endif
    _init.reset_done = true;
}


void AP_Networking::configure()
{
    if (!_init.reset_done) {
        reset();
        return;
    }


//Write to the following MMD registers for each PHY port [1-5]:
// [MMD] [register] [data]
// 0x01 0x6F 0xDD0B
// 0x01 0x8F 0x6032
// 0x01 0x9D 0x248C
// 0x01 0x75 0x0060
// 0x01 0xD3 0x7777
// 0x1C 0x06 0x3008
// 0x1C 0x08 0x2001
    //write_register8(6,3,0x01, temp[0]);

    uint16_t value = 0;
    (void)value;

    //value = read_register16(0, 1, 0x00);


    // 0x0104 - 0x0107  // IBA Enable
    // 0x0210 - 0x0213  // LED Configuration Strap Register


    // 0x0300           // Switch Operation Register
    write_register8(0,3,0x00, 0x01);

    // 0xN100 - 0xN101  // PHY Basic Control Register
    for (uint8_t N=1; N<=AP_NETWORKING_PHY_COUNT; N++) {
        write_register16(N,3,0x00, 0x3100);
    }

    _init.is_configured = true;
}

void AP_Networking::update()
{
    if (_init.failed) {
        return;
    }

}

uint16_t AP_Networking::convert_to_addr16(const uint8_t port, const uint8_t function, const uint8_t reg)
{
    uint16_t addr = 0;
    addr |= (uint16_t(port) << 12);
    addr |= (uint16_t(function) << 8);
    addr |= ((uint16_t(reg)) & 0xFF);
    return htobe16(addr);
}

bool AP_Networking::write_register16(const uint8_t port, const uint8_t function, const uint8_t reg, const uint16_t data)
{
    if (!_dev) {
        return false;
    }

    const uint16_t addr = convert_to_addr16(port, function, reg);
    const uint8_t payload[4] = {
       uint8_t(addr & 0xFF),
       uint8_t(addr >> 8),
       uint8_t(data & 0xFF),
       uint8_t(data >> 8) };

    return _dev->transfer(payload, sizeof(payload), nullptr, 0);
}

bool AP_Networking::write_register8(const uint8_t port, const uint8_t function, const uint8_t reg, const uint8_t data)
{
    if (!_dev) {
        return false;
    }

    const uint16_t addr = convert_to_addr16(port, function, reg);
    const uint8_t payload[3] = {
       uint8_t(addr & 0xFF),
       uint8_t(addr >> 8),
       data };

    return _dev->transfer(payload, sizeof(payload), nullptr, 0);
}

uint16_t AP_Networking::read_register16(const uint8_t port, const uint8_t function, const uint8_t reg)
{
    if (!_dev) {
        return 0;
    }
    const uint16_t addr = convert_to_addr16(port, function, reg);
    uint16_t rx_data = 0;
    _dev->transfer((uint8_t *)&addr, sizeof(addr), (uint8_t *)&rx_data, 2);
    return htobe16(rx_data);
}


uint8_t AP_Networking::read_register8(const uint8_t port, const uint8_t function, const uint8_t reg)
{
    if (!_dev) {
        return 0;
    }
    const uint16_t addr = convert_to_addr16(port, function, reg);
    uint8_t rx_data = 0;
    _dev->transfer((uint8_t *)&addr, sizeof(addr), (uint8_t *)&rx_data, 1);
    return rx_data;
}

void AP_Networking::thread()
{
    uint16_t temp[100];
    uint16_t temp16[100];
    (void)temp;
    (void)temp16;

    if (!_init.is_configured) {
        configure();
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

#ifdef BOARD_PHY_USE_CACHED_LINK_STATUS
    for (uint8_t i=0; i<AP_NETWORKING_PHY_COUNT; i++) {
        const uint16_t phy = i+1;
#if defined(AP_NETWORKING_PHY_PRIMARY) && AP_NETWORKING_PHY_PRIMARY > 0
        if (AP_NETWORKING_PHY_PRIMARY != phy) {
          continue;
        }
#endif

        if (now_ms - _link_stats[i].timer_ms < 500) {
          continue;
        }
        _link_stats[i].timer_ms = now_ms;
        _link_stats[i].registers.bmcr = read_register16(phy, 1, 0x00);
        _link_stats[i].registers.bmsr = read_register16(phy, 1, 0x02);
        _link_stats[i].registers.lpa  = read_register16(phy, 1, 0x0A);

        mac_lld_set_cached_link_status_registers(
              _link_stats[i].registers.bmsr,
              _link_stats[i].registers.bmcr,
              _link_stats[i].registers.lpa);
        break; // only do one of these at a time so we don't spam the I2C bus too hard all at once
    }
#endif

    if (!_debug.timer_ms) {
        printf("-----------------------\r\n");
        printf("Read:  0x%1X%1X%02X\tValue: 0x%04X\r\n", 1, 1, 0x04, read_register16(1, 1, 0x04));
        printf("Read:  0x%1X%1X%02X\tValue: 0x%04X\r\n", 1, 1, 0x06, read_register16(1, 1, 0x06));
        _debug.timer_ms = now_ms;
    }

    if (now_ms - _debug.timer_ms < 200) {
      return;
    }
    _debug.timer_ms = now_ms;

    switch (_debug.state++) {
      case 0 ... 3:
      case 5 ... 7:
        printf("Read:  0x%1X%1X%02X\tValue: 0x%02X\r\n", 6, 3, 0x01, read_register8(6, 3, 0x01));
        break;

      case 4:
        temp[0] = 0x49; // set to RMII
        hal.gpio->write(66, 1);
        temp[1] = write_register8(6,3,0x01, temp[0]);
        hal.gpio->write(66, 0);

        printf("Write: 0x%1X%1X%02X\tValue: 0x%02X success=%d\r\n", 6, 3, 0x01, temp[0], temp[1]);

        //write_register8(6,3,0, 0b01111000); // enable TX&RX flow control
      break;

    default:
      for (uint8_t i=1; i<=AP_NETWORKING_PHY_COUNT; i++) {
          temp16[i] = read_register16(i, 1, 0x26);
      }
      printf("Link Status: \t%4X\t%4X\t%4X\t%4X\t%4X\r\n", temp16[1], temp16[2], temp16[3], temp16[4], temp16[5]);
      break;
    }
}

AP_Networking *AP_Networking::_singleton;
namespace AP { 
    AP_Networking *network() {
        return AP_Networking::get_singleton();
    }
}

#endif // HAL_ENABLE_NETWORKING
