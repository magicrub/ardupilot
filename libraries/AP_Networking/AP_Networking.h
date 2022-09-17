
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_NETWORKING_ENABLED
#define AP_NETWORKING_ENABLED 0
#endif

#if AP_NETWORKING_ENABLED

#include <AP_Param/AP_Param.h>
//#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    #include "lwipthread.h"
    #include "lwip/udp.h"
#else
    #if BYTE_ORDER == LITTLE_ENDIAN
    #define IP4_ADDR_VALUE(a,b,c,d)        \
            (((uint32_t)((d) & 0xff) << 24) | \
            ((uint32_t)((c) & 0xff) << 16) | \
            ((uint32_t)((b) & 0xff) << 8)  | \
            (uint32_t)((a) & 0xff))
    #else
    #define IP4_ADDR_VALUE(a,b,c,d)        \
            (((uint32_t)((a) & 0xff) << 24) | \
            ((uint32_t)((b) & 0xff) << 16) | \
            ((uint32_t)((c) & 0xff) << 8)  | \
            (uint32_t)((d) & 0xff))
    #endif
#endif

#define IP4_ADDR_VALUE_FROM_ARRAY(array) IP4_ADDR_VALUE(array[0],array[1],array[2],array[3])

#ifndef AP_NETWORKING_HAS_THREAD
#define AP_NETWORKING_HAS_THREAD 0
#endif

#ifndef AP_NETWORKING_MTU_SIZE
#define AP_NETWORKING_MTU_SIZE 1500
#endif

#ifndef AP_NETWORKING_SERIAL2UDP_ENABLED
#define AP_NETWORKING_SERIAL2UDP_ENABLED 0
#endif

#if AP_NETWORKING_SERIAL2UDP_ENABLED
#ifndef AP_NETWORKING_SERIAL2UDP_INSTANCE_MAX
#define AP_NETWORKING_SERIAL2UDP_INSTANCE_MAX 1
#endif

#if AP_NETWORKING_SERIAL2UDP_INSTANCE_MAX <= 0
#error "AP_NETWORKING_SERIAL2UDP_INSTANCE_MAX is too small"
#endif

#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_RX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_RX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_TX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif

#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif
#endif // AP_NETWORKING_SERIAL2UDP_ENABLED

class AP_Networking {
public:
    AP_Networking();

    /* Do not allow copies */
    AP_Networking(const AP_Networking &other) = delete;
    AP_Networking &operator=(const AP_Networking&) = delete;

    void init();

    void update();

    static AP_Networking *get_singleton(void) {return _singleton; }

    static const struct AP_Param::GroupInfo        var_info[];

    bool is_healthy() const { return _init.done; }    // TODO: is_healthy()
    bool get_dhcp_enabled() const { return _param.dhcp; }
    void set_dhcp_enable(const bool enable) { _param.dhcp = enable; }

    // TODO: implement all _active() helpers
    uint32_t get_ip_active() const { return _activeSettings.ip; }
    uint32_t get_ip_param() const { return IP4_ADDR_VALUE_FROM_ARRAY(_param.ipaddr); }
    char*    get_ip_active_str() { return convert_ip_to_str(get_ip_active()); }
    char*    get_ip_param_str() { return convert_ip_to_str(get_ip_param()); }
    void     set_ip_param_str(const char* ip_str) { set_ip_param(convert_str_to_ip((char*)ip_str)); }
    void     set_ip_param(const uint32_t ip) {
                //put_le32_ptr(_param.ipaddr->get(), ip);
                _param.ipaddr[3].set_and_save((ip >> 24) & 0xff);
                _param.ipaddr[2].set_and_save((ip >> 16) & 0xff);
                _param.ipaddr[1].set_and_save((ip >> 8) & 0xff);
                _param.ipaddr[0].set_and_save(ip & 0xff);
            }

    uint32_t get_netmask_active() const { return _activeSettings.nm; }
    uint32_t get_netmask_param() const { return convert_netmask_bitcount_to_ip(_param.netmask.get()); }
    char*    get_netmask_active_str() { return convert_ip_to_str(get_netmask_active()); }
    char*    get_netmask_param_str() { return convert_ip_to_str(get_netmask_param()); }
    void     set_netmask_param_str(const char* nm_str) { set_netmask_param(convert_str_to_ip((char*)nm_str)); }
    void     set_netmask_param(const uint32_t nm) { _param.netmask = convert_netmask_ip_to_bitcount(nm); }

    uint32_t get_gateway_active() const { return _activeSettings.gw; }
    uint32_t get_gateway_param() const { return IP4_ADDR_VALUE_FROM_ARRAY(_param.gwaddr); }
    char*    get_gateway_active_str() { return convert_ip_to_str(get_gateway_active()); }
    char*    get_gateway_param_str() { return convert_ip_to_str(get_gateway_param()); }
    void     set_gateway_param_str(const char* gw_str) { set_gateway_param(convert_str_to_ip((char*)gw_str)); }
    void     set_gateway_param(const uint32_t gw) {
                //put_le32_ptr(_param.gwaddr->get(), gw);
                _param.gwaddr[3].set_and_save((gw >> 24) & 0xff);
                _param.gwaddr[2].set_and_save((gw >> 16) & 0xff);
                _param.gwaddr[1].set_and_save((gw >> 8) & 0xff);
                _param.gwaddr[0].set_and_save(gw & 0xff);
            }


    static uint32_t convert_str_to_ip(char* ip_str);
    static char* convert_ip_to_str(const uint8_t ip[4]);
    static char* convert_ip_to_str(const uint32_t ip);

    static uint32_t convert_netmask_bitcount_to_ip(const uint32_t netmask_bitcount);
    static uint8_t convert_netmask_ip_to_bitcount(const uint32_t netmask_ip);


#if AP_NETWORKING_SERIAL2UDP_ENABLED
    char* serial2udp_get_ip(const uint32_t stream_id);
    uint16_t serial2udp_get_port(const uint32_t stream_id);
    uint32_t serial2udp_get_udp_outbound_buffer(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max);
    uint32_t serial2udp_load_udp_inbound_buffer(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max);
    static void serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
#endif

private:
    static AP_Networking *_singleton;

#if AP_NETWORKING_HAS_THREAD
    void thread();
#endif
    
    void apply_errata_for_mac_KSZ9896C();
    void check_for_config_changes();

#if AP_NETWORKING_SERIAL2UDP_ENABLED
    struct Serial2UDP_t {
        struct Serial2UDP_Eth_t{
            int16_t ip[4];  // change to AP_Int16
            int32_t port;  // change to AP_Int32
            ByteBuffer buf_in{0};
            ByteBuffer buf_out{0};

            //struct bbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(DataFrame), PBUF_RAM);
            // struct ip_addr destAddr;
 
            // DataFrame data;

            ip_addr_t ip_addr;
            struct udp_pcb* pcb;
        } eth;
        AP_HAL::UARTDriver *uart;
    } _serial2udp[AP_NETWORKING_SERIAL2UDP_INSTANCE_MAX];
    uint16_t _serial2udp_count;

    void serial2udp_init();
    void serial2udp_update();
    bool send_udp(Serial2UDP_t::Serial2UDP_Eth_t &eth, const uint8_t* data, const uint16_t data_len);
#endif

    struct {
        bool done;
    } _init;

    struct {
        AP_Int16 ipaddr[4];
        AP_Int8 netmask;    // bits to mask. example: (16 == 255.255.0.0) and (24 == 255.255.255.0)
        AP_Int16 gwaddr[4];

        AP_Int8 dhcp;
        AP_Int16 macaddr[6];
        AP_Int8 enabled;
    } _param;

    struct {
        uint32_t ip;
        uint32_t nm;
        uint32_t gw;
        uint32_t announce_ms;
        bool once;
    } _activeSettings;


    HAL_Semaphore _sem;
};

namespace AP {
    AP_Networking *network();
};

#endif // AP_NETWORKING_ENABLED
