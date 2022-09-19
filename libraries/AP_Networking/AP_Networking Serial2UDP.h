
#pragma once

#include <AP_Networking/AP_Networking.h>

#ifndef AP_NETWORKING_SERIAL2UDP_ENABLED
#define AP_NETWORKING_SERIAL2UDP_ENABLED AP_NETWORKING_ENABLED
#endif

#if AP_NETWORKING_SERIAL2UDP_ENABLED

#include <AP_Param/AP_Param.h>
//#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>


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

class AP_Networking_Serial2UDP {
public:
    AP_Networking_Serial2UDP();

    /* Do not allow copies */
    AP_Networking_Serial2UDP(const AP_Networking_Serial2UDP &other) = delete;
    AP_Networking_Serial2UDP &operator=(const AP_Networking_Serial2UDP&) = delete;

    virtual void init();

    virtual void update();

    static const struct AP_Param::GroupInfo        var_info[];

    char* serial2udp_get_ip(const uint32_t stream_id);
    uint16_t serial2udp_get_port(const uint32_t stream_id);
    uint32_t serial2udp_get_udp_outbound_buffer(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max);
    uint32_t serial2udp_load_udp_inbound_buffer(const uint32_t stream_id, uint8_t* data, const uint32_t data_len_max);
    static void serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

private:

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

    bool send_udp(Serial2UDP_t::Serial2UDP_Eth_t &eth, const uint8_t* data, const uint16_t data_len);

};

#endif // AP_NETWORKING_SERIAL2UDP_ENABLED
