
#pragma once

#include "AP_Networking_Backend.h"

#define AP_NETWORKING_SERIAL2UDP_ENABLED 1

#ifndef AP_NETWORKING_SERIAL2UDP_ENABLED
#define AP_NETWORKING_SERIAL2UDP_ENABLED AP_NETWORKING_ENABLED
#endif

#if AP_NETWORKING_SERIAL2UDP_ENABLED

//#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>


#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif

class AP_Networking_Serial2UDP  : public AP_Networking_Backend {

public:
    AP_Networking_Serial2UDP(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;

    static void serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

    static const struct AP_Param::GroupInfo var_info[];

private:

     AP_Int8 _foo;
    
    struct Eth_t{
        int16_t ip[4];  // change to AP_Int16
        int32_t port;  // change to AP_Int32
        ByteBuffer buf_in{AP_NETWORKING_MTU_SIZE};
        ByteBuffer buf_out{AP_NETWORKING_MTU_SIZE};

        //struct bbuf* p = pbuf_alloc(PBUF_TRANSPORT, sizeof(DataFrame), PBUF_RAM);
        // struct ip_addr destAddr;

        // DataFrame data;

        ip4_addr_t ip_addr;

        uint32_t last_send_ms;
    } _eth;
    static struct udp_pcb *_serial2udp_pcb;


    AP_HAL::UARTDriver *_uart;

    static uint8_t _serial2udp_instance_count;
    uint8_t _instance;
    uint32_t last_update_ms;


    int32_t send_udp(AP_Networking_Serial2UDP::Eth_t &eth, const uint8_t* data, uint32_t data_len);

};

#endif // AP_NETWORKING_SERIAL2UDP_ENABLED
