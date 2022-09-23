
#pragma once

#include "AP_Networking_Backend.h"

#define AP_NETWORKING_SERIAL2UDP_ENABLED 1

#ifndef AP_NETWORKING_SERIAL2UDP_ENABLED
#define AP_NETWORKING_SERIAL2UDP_ENABLED AP_NETWORKING_ENABLED && SERIALMANAGER_NUM_PORTS>=1
#endif

#if AP_NETWORKING_SERIAL2UDP_ENABLED

//#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>


#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE 2*AP_NETWORKING_MTU_SIZE
#endif

#define AP_NETWORKING_SERIAL2UDP_UDP_MAX_PACKET_SIZE 950

#ifndef AP_Networking_Serial2UDP_DELAY_MS
#define AP_Networking_Serial2UDP_DELAY_MS 10
#endif

#ifndef AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES
#define AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES 4
#endif

#if SERIALMANAGER_NUM_PORTS < AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES
// limit UDP_serial tunnel count to physical serial port count
#undef AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES
#define AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES SERIALMANAGER_NUM_PORTS
#endif


#define AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_UNBUFFERED 1


class AP_Networking_Serial2UDP  : public AP_Networking_Backend {

public:
    AP_Networking_Serial2UDP(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;


    static const struct AP_Param::GroupInfo var_info[];

    struct Eth_t {
        int16_t ip[4];  // change to AP_Int16[4]
        int32_t port;  // change to AP_Int32

#if !AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_UNBUFFERED
        ByteBuffer buf_in{1000};
#endif
        ByteBuffer buf_out{1000};
        HAL_Semaphore sem;

        struct udp_pcb *pcb;
        ip4_addr_t ip4_addr;

        uint32_t last_tx_ms;
        uint32_t last_rx_ms;
    };

    static AP_Networking_Serial2UDP* get_Serial2UDP_backend(struct udp_pcb *pcb, const uint16_t port);

private:

    static void serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

    AP_Int8 _foo;
    
    Eth_t _eth;

    struct Uart_t {
        AP_HAL::UARTDriver *uart;
        uint32_t last_rx_ms;
        uint32_t last_tx_ms;
    } _serial;

    int32_t serial2udp_send_buf_out();

    static uint8_t _serial2udp_instance_count;
    uint8_t _instance;
};

#endif // AP_NETWORKING_SERIAL2UDP_ENABLED
