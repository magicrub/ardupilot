
#pragma once

#include "AP_Networking_Backend.h"

#ifndef AP_NETWORKING_SERIAL2UDP_ENABLED
#define AP_NETWORKING_SERIAL2UDP_ENABLED AP_NETWORKING_ENABLED && SERIALMANAGER_NUM_PORTS>=1
#endif

#if AP_NETWORKING_SERIAL2UDP_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>

#ifndef AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_BUFFERED
#define AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_BUFFERED   0
#endif


#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE 1000
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE 1000
#endif
#ifndef AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_TX_SIZE
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_TX_SIZE 1000
#endif
#if !defined(AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_RX_SIZE) && AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_BUFFERED
#define AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_RX_SIZE 1000
#endif

class AP_Networking_Serial2UDP  : public AP_Networking_Backend {
public:
    AP_Networking_Serial2UDP(AP_Networking &front,
                        AP_Networking::AP_Networking_State &state,
                        AP_Networking_Params &params);

    void init() override;

    void update() override;

    static const struct AP_Param::GroupInfo var_info[];

    static AP_Networking_Serial2UDP* get_Serial2UDP_backend(struct udp_pcb *pcb, const uint16_t port);

private:

    static void serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
    
        struct {
        AP_Int16 ip[4];
        AP_Int32 port;

#if AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_BUFFERED
        ByteBuffer buf_in{AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_RX_SIZE};
#endif
        ByteBuffer buf_out{AP_NETWORKING_SERIAL2UDP_BUFFER_UDP_TX_SIZE};
        HAL_Semaphore sem;

        struct udp_pcb *pcb;
        ip4_addr_t ip4_addr;

        uint32_t last_tx_ms;
        uint32_t last_rx_ms;
    } _eth;

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
