
#include "AP_Networking_Serial2UDP.h"

#if AP_NETWORKING_SERIAL2UDP_ENABLED
#include <AP_Math/AP_Math.h> // for MIN()

struct udp_pcb *AP_Networking_Serial2UDP::_serial2udp_pcb;
uint8_t AP_Networking_Serial2UDP::_serial2udp_instance_count = 0;

const AP_Param::GroupInfo AP_Networking_Serial2UDP::var_info[] = {

    // @Param: FOO
    // @DisplayName: FOO
    // @Description: FOO
    // @Values: FOO
    // @User: Standard
    AP_GROUPINFO("FOO", 1, AP_Networking_Serial2UDP, _foo, 0),

  
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

/// Constructor
AP_Networking_Serial2UDP::AP_Networking_Serial2UDP(AP_Networking &front, AP_Networking::AP_Networking_State &state,  AP_Networking_Params &params) :
    AP_Networking_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}


void AP_Networking_Serial2UDP::init()
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Network_Serial2UDP, 0);
    if (_uart == nullptr) {
        printf("\n\nSerial2UDP init failed, could not find serial port\r\n");
        return;
    }

    _instance = _serial2udp_instance_count;
    _serial2udp_instance_count++;

    const uint32_t baudrate = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Network_Serial2UDP, _instance);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->begin(baudrate, AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE, AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE);

    // _eth.buf_in.set_size(AP_NETWORKING_MTU_SIZE);
    // _eth.buf_out.set_size(AP_NETWORKING_MTU_SIZE);


    _eth.port = 1313;
    //IP4_ADDR(_eth->ip_addr, 255, 255, 255, 255);
    //_eth.u_addr.ip4.addr = IPADDR_BROADCAST;

    IP4_ADDR(&_eth.ip_addr, 255, 255, 255, 255);

    if (_serial2udp_pcb == NULL) {
        _serial2udp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_serial2udp_pcb != NULL) {

            // // set up RX callback
            // udp_recv(_eth.pcb, AP_Networking_Serial2UDP::serial2udp_recv_callback, _eth.pcb);

            // // allow to sending broadcast packets
            ip_set_option(_serial2udp_pcb, SOF_BROADCAST);
            // udp_bind(_eth.pcb, IP_ANY_TYPE, _eth.port);


            // // listen only
            // ip_set_option(_eth.pcb, SOF_BROADCAST);
            // udp_bind(_eth.pcb, IP_ANY_TYPE, _eth.port);
        }
    }
}

void AP_Networking_Serial2UDP::serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    printf("serial2udp_recv_callback\r\n");
}


// #pragma GCC diagnostic push
// #pragma GCC diagnostic error "-Wframe-larger-than=1800"
void AP_Networking_Serial2UDP::update()
{
    if (_uart == nullptr || _serial2udp_pcb == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

     
    if (now_ms - last_update_ms < 100) {
        return;
    }
    last_update_ms = now_ms;


    uint8_t buf[100] {'x'};
    const int32_t result = send_udp(_eth, buf, sizeof(buf));
    printf("%d send_udp() result %d\r\n", (int)now_ms, (int)result);



    // if (_eth.buf_out.available() < 100) {
    //     uint8_t buf[100];
    //     memset(&buf, 'x', sizeof(buf));
    //     _eth.buf_out.write(buf, sizeof(buf));
    // }
    
    // // process eth.buf_out -> UDP
    // uint32_t while_udp_out_loop_count = 0;
    // while(while_udp_out_loop_count++ < 10)
    // {
    //     if (_eth.buf_out.available() == 0) {
    //         // nothing to send
    //         break;
    //     }

    //     if ((_eth.buf_out.space() > 0) && (now_ms - _eth.last_send_ms < 10)) {
    //         // we have stuff to send but there' still room to queuing
    //         // it up into a single packet before the timer expires
    //         break;
    //     }

    //     // send it!
    //     uint32_t navail;
    //     int32_t result = send_udp(_eth, _eth.buf_out.readptr(navail), navail);
    //     if (result > 0) {
    //         // sent!
    //         _eth.last_send_ms = now_ms;
    //         if (result != navail) {
    //             // we sent something, but not everything
    //         }
    //     } else {
    //         // failed to send. Check result for error code
    //         break;
    //     }


    //     // // eth.buf_out -> UDP
    //     // uint8_t buf[128];
    //     // const uint32_t uart_in_avail = _uart->available();
    //     // const uint32_t udp_out_txspace = _eth.buf_out.space();
    //     // if (uart_in_avail == 0) {
    //     //     break;
    //     // }
    //     // if (udp_out_txspace == 0) {
    //     //     break;
    //     // }
    //     // const uint32_t txfr_size = MIN(MIN(uart_in_avail, udp_out_txspace), sizeof(buf));


    //     // const uint32_t uart_read_count = _uart->read(&buf, MIN(buf_in_size, sizeof(buf)));
    //     // uart_in_avail -= uart_read_count;

    //     // const uint32_t udp_write_count = _eth.buf_out.write(buf, uart_read_count);
    //     // if (uart_read_count != udp_write_count) {
    //     //     //not all bytes were written to eth.buf_out
    //     //     break;
    //     // }
    //     // udp_out_txspace -= 

    // }






    // // process UDP.buf -> buf -> UART
    // const uint32_t udp_in_avail = _eth.buf_in.available();
    // const uint32_t uart_out_txspace = _uart->txspace();
    // if (udp_in_avail > 0 && uart_out_txspace > 0) {
    //     const uint32_t buf_size = MIN(MIN(udp_in_avail, uart_out_txspace), sizeof(buf));
    //     const ssize_t buf_read_count =_uart->read(buf, buf_size);
    //     if (buf_read_count != buf_size) {
    //         // not all bytes were read??!?!
    //     }
    //     const uint32_t buf_write_count = _eth.buf_in.write(buf, buf_read_count);
    //     if (buf_write_count != buf_size) {
    //         // not all read bytes were written??!?!
    //     }
    // }

}

// #pragma GCC diagnostic pop

int32_t AP_Networking_Serial2UDP::send_udp(AP_Networking_Serial2UDP::Eth_t &eth, const uint8_t* data, uint32_t data_len)
{
    if (data == nullptr || data_len == 0) {
        return 0;
    }

    data_len = MIN(data_len, AP_NETWORKING_MTU_SIZE);

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, data_len , PBUF_RAM);
    if (p == NULL) {
        return -99;
    }

    _eth.port = 1313;
    IP4_ADDR(&_eth.ip_addr, 255, 255, 255, 255);


    ip_addr_t dst;
    (void)dst;
    ip_addr_copy_from_ip4(dst, eth.ip_addr);

    memcpy(p->payload, data, data_len);
    err_t err = ERR_OK;
    err = udp_sendto(_serial2udp_pcb, p, &dst, _eth.port);
    pbuf_free(p);

    if (err == ERR_OK) {
        return data_len;
    }
    return err;
}


#endif // AP_NETWORKING_SERIAL2UDP_ENABLED
