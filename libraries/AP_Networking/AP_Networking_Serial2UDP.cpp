
#include "AP_Networking_Serial2UDP.h"

#if AP_NETWORKING_SERIAL2UDP_ENABLED
#include <AP_Math/AP_Math.h> // for MIN()

//struct udp_pcb *AP_Networking_Serial2UDP::_serial2udp_pcb;
uint8_t AP_Networking_Serial2UDP::_serial2udp_instance_count = 0;
// AP_Networking_Serial2UDP::Eth_static_t AP_Networking_Serial2UDP::_eth_static[AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES];

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
    if (_serial2udp_instance_count >= AP_NETWORKING_SERIAL2UDP_MAX_INSTANCES) {
        printf("\n\nSerial2UDP init failed, too many instances\r\n");
        return;
    }

    _serial.uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Network_Serial2UDP, _serial2udp_instance_count);
    if (_serial.uart == nullptr) {
        printf("\n\nSerial2UDP init failed, could not find serial port\r\n");
        return;
    }

    _instance = _serial2udp_instance_count;
    _serial2udp_instance_count++;

    const uint32_t baudrate = AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Network_Serial2UDP, _instance);
    _serial.uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _serial.uart->begin(baudrate, AP_NETWORKING_SERIAL2UDP_BUFFER_UART_RX_SIZE, AP_NETWORKING_SERIAL2UDP_BUFFER_UART_TX_SIZE);

    _eth.port = 10000 + _instance;
    //IP4_ADDR(_eth->ip_addr, 255, 255, 255, 255);
    //_eth.u_addr.ip4.addr = IPADDR_BROADCAST;

#if !AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_UNBUFFERED
    {
        WITH_SEMAPHORE(_eth.sem);
        _eth.buf_in.set_size(1000);
    }
#endif

    IP4_ADDR(&_eth.ip4_addr, 255, 255, 255, 255);

    if (_eth.pcb == NULL) {
        _eth.pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (_eth.pcb != NULL) {


            // // allow to sending broadcast packets
            ip_set_option(_eth.pcb, SOF_BROADCAST);
            //udp_bind(_eth.pcb, IP_ANY_TYPE, _eth.port);


            // // listen only
            // ip_set_option(_eth.pcb, SOF_BROADCAST);
            // udp_bind(_eth.pcb, IP_ANY_TYPE, _eth.port);


            // // set up RX callback
            udp_recv(_eth.pcb, AP_Networking_Serial2UDP::serial2udp_recv_callback, _eth.pcb);
        }
    }
}

void AP_Networking_Serial2UDP::serial2udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    // static uint32_t rx_count = 0;
    //printf("serial2udp_recv_callback\r\n");
    // printf("\r\n%u, %u, %u, %u ", AP_HAL::millis(), rx_count++, (unsigned)port, (unsigned)p->len);

    // if (port != 13131) {
    //     return;
    // }

    if (p == nullptr) {
        return;
    }
    // const int32_t instance = find_eth_static_instance_by_port(port);
    // if (instance >= 0) {
    //     WITH_SEMAPHORE(_eth_static[instance].sem);
    //     _eth_static[instance].buf_in.write((uint8_t*)p->payload, p->len);
    //     _eth_static[instance].last_rx_ms = AP_HAL::millis();
    // }


    AP_Networking_Serial2UDP* driver = get_Serial2UDP_backend(pcb, port);
    if (driver != nullptr)
    {
        WITH_SEMAPHORE(driver->_eth.sem);
#if AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_UNBUFFERED
        driver->_serial.uart->write((uint8_t*)p->payload, p->len);
#else
        driver->_eth.buf_in.write((uint8_t*)p->payload, p->len);
#endif
        driver->_eth.last_rx_ms = AP_HAL::millis();
    }
    pbuf_free(p);
}

AP_Networking_Serial2UDP* AP_Networking_Serial2UDP::get_Serial2UDP_backend(struct udp_pcb *pcb, const uint16_t port)
{
    for (uint8_t i = 0; i < _serial2udp_instance_count; i++) {
        AP_Networking_Serial2UDP* driver = (AP_Networking_Serial2UDP*)AP::network()._drivers[i];
        if (driver != nullptr && driver->_eth.pcb == pcb && driver->_eth.port == port) {
            return driver;
        }
    }
    return nullptr;
}

void AP_Networking_Serial2UDP::update()
{
    if (_serial.uart == nullptr || _eth.pcb == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

#if 0
    // throughput test by sending whatever is in _eth.buf_out
    // by keeping _eth.buf_out[] filled with dummy data 'z'
    uint8_t buf2[1] = {'z'};
    while (_eth.buf_out.space() && _eth.buf_out.write(buf2, 1));

    const uint32_t send_count = 2;
    for (uint32_t i=0; i<send_count; i++) {
        result = serial2udp_send_buf_out();
        if (result < 0) {
        //     printf("%d send_udp(%d) result %d\r\n", (int)now_ms, (int)i, (int)result);
        //     break;
        }
    }
#endif

#if 1
    // process eth.buf_out -> UDP
    if (_eth.buf_out.available()) {
        const bool send_now = (_eth.last_tx_ms == 0) || (AP_Networking_Serial2UDP_DELAY_MS <= 0);
        if (send_now || (_eth.buf_out.space() == 0) || (now_ms - _eth.last_tx_ms > AP_Networking_Serial2UDP_DELAY_MS)) {
            // we've either filled up out out buffer, expired our timer, or a send is forced

            // send it!
            int32_t result = serial2udp_send_buf_out();
            if (result > 0) {
                // send is complete!
                _eth.last_tx_ms = now_ms;
            }
        }
    }
#endif




#if !AP_NETWORKING_SERIAL2UDP_UDP_TO_UART_UNBUFFERED
    // process eth.buf_in -> UART
    const uint32_t udp_in_avail = _eth.buf_in.available();
    const uint32_t uart_out_txspace = _serial.uart->txspace();
    if (udp_in_avail > 0 && uart_out_txspace > 0) {
        WITH_SEMAPHORE(_eth.sem);
        uint32_t ptr_continuous_length;
        const uint8_t *ptr = _eth.buf_in.readptr(ptr_continuous_length);

        // NOTE: ptr_continuous_length may not be the same as udp_in_avail.
        // If it's smaller, the ring buffer wrapped and the next call to
        // update() will send the rest
        const size_t result = _serial.uart->write(ptr, ptr_continuous_length);
        if (result > 0) {
            _eth.buf_in.advance(result);
            _serial.last_tx_ms = now_ms;
        }
    }
#endif

#if 1
    // process UART -> eth.buf_out
    const uint32_t uart_available = _serial.uart->available();
    const uint32_t eth_in_space = _eth.buf_out.space();
    if (uart_available > 0 && eth_in_space > 0) {
        const uint32_t read_len_max = MIN(uart_available, eth_in_space);
        uint8_t buf[read_len_max];
        
        const uint32_t read_len = _serial.uart->read(buf, sizeof(buf));
        _eth.buf_out.write(buf, read_len);
        _serial.last_rx_ms = now_ms;
    }
#endif

}



int32_t AP_Networking_Serial2UDP::serial2udp_send_buf_out()
{
    const uint32_t available = _eth.buf_out.available();
    uint32_t ptr_continuous_length;
    const uint8_t *ptr = _eth.buf_out.readptr(ptr_continuous_length);
    const uint32_t n = MIN(AP_NETWORKING_SERIAL2UDP_UDP_MAX_PACKET_SIZE, ptr_continuous_length); // the packet_size we want to send

    if (n == available) {
        // simple transfer, ring buffer is not wrapping so just toss the sender the ring buffer pointer
        const int32_t result = AP_Networking::send_udp(_eth.pcb, _eth.ip4_addr, _eth.port, ptr, n);
        if (result > 0) {
            _eth.buf_out.advance(result);
        }
        return result;
    }

    // ring buffer wraps so we need to fill a temp buffer
    uint8_t buf[MIN(AP_NETWORKING_SERIAL2UDP_UDP_MAX_PACKET_SIZE, available)];
    const uint32_t read_count = _eth.buf_out.read(buf, sizeof(buf));

    return AP_Networking::send_udp(_eth.pcb, _eth.ip4_addr, _eth.port, buf, read_count);
}



#endif // AP_NETWORKING_SERIAL2UDP_ENABLED
