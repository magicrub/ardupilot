#include "AP_Networking_Serial.h"

#if AP_SERIAL_EXTENSION_ENABLED
#include "AP_Math/AP_Math.h"

extern const AP_HAL::HAL& hal;

void AP_Networking_Serial::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    if (_initialized) {
        return;
    }
    if (rxS == 0) {
        rxS = AP_NETWORKING_UDP_RX_BUF_SIZE;
    }
    if (txS == 0) {
        txS = AP_NETWORKING_UDP_TX_BUF_SIZE;
    }
    // initialise bytebuffers
    _readbuf.set_size(rxS);
    _writebuf.set_size(txS);

    // setup UDP socket
    if (pcb == nullptr) {
        pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
        if (pcb != nullptr) {
            // allow to sending broadcast packets
            ip_set_option(pcb, SOF_BROADCAST);
            // set up RX callback
            udp_recv(pcb, AP_Networking_Serial::udp_recv_callback, this);
        }
    }

    // start a thread to send data
    if (thread_ctx == nullptr) {
        // create thread name
        snprintf(thread_name, sizeof(thread_name), "UDP-%u", (unsigned)_params.port.get());
        if(!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Serial::thread, void), thread_name, 1024, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
            AP_HAL::panic("Failed to create AP_Networking_Serial thread");
        }
    }
    _initialized = true;
}

void AP_Networking_Serial::udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    AP_Networking_Serial* driver = (AP_Networking_Serial*)arg;
    if (driver != nullptr)
    {
        WITH_SEMAPHORE(driver->_rx_sem);
        driver->_readbuf.write((uint8_t*)p->payload, p->len);
    }
    pbuf_free(p);
}

void AP_Networking_Serial::end()
{
    if (pcb != nullptr) {
        udp_remove(pcb);
        pcb = nullptr;
    }
}

void AP_Networking_Serial::flush()
{
    // nothing to do
}

void AP_Networking_Serial::set_blocking_writes(bool blocking)
{
    blocking_writes = blocking;
}

bool AP_Networking_Serial::read(uint8_t &b)
{
    WITH_SEMAPHORE(_rx_sem);
    return (_readbuf.read(&b, 1) == 1) ? true : false;
}

ssize_t AP_Networking_Serial::read(uint8_t *buffer, uint16_t count)
{
    WITH_SEMAPHORE(_rx_sem);
    return _readbuf.read(buffer, count);
}

size_t AP_Networking_Serial::write(uint8_t c)
{
    return write(&c, 1);
}

size_t AP_Networking_Serial::write(const uint8_t *buffer, size_t size)
{
    if (pcb == nullptr) {
        return 0;
    }
    if (size == 0) {
        return 0;
    }
    if (blocking_writes) {
        struct pbuf *p = pbuf_alloc_reference((void*)buffer, size, PBUF_REF);
        if (p == nullptr) {
            return 0;
        }
        const err_t err = udp_sendto(pcb, p, &dst_addr, _params.port);
        pbuf_free(p);
        if (err != ERR_OK) {
            return 0;
        }
    } else {
        WITH_SEMAPHORE(_tx_sem);
        if (_writebuf.space() < size) {
            return 0;
        }
        _writebuf.write(buffer, size);
    }
    return size;
}

void AP_Networking_Serial::thread()
{
    while(true) {

        if (_params.protocol == 0 && _params.passthru > 0) {
            // serial passthrough mode
            auto uart = hal.serial(_params.passthru);
            if (uart != nullptr && uart->is_initialized()) {
                uint8_t buf[32];

                // uart -> IP buffer queue transfer
                const uint32_t uart_to_ip_min_len = MIN(uart->available(), _writebuf.space());
                if (uart_to_ip_min_len > 0) {
                    const ssize_t uart_to_ip_count = uart->read(buf, MIN(sizeof(buf), uart_to_ip_min_len));
                    if (uart_to_ip_count > 0) {
                        write(buf, uart_to_ip_count);
                    }
                }

                // IP -> uart buffer queue transfer
                const uint32_t ip_to_uart_min_len = MIN(_readbuf.available(), uart->txspace());
                if (ip_to_uart_min_len > 0) {
                    const ssize_t ip_to_uart_count = read(buf, MIN(sizeof(buf), ip_to_uart_min_len));
                    if (ip_to_uart_count > 0) {
                        uart->write(buf, ip_to_uart_count);
                    }
                }
            }
        }
        
        

        const uint32_t available = _writebuf.available();
        if (available > 0) {
            uint16_t bytes_already_queued = (_p_tx != nullptr) ? _p_tx->tot_len : 0;
            const uint32_t len = MIN(available, AP_NETWORKING_UDP_TX_BUF_SIZE - bytes_already_queued);

            ByteBuffer::IoVec vec[2];
            const auto n_vec = _writebuf.peekiovec(vec, len);
            for (uint8_t i=0; i<n_vec; i++) {
                if (_p_tx == nullptr) {
                    // create new pbuf chain
                    _p_tx = pbuf_alloc_reference((void*)vec[i].data, vec[i].len, PBUF_REF);
                    if (_p_tx == nullptr) {
                        break;
                    }
                } else {
                    // append to existing pbuf chain
                    pbuf_chain(_p_tx, pbuf_alloc_reference((void*)vec[i].data, vec[i].len, PBUF_REF));
                }
            }

            if (_p_tx != nullptr) {
                WITH_SEMAPHORE(_tx_sem);
                _writebuf.advance(len);
            }
        }

        if ((_p_tx != nullptr) && _p_tx->tot_len > 0) {
            // a packet buffer has been filled, send it!

            const uint64_t now_us = AP_HAL::micros64();
            bool send_it = (_p_tx->tot_len >= AP_NETWORKING_UDP_TX_BUF_SIZE); // packet is full
            send_it |= (now_us - _last_ip_send_us) >= 5*1000;               // 5ms since last send
            //TODO: send_it |= end_of_packet_detected; // protocol aware packetization

            if (send_it) {
                const err_t err = udp_sendto(pcb, _p_tx, &dst_addr, _params.port);
                pbuf_free(_p_tx);
                if (err == ERR_OK) {
                    _last_ip_send_us = now_us;
                }
            }
        }

        hal.scheduler->delay_microseconds(1000);
    }
}


#endif
            // AP_NETWORKING_ETHERNET_UDP_PAYLOAD_MAX_SIZE
