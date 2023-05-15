#include "AP_Networking_Serial.h"

#if AP_SERIAL_EXTENSION_ENABLED
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
        snprintf(thread_name, sizeof(thread_name), "UDP-%u", dst_port);
        if(!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_Serial::thread, void), thread_name, 512, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
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
        if (_writebuf.available()) {
            ByteBuffer::IoVec vec[2];
            const auto n_vec = _writebuf.peekiovec(vec, _writebuf.available());
            for (uint8_t i=0; i<n_vec; i++) {
                struct pbuf *p = pbuf_alloc_reference((void*)vec[i].data, vec[i].len, PBUF_REF);
                if (p == nullptr) {
                    break;
                }
                const err_t err = udp_sendto(pcb, p, &dst_addr, _params.port);
                pbuf_free(p);
                if (err != ERR_OK) {
                    break;
                }

                {
                    WITH_SEMAPHORE(_tx_sem);
                    _writebuf.advance(vec[i].len);
                }
            }
        }
        hal.scheduler->delay(5);
    }
}

#endif
