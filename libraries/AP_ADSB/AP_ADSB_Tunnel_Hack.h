/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_ADSB_config.h"

#ifndef HAL_ADSB_TUNNEL_HACK_ENABLED
#define HAL_ADSB_TUNNEL_HACK_ENABLED HAL_ADSB_ENABLED
#endif

#if HAL_ADSB_TUNNEL_HACK_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>

#ifndef HAL_ADSB_TUNNEL_HACK_BUF_SIZE
#define HAL_ADSB_TUNNEL_HACK_BUF_SIZE 1024
#endif

class ADSB_Tunnel_Hack {
public:
    ADSB_Tunnel_Hack();

    /* Do not allow copies */
    CLASS_NO_COPY(ADSB_Tunnel_Hack);

    static ADSB_Tunnel_Hack *get_singleton() { return _singleton; }

    AP_HAL::UARTDriver *uart;


    // emulate AP_HAL::UARTDriver
    bool read(uint8_t &data);
    size_t write(const uint8_t *buffer, const size_t size);
    uint32_t available();
    uint32_t txspace();

    uint32_t available_can_outbound();
    uint32_t read_can_outbound(uint8_t *data, const uint32_t len);
    void handle_can_msg(const uint8_t *data, const uint16_t len);


    // handle "_port == nullptr" checks to do their intended uart check behavior. _port itself should never be null
    bool operator ==(const std::nullptr_t that) const { return (uart == that); } 
    bool operator !=(const std::nullptr_t that) const { return (uart != that); }

private:
    static ADSB_Tunnel_Hack *_singleton;

    bool use_normal_uart_method() const;

    ByteBuffer readbuf{HAL_ADSB_TUNNEL_HACK_BUF_SIZE};  //  inbound: CAN/periph to Vehicle driver
    ByteBuffer writebuf{HAL_ADSB_TUNNEL_HACK_BUF_SIZE}; // outbound: Vehicle driver to CAN/periph

    HAL_Semaphore sem_read;
    HAL_Semaphore sem_write;

    bool _thread_running;

};
#endif // HAL_ADSB_TUNNEL_HACK_ENABLED
