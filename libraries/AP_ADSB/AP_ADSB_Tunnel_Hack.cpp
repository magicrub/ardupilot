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

#include "AP_ADSB_Tunnel_Hack.h"

#if HAL_ADSB_TUNNEL_HACK_ENABLED
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

#define HAL_ADSB_TUNNEL_HACK_USE_OLD_METHOD_UART_PARAM_BRD_SERIAL_NUM 1313

ADSB_Tunnel_Hack *ADSB_Tunnel_Hack::_singleton;

ADSB_Tunnel_Hack::ADSB_Tunnel_Hack()
{
    if (_singleton != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ADSB_Tunnel_Hack: multiple instances not supported");
        return;
    }
    _singleton = this;
}

bool ADSB_Tunnel_Hack::use_normal_uart_method() const
{
    AP_BoardConfig *bc = AP::boardConfig();
    if (bc == nullptr) {
        return true;
    }
    return (bc->AP_BoardConfig::get_serial_number() == HAL_ADSB_TUNNEL_HACK_USE_OLD_METHOD_UART_PARAM_BRD_SERIAL_NUM);
}

// ======================================================================================================
// INBOUND (Vehicle driver reading from CAN/periph)
// ======================================================================================================
bool ADSB_Tunnel_Hack::read(uint8_t &data)
{
    if (use_normal_uart_method()) {
        return uart->read(data);
    }

    WITH_SEMAPHORE(sem_read);
    return readbuf.read_byte(&data);
}

uint32_t ADSB_Tunnel_Hack::available()
{
    if (use_normal_uart_method()) {
        return uart->available();
    }

    WITH_SEMAPHORE(sem_read);
    return readbuf.available();
}

uint32_t ADSB_Tunnel_Hack::read_can_outbound(uint8_t *data, const uint32_t len)
{
    WITH_SEMAPHORE(sem_read);
    return readbuf.read(data, len);
}

// ======================================================================================================
// OUTBOUND (Vehicle driver writing to CAN/periph)
// ======================================================================================================
size_t ADSB_Tunnel_Hack::write(const uint8_t *buffer, const size_t size)
{
    if (use_normal_uart_method()) {
        return uart->write(buffer, size);
    }

    WITH_SEMAPHORE(sem_write);
    return writebuf.write(buffer, size);
}

uint32_t ADSB_Tunnel_Hack::txspace()
{
    if (use_normal_uart_method()) {
        return uart->txspace();
    }

    WITH_SEMAPHORE(sem_write);
    return writebuf.space();
}

uint32_t ADSB_Tunnel_Hack::available_can_outbound()
{
    WITH_SEMAPHORE(sem_write);
    return writebuf.available();
}

void ADSB_Tunnel_Hack::handle_can_msg(const uint8_t *data, const uint16_t len)
{
    WITH_SEMAPHORE(sem_write);
    (void)writebuf.read((uint8_t*)data, len);
}

#endif // HAL_ADSB_TUNNEL_HACK_ENABLED

