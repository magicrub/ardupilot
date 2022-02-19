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
/*
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_SBG.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

void AP_ExternalAHRS_SBG::send_config(void) const
{
}

// constructor
AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
bool AP_ExternalAHRS_SBG::check_uart()
{
    if (!port_opened) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);

    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    while (n--) {
        const int16_t data_byte = uart->read();
        if (data_byte < 0) {
            return false;
        }
        if (parse_byte((uint8_t)data_byte, _inbound_msg, _inbound_state)) {
            handle_msg(_inbound_msg);
        }
    }

    // if (pktoffset < bufsize) {
    //     ssize_t nread = uart->read(&pktbuf[pktoffset], MIN(n, unsigned(bufsize-pktoffset)));
    //     if (nread <= 0) {
    //         return false;
    //     }
    //     pktoffset += nread;
    // }


    return true;
}

bool AP_ExternalAHRS_SBG::parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &state)
{
    switch (state.parser) {
        case SBG_PACKET_PARSE_STATE::SYNC1:
            state.parser = (data == SBG_PACKET_SYNC1) ? SBG_PACKET_PARSE_STATE::SYNC2 : SBG_PACKET_PARSE_STATE::SYNC1;
            break;
            
        case SBG_PACKET_PARSE_STATE::SYNC2:
            state.parser = (data == SBG_PACKET_SYNC2) ? SBG_PACKET_PARSE_STATE::MSG : SBG_PACKET_PARSE_STATE::SYNC1;
            break;

        case SBG_PACKET_PARSE_STATE::MSG:
            msg.msgid = data;
            state.parser = SBG_PACKET_PARSE_STATE::CLASS;
            break;

        case SBG_PACKET_PARSE_STATE::CLASS:
            msg.msgclass = data;
            state.parser = SBG_PACKET_PARSE_STATE::LEN1;
            break;

        case SBG_PACKET_PARSE_STATE::LEN1:
            msg.len = data;
            state.parser = SBG_PACKET_PARSE_STATE::LEN2;
            break;

        case SBG_PACKET_PARSE_STATE::LEN2:
            msg.len |= uint16_t(data) << 8;
            state.data_count = 0;
            state.parser = (msg.len > 0) ? SBG_PACKET_PARSE_STATE::DATA : SBG_PACKET_PARSE_STATE::CRC1;
            break;

        case SBG_PACKET_PARSE_STATE::DATA:
            msg.data[state.data_count++] = data;
            if (state.data_count >= sizeof(msg.data)) {
                state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
            } else if (state.data_count >= msg.len) {
                state.parser = SBG_PACKET_PARSE_STATE::CRC1;
            }
            break;

        case SBG_PACKET_PARSE_STATE::CRC1:
            state.crc = data;
            state.parser = SBG_PACKET_PARSE_STATE::CRC2;
            break;

        case SBG_PACKET_PARSE_STATE::CRC2:
            state.crc |= uint16_t(data) << 8;
            state.parser = SBG_PACKET_PARSE_STATE::SYNC1; // skip ETX and go directly to SYNC1. Do not pass Go.
            {
                // CRC field is computed on [MSG(1), CLASS(1), LEN(2), DATA(msg.len)] fields
                const uint16_t crc = calcCRC(&msg, msg.len+4);
                if (crc == state.crc) {
                    return true;
                }
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: Failed CRC. Received 0x%04X, Expected 0x%04X", (unsigned)state.crc, (unsigned)crc);
            }
            break;

        case SBG_PACKET_PARSE_STATE::ETX:
        // we can just skip this state and let SYNC1 fail once when ti gets the ETX byte every time... same amount of work
        default:
            state.parser = SBG_PACKET_PARSE_STATE::SYNC1;
            break;
    }

    return false;
}

uint16_t AP_ExternalAHRS_SBG::calcCRC(const void *pBuffer, const uint16_t bufferSize)
{
    //Note, this is X-MODEM REVERSE_ORDER

    const uint8_t *pBytesArray = (const uint8_t*)pBuffer;
    uint16_t poly = 0x8408;
    uint16_t crc = 0;
    uint8_t carry;
    uint8_t i;
    uint16_t j;

    for (j = 0; j < bufferSize; j++) {
        crc = crc ^ pBytesArray[j];
        for (i = 0; i < 8; i++) {
            carry = crc & 1;
            crc = crc / 2;
            if (carry) {
                crc = crc^poly;
            }
        }
    }
    return crc;
}

void AP_ExternalAHRS_SBG::handle_msg(const sbgMessage &msg)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);

    // switch (msg.msgid) {
    //     case 120:
    //     case 121:
    //     case 122:
    //     case 123:
    //     case 124:
    //     break;
    // }
}

void AP_ExternalAHRS_SBG::update_thread()
{
    if (!port_opened) {
        // open port in the thread
        port_opened = true;
        uart->begin(baudrate, 1024, 512);
        send_config();
    }

    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_SBG::get_port(void) const
{
    if (uart == nullptr) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_SBG::healthy(void) const
{
    return uart != nullptr;
}

bool AP_ExternalAHRS_SBG::initialised(void) const
{
    return uart != nullptr;
}

bool AP_ExternalAHRS_SBG::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_SBG::get_filter_status(nav_filter_status &status) const
{

}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_SBG::send_status_report(mavlink_channel_t chan) const
{
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED

