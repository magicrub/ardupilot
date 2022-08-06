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
  support for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_SBG_ENABLED
#define HAL_EXTERNAL_AHRS_SBG_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_SBG_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_SBG : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_SBG(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    static AP_ExternalAHRS_SBG *get_singleton(void) { return _singleton; }

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    void calibrate_mag_start() const;
    void calibrate_mag_stop() const;
    void calibrate_mag_set() const;
    bool calibration_ok(const Vector3f &mag) const;
    bool calibration_ok() const { return calibration_ok(Vector3f(1,1,1)); }

    void configure_sensor();

    //SbgEComDeviceInfo get_version() const { return _version; };
    uint32_t get_version_hardwareRev() const { return 0; };
    uint32_t get_version_firmwareRev() const { return 0; };

    // periodic update
    void update() override;

private:
    static AP_ExternalAHRS_SBG *_singleton;

    static constexpr uint8_t SBG_PACKET_SYNC1 = 0xFF;
    static constexpr uint8_t SBG_PACKET_SYNC2 = 0x5A;
    static constexpr uint8_t SBG_PACKET_ETX = 0x33;
    static constexpr uint16_t SBG_PACKET_PAYLOAD_SIZE_MAX = 4086; // largest packet we care about is 72
    static constexpr uint16_t SBG_PACKET_OVERHEAD = 9; // sync1, sync2, id, class, lenLSB, lenMSB, crcLSB, crcMSB, etx

    struct PACKED sbgMessage {

        uint8_t msgid;
        uint8_t msgclass;
        uint16_t len;
        uint8_t data[SBG_PACKET_PAYLOAD_SIZE_MAX];

        sbgMessage() {};

        sbgMessage(const uint8_t msgId_, const uint8_t msgClass_) {
            msgid = msgId_;
            msgclass = msgClass_;
            len = 0;
        };

        sbgMessage(const uint8_t msgId_, const uint8_t msgClass_, const uint16_t len_, const uint8_t &data_) {
            msgid = msgId_;
            msgclass = msgClass_;
            len = MIN(sizeof(data), len_);
            memcpy(data, &data_, len);
        };

    };

    enum class SBG_PACKET_PARSE_STATE : uint8_t {
        SYNC1,
        SYNC2,
        MSG,
        CLASS,
        LEN1,
        LEN2,
        DATA,
        CRC1,
        CRC2,
        ETX
    };


    struct SBG_PACKET_INBOUND_STATE {
        SBG_PACKET_PARSE_STATE parser;
        uint16_t data_count;
        uint16_t crc;
        sbgMessage msg;
    } _inbound_state;


    void handle_msg(const sbgMessage &msg);
    static uint16_t calcCRC(const void *pBuffer, const uint16_t bufferSize);

    static bool parse_byte(const uint8_t data, sbgMessage &msg, SBG_PACKET_INBOUND_STATE &state);
    static void send_msg(AP_HAL::UARTDriver& uart_driver, const sbgMessage &msg);
    // static void send_msg(AP_HAL::UARTDriver &uart_driver, const uint8_t msgId, const uint8_t msgClass, const uint16_t len, const uint8_t &data);
    static void send_msg(AP_HAL::UARTDriver &uart_driver, const uint8_t msgId, const uint8_t msgClass, const uint16_t len, const uint8_t &data);
    static uint16_t create_packet(const sbgMessage &msg, const uint16_t len_max, uint8_t *data);


    AP_HAL::UARTDriver *_uart;
    int8_t _port_num;
    bool _port_opened;
    uint32_t _baudrate;
    uint16_t _rate;

    //SbgEComDeviceInfo _version;

    void update_thread();

    void send_config(void);

    //void handle_msg(const SbgEComMagCalibResults &msg);

};

namespace AP {
    AP_ExternalAHRS_SBG *sbg();
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

