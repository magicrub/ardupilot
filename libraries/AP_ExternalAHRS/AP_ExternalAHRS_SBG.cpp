/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details. #include <sbgCommon.h>

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_SBG.h"
#if HAL_EXTERNAL_AHRS_SBG_ENABLED

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

extern const AP_HAL::HAL &hal;

// constructor
AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Can only be one AP_ExternalAHRS_SBG");
    }
    _singleton = this;

    auto &sm = AP::serialmanager();
    _uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (_uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }
    _baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
    }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=4200"
void AP_ExternalAHRS_SBG::send_config(void)
{
    // sbgMessage pkt_out {};
    // send_msg(*uart, pkt_out);
}
#pragma GCC diagnostic pop

uint16_t AP_ExternalAHRS_SBG::create_packet(const sbgMessage &msg, const uint16_t len_max, uint8_t *buffer)
{
    if (msg.len > sizeof(msg.data)) {
        // msg.len is out of range
        return 0;
    }

    const uint16_t packet_len = (SBG_PACKET_OVERHEAD + msg.len);
    if (len_max < packet_len) {
        // msg.len is out of range OR len_max is not big enough for this packet (we just protected from a buffer overrun)
        return 0;
    }

    buffer[0] = SBG_PACKET_SYNC1;
    buffer[1] = SBG_PACKET_SYNC2;
    buffer[2] = msg.msgid;
    buffer[3] = msg.msgclass;
    buffer[4] = msg.len & 0xFF; // LSB first
    buffer[5] = msg.len >> 8;

    for (uint16_t i=0; i<msg.len; i++) {
        buffer[6+i] = msg.data[i];
    }

    const uint16_t crc = calcCRC(&msg, msg.len+4);

    buffer[packet_len-3] = crc & 0xFF; // LSB First
    buffer[packet_len-2] = crc >> 8;
    buffer[packet_len-1] = SBG_PACKET_ETX;

    return packet_len;
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
    WITH_SEMAPHORE(state.sem);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG: ID=%u, CLASS=%u, LEN=%u", (unsigned)msg.msgid, (unsigned)msg.msgclass, (unsigned)msg.len);

    if ((_SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_ECOM_0 || msg.msgclass == (_SbgEComClass)SBG_ECOM_CLASS_LOG_ECOM_1) {
        // SBG_ECOM_CLASS_LOG_ECOM_0 Class that contains sbgECom output log messages
        // SBG_ECOM_CLASS_LOG_ECOM_1 Class that contains sbgECom messages that handle high frequency output

        switch ((_SbgEComLog)msg.msgid) {
            // case SBG_ECOM_LOG_MAG:
            //     if (sizeof(SbgEComMagCalibResults) == msg.len) {
            //         SbgEComMagCalibResults pkt;
            //         memcpy(&pkt, msg.data, sizeof(pkt));
            //         handle_msg(pkt);
            //     }
            //     break;

            case SBG_ECOM_LOG_STATUS: /*!< Status general, clock, com aiding, solution, heave */
            case SBG_ECOM_LOG_UTC_TIME: /*!< Provides UTC time reference */
            case SBG_ECOM_LOG_IMU_DATA: /*!< Includes IMU status, acc., gyro, temp delta speeds and delta angles values */
            case SBG_ECOM_LOG_MAG: /*!< Magnetic data with associated accelerometer on each axis */
            case SBG_ECOM_LOG_MAG_CALIB: /*!< Magnetometer calibration data (raw buffer) */
            case SBG_ECOM_LOG_EKF_EULER: /*!< Includes roll, pitch, yaw and their accuracies on each axis */
            case SBG_ECOM_LOG_EKF_QUAT: /*!< Includes the 4 quaternions values */
            case SBG_ECOM_LOG_EKF_NAV: /*!< Position and velocities in NED coordinates with the accuracies on each axis */
            case SBG_ECOM_LOG_GPS1_VEL: /*!< GPS velocities from primary or secondary GPS receiver */
            case SBG_ECOM_LOG_GPS1_POS: /*!< GPS positions from primary or secondary GPS receiver */
            case SBG_ECOM_LOG_GPS1_HDT: /*!< GPS true heading from dual antenna system */
            case SBG_ECOM_LOG_GPS1_RAW: /*!< GPS 1 raw data for post processing. */

            case SBG_ECOM_LOG_AIR_DATA: /*!< Air Data aiding such as barometric altimeter and true air speed. */

            case SBG_ECOM_LOG_IMU_RAW_DATA: /*!< DEPRECATED: Private only log. */

            case SBG_ECOM_LOG_IMU_SHORT: /*!< Short IMU message recommended for post processing usages. */
                break;

            default:
                return;
            } // switch log

    } else if ((_SbgEComClass)msg.msgclass == SBG_ECOM_CLASS_LOG_CMD_0) {
        switch ((_SbgEComCmd)msg.msgid) {
            case SBG_ECOM_CMD_ACK: /*!< Acknowledge */
            case SBG_ECOM_CMD_SETTINGS_ACTION: /*!< Performs various settings actions */
            case SBG_ECOM_CMD_IMPORT_SETTINGS: /*!< Imports a new settings structure to the sensor */
            case SBG_ECOM_CMD_EXPORT_SETTINGS: /*!< Export the whole configuration from the sensor */

            /* Device info */
            case SBG_ECOM_CMD_INFO: /*!< Get basic device information */

            /* Sensor parameters */
            case SBG_ECOM_CMD_INIT_PARAMETERS: /*!< Initial configuration */
            case SBG_ECOM_CMD_MOTION_PROFILE_ID: /*!< Set/get motion profile information */
            case SBG_ECOM_CMD_IMU_ALIGNMENT_LEVER_ARM: /*!< Sensor alignment and lever arm on vehicle configuration */
            case SBG_ECOM_CMD_AIDING_ASSIGNMENT: /*!< Aiding assignments such as RTCM / GPS / Odometer configuration */

            /* Magnetometer configuration */
            case SBG_ECOM_CMD_MAGNETOMETER_MODEL_ID: //	 	= 11,		/*!< Set/get magnetometer error model information */
            case SBG_ECOM_CMD_MAGNETOMETER_REJECT_MODE: // 	= 12,		/*!< Magnetometer aiding rejection mode */
            case SBG_ECOM_CMD_SET_MAG_CALIB: // 				= 13,		/*!< Set magnetic soft and hard Iron calibration data */

            /* Magnetometer on-board calibration */
            case SBG_ECOM_CMD_START_MAG_CALIB: //			= 14,		/*!< Start / reset internal magnetic field logging for calibration. */
            case SBG_ECOM_CMD_COMPUTE_MAG_CALIB: //			= 15,		/*!< Compute a magnetic calibration based on previously logged data. */

            /* GNSS configuration */
            case SBG_ECOM_CMD_GNSS_1_MODEL_ID: // 			= 17,		/*!< Set/get GNSS model information */
            case SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT: // = 18,		/*!< DEPRECATED: GNSS installation configuration (lever arm, antenna alignments) */
            case SBG_ECOM_CMD_GNSS_1_INSTALLATION: //		= 46,		/*!< Define or retrieve the GNSS 1 main and secondary lever arms configuration. */
            case SBG_ECOM_CMD_GNSS_1_REJECT_MODES: // 		= 19,		/*!< GNSS aiding rejection modes configuration. */

            /* Odometer configuration */
            case SBG_ECOM_CMD_ODO_CONF: // 					= 20,		/*!< Odometer gain, direction configuration */
            case SBG_ECOM_CMD_ODO_LEVER_ARM: // 				= 21,		/*!< Odometer installation configuration (lever arm) */
            case SBG_ECOM_CMD_ODO_REJECT_MODE: // 			= 22,		/*!< Odometer aiding rejection mode configuration. */

            /* Interfaces configuration */
            case SBG_ECOM_CMD_UART_CONF: // 					= 23,		/*!< UART interfaces configuration */
            case SBG_ECOM_CMD_CAN_BUS_CONF: // 				= 24,		/*!< CAN bus interface configuration */
            case SBG_ECOM_CMD_CAN_OUTPUT_CONF: //			= 25,		/*!< CAN identifiers configuration */

            /* Events configuration */
            case SBG_ECOM_CMD_SYNC_IN_CONF: // 				= 26,		/*!< Synchronization inputs configuration */
            case SBG_ECOM_CMD_SYNC_OUT_CONF: // 				= 27,		/*!< Synchronization outputs configuration */

            /* Output configuration */
            case SBG_ECOM_CMD_NMEA_TALKER_ID: // 			= 29,		/*!< NMEA talker ID configuration */
            case SBG_ECOM_CMD_OUTPUT_CONF: // 				= 30,		/*!< Output configuration */

            /* Advanced configuration */
            case SBG_ECOM_CMD_ADVANCED_CONF: // 				= 32,		/*!< Advanced settings configuration */

            /* Features related commands */
            case SBG_ECOM_CMD_FEATURES: //					= 33,		/*!< Retrieve device features */

            /* Licenses related commands */
            case SBG_ECOM_CMD_LICENSE_APPLY: //				= 34,		/*!< Upload and apply a new license */

            /* Message class output switch */
            case SBG_ECOM_CMD_OUTPUT_CLASS_ENABLE: //		= 35,		/*!< Enable/disable the output of an entire class */

            /* Ethernet configuration */
            case SBG_ECOM_CMD_ETHERNET_CONF: //				= 36,		/*!< Set/get Ethernet configuration such as DHCP mode and IP address. */
            case SBG_ECOM_CMD_ETHERNET_INFO: //				= 37,		/*!< Return the current IP used by the device. */

            /* Validity thresholds */
            case SBG_ECOM_CMD_VALIDITY_THRESHOLDS: //		= 38,		/*!< Set/get Validity flag thresholds for position, velocity, attitude and heading */

            /* DVL configuration */
            case SBG_ECOM_CMD_DVL_MODEL_ID: //				= 39,		/*!< Set/get DVL model id to use */
            case SBG_ECOM_CMD_DVL_INSTALLATION: //			= 40,		/*!< DVL installation configuration (lever arm, alignments) */
            case SBG_ECOM_CMD_DVL_REJECT_MODES: //			= 41,		/*!< DVL aiding rejection modes configuration. */

            /* AirData configuration */
            case SBG_ECOM_CMD_AIRDATA_MODEL_ID: //			= 42,		/*!< Set/get AirData model id and protocol to use. */
            case SBG_ECOM_CMD_AIRDATA_LEVER_ARM: //			= 43,		/*!< AirData installation configuration (lever arm, offsets) */
            case SBG_ECOM_CMD_AIRDATA_REJECT_MODES: //		= 44,		/*!< AirData aiding rejection modes configuration. */

            /* Odometer configuration (using CAN) */
            case SBG_ECOM_CMD_ODO_CAN_CONF: // 				= 45,		/*!< Configuration for CAN based odometer (CAN ID & DBC) */
            default:
                break;

        } // switch cmd
    } else {
        // unhandled class type
        return;
    }
}

void AP_ExternalAHRS_SBG::update(void)
{

}

void AP_ExternalAHRS_SBG::update_thread()
{
    if (_uart == nullptr) {
        return;
    }

    if (!_port_opened) {
        // open port in the thread
        _uart->begin(_baudrate, 1024, 512);
        _port_opened = true;
        send_config();
    }

    while (true) {
        uint32_t n = _uart->available();
        if (n == 0) {
            hal.scheduler->delay(1);
            continue;
        }
        while (n--) {
            const int16_t data_byte = _uart->read();
            if (data_byte < 0) {
                continue;
            }
            if (parse_byte((uint8_t)data_byte, _inbound_state.msg, _inbound_state)) {
                handle_msg(_inbound_state.msg);
            }
        }
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_SBG::get_port(void) const
{
    return (_uart == nullptr) ? -1 : _port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_SBG::healthy(void) const
{
    return _uart != nullptr;
}

bool AP_ExternalAHRS_SBG::initialised(void) const
{
    return _uart != nullptr;
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

void AP_ExternalAHRS_SBG::calibrate_mag_start() const
{
    // sbgEComCmdMagStartCalib
}

void AP_ExternalAHRS_SBG::calibrate_mag_stop() const
{
    // bgEComCmdMagComputeCalib
}

void AP_ExternalAHRS_SBG::calibrate_mag_set() const
{
    // bgEComCmdMagComputeCalib
}

bool AP_ExternalAHRS_SBG::calibration_ok(const Vector3f &mag) const
{
    const float AHRS_norm = sqrtf(mag.x*mag.x + mag.y*mag.y + mag.z*mag.z);

    const float threshold_HIGH = 1.02f;
    const float threshold_LOW = 0.98f;

    return (AHRS_norm <= threshold_HIGH && AHRS_norm >= threshold_LOW);

}

void AP_ExternalAHRS_SBG::configure_sensor()
{
    // page 12, section 2.2.2

    // sbgEComCmdGnss1SetLeverArmAlignment
    // (platform-dependent)
    // sbgEComCmdSensorSetAlignmentAndLeverArm
    // (platform-dependent)
    // sbgEComCmdSensorSetMotionProfileId
    // SBG_ECOM_MOTION_PROFILE_AIRPLANE
    // sbgEComCmdMagSetRejection
    // SBG_ECOM_AUTOMATIC_MODE
    // sbgEComCmdMagSetModelId
    // SBG_ECOM_MAG_MODEL_NORMAL
    // sbgEComCmdGnss1SetRejection
    // SBG_ECOM_ALWAYS_ACCEPT_MODE,
    // SBG_ECOM_ALWAYS_ACCEPT_MODE,
    // SBG_ECOM_NEVER_ACCEPT_MODE,
    // SBG_ECOM_AUTOMATIC_MODE
    // sbgEComCmdGnssSetModelId
    // SBG_ECOM_GNSS_MODEL_UBLOX_GPS_GLONASS
    // sbgEComCmdSyncOutSetConf
    // SBG_ECOM_SYNC_OUT_A, SBG_ECOM_SYNC_OUT_MODE_DIRECT_PPS
}

void AP_ExternalAHRS_SBG::handle_msg(const SbgEComMagCalibResults &msg)
{

}

// singleton instance
AP_ExternalAHRS_SBG *AP_ExternalAHRS_SBG::_singleton;
namespace AP {
AP_ExternalAHRS_SBG *sbg() {
    return AP_ExternalAHRS_SBG::get_singleton();
}
};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

