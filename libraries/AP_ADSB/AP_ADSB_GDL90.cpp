/*
   Copyright (C) 2020  Kraus Hamdani Aerospace Inc. All rights reserved.

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

#include "AP_ADSB_GDL90.h"

#if HAL_ADSB_GDL90_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>

// #include <GCS_MAVLink/GCS.h>
// #include <AP_AHRS/AP_AHRS.h>
// #include <AP_RTC/AP_RTC.h>
// #include <AP_HAL/utility/sparse-endian.h>
// #include <stdio.h>
// #include <time.h>
// #include <string.h>
// #include <math.h>

// detect if any port is configured as Sagetech
bool AP_ADSB_GDL90::detect()
{
    return (AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0) != nullptr);
}

// Init, called once after class is constructed
bool AP_ADSB_GDL90::init()
{
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);

    return (_port != nullptr);
}

void AP_ADSB_GDL90::update()
{
    if (_port == nullptr) {
        return;
    }

    // const uint32_t now_ms = AP_HAL::millis();

    // // -----------------------------
    // // read any available data on serial port
    // // -----------------------------
    // uint32_t nbytes = MIN(_port->available(), 10 * PAYLOAD_XP_MAX_SIZE);
    // while (nbytes-- > 0) {
    //     const int16_t data = (uint8_t)_port->read();
    //     if (data < 0) {
    //         break;
    //     }
    //     if (parse_byte_XP((uint8_t)data)) {
    //         handle_packet_XP(message_in.packet);
    //     }
    // } // while nbytes


    // // -----------------------------
    // // handle timers for generating data
    // // -----------------------------
    // if (!last_packet_initialize_ms || (now_ms - last_packet_initialize_ms >= 5000)) {
    //     last_packet_initialize_ms = now_ms;
    //     send_packet(MsgType_XP::Installation_Set);

    // } else if (!last_packet_PreFlight_ms || (now_ms - last_packet_PreFlight_ms >= 8200)) {
    //     last_packet_PreFlight_ms = now_ms;
    //     // TODO: allow callsign to not require a reboot
    //     send_packet(MsgType_XP::Preflight_Set);

    // } else if (now_ms - last_packet_Operating_ms >= 1000 && (
    //         last_packet_Operating_ms == 0 || // send once at boot
    //         // send as data changes
    //         last_operating_squawk != _frontend.out_state.cfg.squawk_octal ||
    //         abs(last_operating_alt - _frontend._my_loc.alt) > 1555 ||      // 1493cm == 49ft. The output resolution is 100ft per bit
    //         last_operating_rf_select != _frontend.out_state.cfg.rfSelect))
    // {
    //     last_packet_Operating_ms = now_ms;
    //     last_operating_squawk = _frontend.out_state.cfg.squawk_octal;
    //     last_operating_alt = _frontend._my_loc.alt;
    //     last_operating_rf_select = _frontend.out_state.cfg.rfSelect;
    //     send_packet(MsgType_XP::Operating_Set);

    // } else if (now_ms - last_packet_GPS_ms >= (_frontend.out_state.is_flying ? 200 : 1000)) {
    //     // 1Hz when not flying, 5Hz when flying
    //     last_packet_GPS_ms = now_ms;
    //     send_packet(MsgType_XP::GPS_Set);
    // }

}

bool AP_ADSB_GDL90::hostTransmit(HOST_CHANNEL channel, uint8_t *buffer, uint16_t length)
{
    return false;
}

// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// Below is a driver supplied by uAvionix which was slightly modifyed to be adapted to C++ for ArduPilot.
// This is included this way for easier long-term maintenence.
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------

/**
  @brief  Stuff, frame and transmit a GDL90 message

  @param  channel The communication port for the message to be sent
  @param  message GDL90_MESSAGE to stuff and transmit
  @param  length Length of the message to transmit including CRC
  @retval uint16_t Number of bytes sent
*/
uint16_t AP_ADSB_GDL90::gdl90Transmit(HOST_CHANNEL channel, GDL90_TX_MESSAGE* message, uint16_t length)
{
  memset(gdl90FrameBuffer, 0, GDL90_TX_MAX_FRAME_LENGTH);
  
  uint16_t frameCrc = 0;
  uint8_t* data = (uint8_t*) &message->messageId;
  
  // Calculate the CRC off the message
  for (uint16_t i = 0; i < length - 2; i++)
    frameCrc = _GDL90crcTable[frameCrc >> 8] ^ (frameCrc << 8) ^ data[i];
  
  // Append CRC to payload
  message->payload[length - 3] = frameCrc & 0xFF;
  message->payload[length - 2] = (frameCrc >> 8) & 0xFF;
  
  // Set flag byte in frame buffer
  gdl90FrameBuffer[0] = GDL90_FLAG_BYTE;
  uint16_t frameIndex = 1;
  
  // Copy and stuff all payload bytes into frame buffer
  for (uint16_t i = 0; i < length; i++)
  {
    // Check for overflow of frame buffer
    if (frameIndex >= GDL90_TX_MAX_FRAME_LENGTH)
    {
      return 0;
    }
    
    if (data[i] == GDL90_FLAG_BYTE || data[i] == GDL90_CONTROL_ESCAPE_BYTE)
    {
      // Check for frame buffer overflow on stuffed byte
      if (frameIndex + 2 > GDL90_TX_MAX_FRAME_LENGTH)
      {
        return 0;
      }
      
      // Set control break and stuff this byte
      gdl90FrameBuffer[frameIndex++] = GDL90_CONTROL_ESCAPE_BYTE;
      gdl90FrameBuffer[frameIndex++] = data[i] ^ GDL90_STUFF_BYTE;
    }
    else
      gdl90FrameBuffer[frameIndex++] = data[i];
  }
  
  // Add end of frame indication
  gdl90FrameBuffer[frameIndex++] = GDL90_FLAG_BYTE;
  
  // Push packet to UART
  if (hostTransmit(channel, gdl90FrameBuffer, frameIndex))
    return frameIndex;
  
  return 0;
}


/**
  @brief  Sends the GDL90 heartbeat message

  @param  channel The communication port for the message to be sent
  @retval uint16_t Number of bytes sent
*/
uint16_t AP_ADSB_GDL90::gdl90SendHeartbeat(HOST_CHANNEL channel)
{
  GDL90_HEARTBEAT message;
  memset(&message, 0, sizeof(GDL90_HEARTBEAT));
  
  message.messageId = GDL90_ID_HEARTBEAT;
  message.status.one.gpsPositionValid = hostGetGPSFix() >= GPS_FIX_3D;
  message.status.one.uatInitialized = 1;
  message.status.one.addressType = (hostGetUATAddressType() == UAT_ADDRESS_ADSB_SELF_ASSIGNED);
  message.status.one.ident = hostGetIdent();
  message.status.one.maintenanceRequired = hostGetFaultStatus(0);
 
  message.status.two.utcOk = isRadioSyncdToUTC();
  // TODO CSA bits?
 
  uint32_t timeSinceMidnight = hostGetGPSTime_s() % SEC_IN_A_DAY;
  message.status.two.timestampMsb = timeSinceMidnight >> 16;
  message.timestamp = (uint16_t) timeSinceMidnight;
  
  // Get number of messages in the last second
  uint16_t uplink = 0;
  uint16_t traffic = 0;
  //  pingGetNewMessageCount(&uplink, &traffic); //TODO, add this support
  
  message.uatMessages = traffic;
  message.uplinkMessages = uplink;
  message.messageCount = UINT16_ENDIAN(message.messageCount);
  
  return gdl90Transmit(channel, (GDL90_TX_MESSAGE*) &message, sizeof(GDL90_HEARTBEAT));
}



/**
  @brief  Sends the GDL90 ownship message

  @param  channel The communication port for the message to be sent
  @retval uint16_t Number of bytes sent
*/
uint16_t AP_ADSB_GDL90::gdl90SendOwnshipReport(HOST_CHANNEL channel)
{
  GDL90_OWNSHIP_REPORT message;
  memset(&message, 0, sizeof(GDL90_OWNSHIP_REPORT));
  
  message.messageId = GDL90_ID_OWNSHIP_REPORT;
  
  GDL90_REPORT* report = &message.report;
  
  report->addressType = (GDL90_ADDRESS_TYPE) hostGetUATAddressType();
  
  uint32_t icaoAddress = hostGetICAOAddress();
  report->address[0] = icaoAddress >> 16;
  report->address[1] = icaoAddress >> 8;
  report->address[2] = icaoAddress;
  
  int32_t latitude = hostGetLat_ddE7();
  int32_t longitude = hostGetLon_ddE7();
  
  latitude = (latitude < 0) ? (uint32_t)((((uint64_t)latitude + 3600000000) * 16777216l) / 3600000000) : (uint32_t)(((uint64_t)latitude * 16777216l) / 3600000000);
  longitude = (longitude < 0) ? (uint32_t)((((uint64_t)longitude + 3600000000) * 16777216l) / 3600000000) : (uint32_t)(((uint64_t)longitude * 16777216l) / 3600000000);
  
  report->latitude[0] = latitude >> 16;
  report->latitude[1] = latitude >> 8;
  report->latitude[2] = latitude;
  
  report->longitude[0] = longitude >> 16;
  report->longitude[1] = longitude >> 8;
  report->longitude[2] = longitude;
  
  int32_t altitude = (((int64_t)hostGetBaroAlt_mm() * 10) / 3048);
  if (altitude > UAT_MAX_ALTITUDE_FT) altitude = UAT_MAX_ALTITUDE_FT;
  if (altitude < UAT_MIN_ALTITUDE_FT)  altitude = UAT_MIN_ALTITUDE_FT;
  altitude = (altitude + 1000) / 25;
  
  GDL90_MISCELLANEOUS misc;
  misc.track =  GDL90_MISC_TRUE_TRACK;
  misc.reportType = GDL90_MISC_REPORT_UPDATED;
  misc.agState = (hostGetAGState() == ADSB_ON_GROUND) ? GDL90_MISC_ON_GROUND : GDL90_MISC_AIRBORNE;
  
  report->altitude = hostGetBaroAltitudeValid() ? altitude : GDL90_ALTITUDE_INVALID;
  report->misc = misc.data;
  report->altitudeMisc = UINT16_ENDIAN(report->altitudeMisc);
  
  report->NACp = hostGetNACp();
  // Must be 0 if position unavailable
  report->NIC = hostGetNIC();
  
  // Load and byte swap velocity data
  report->horizontalVelocity = (hostGetGroundSpeed_mmps() * 10) / 5144;
  report->verticalVelocity = (int16_t) (hostGetVerticalVelocity_mmps() / 325.12);
  report->heading = (hostGetCOG_dE2() * 256)/36000;
  report->velocities = UINT32_ENDIAN(report->velocities);
  
  report->emitterCategory = hostGetEmitCat();
  report->emergencyCode = hostGetEmergencyState();
  
  memcpy(report->callsign, hostGetCallsign(), 8);

  return gdl90Transmit(channel, (GDL90_TX_MESSAGE*) &message, sizeof(GDL90_OWNSHIP_REPORT));
}



/**
  @brief  Verifies the received GDL90 message CRC

  @param  message* Pointer to received GDL90 message
  @param  length Length of received message
  @retval bool
*/
bool AP_ADSB_GDL90::gdl90VerifyMessageCrc(GDL90_RX_MESSAGE* message, uint16_t length)
{  
  if (length <= GDL90_OVERHEAD_LENGTH)
    return false;
    
  uint16_t messaageCrc = message->payload[length - 3] | (message->payload[length - 2] << 8);
  
  uint16_t calcCrc = 0;
  uint8_t* data = (uint8_t*) &message->messageId;
  
  for (uint16_t i = 0; i < length - 2; i++)
    calcCrc = _GDL90crcTable[calcCrc >> 8] ^ (calcCrc << 8) ^ data[i];
  
  return (messaageCrc == calcCrc);
}

/**
  @brief  Processes received GDL90 bytes into a GDL90_MESSAGE

  @param  data The received byte
  @param  message Pointer to a GDL90_RX_MESSAGE to store bytes in
  @param  status Pointer to a GDL90_RX_STATUS
  @retval bool
*/
bool AP_ADSB_GDL90::gdl90ParseByte(uint8_t data, GDL90_RX_MESSAGE* message, GDL90_RX_STATUS* status)
{
  switch (status->state)
  {
  case GDL90_RX_IDLE:
    {
      status->length = 0;
      
      if (data == GDL90_FLAG_BYTE)
        status->state = GDL90_RX_IN_PACKET;
      
      return false;
    }
    
  case GDL90_RX_IN_PACKET:
    {
      if (data == GDL90_CONTROL_ESCAPE_BYTE)
      {
        status->state = GDL90_RX_UNSTUFF;
        return false;
      }
      else if (data == GDL90_FLAG_BYTE)
      {
        // If we receive a flag byte, but no other data 
        // has been received we likely are desync'd so treat is as a start byte
        if (status->length != 0)
          status->state = GDL90_RX_END;
        else    
        {
          return false;   
        }
      }
      
      break;
    }
    
  case GDL90_RX_UNSTUFF:
    {
      data = data ^ GDL90_STUFF_BYTE;
      status->state = GDL90_RX_IN_PACKET;
      break;
    }

  case GDL90_RX_END:
    // unhandled
    break;
  }
  
  // Handle end of packet
  if (status->state == GDL90_RX_END)
  {
    if (gdl90VerifyMessageCrc(message, status->length))
    {
      status->complete++; 
      status->state = GDL90_RX_IDLE;
      return true;
    }
    else
    {
      status->state = GDL90_RX_IDLE;
      return false;
    }
  }
  
  // Check for rx overflow
  if (status->length >= GDL90_RX_MAX_PACKET_LENGTH)
  {
    status->overflow++; 
    status->state = GDL90_RX_IDLE;
    return false;
  }
  
  // Store the received byte
  message->raw[status->length] = data;
  status->length++;
  
  return false;
}
#endif // HAL_ADSB_GDL90_ENABLED

