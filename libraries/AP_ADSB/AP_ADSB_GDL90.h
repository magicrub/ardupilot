#pragma once

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

#include "AP_ADSB_Backend.h"

#ifndef HAL_ADSB_GDL90_ENABLED
#define HAL_ADSB_GDL90_ENABLED HAL_ADSB_ENABLED
#endif

#if HAL_ADSB_GDL90_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>

#include "GDL90_protocol/GDL90_Message_Structs.h"
#include "GDL90_protocol/hostGDL90Support.h"

class AP_ADSB_GDL90 : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    // init - performs any required initialisation for this instance
    bool init() override;

    // update - should be called periodically
    void update() override;

    // static detection function
    static bool detect();

private:

    // helpers to translate uAvionix library to ArduPilot library calls
    typedef uint8_t HOST_CHANNEL;
    bool hostTransmit(HOST_CHANNEL channel, uint8_t *buffer, uint16_t length);
    GPS_FIX hostGetGPSFix() { return (GPS_FIX)AP::gps().status(); }
    int32_t hostGetUATAddressType() { return _frontend.out_state.cfg.ICAO_id_param.get(); }
    static constexpr int32_t UAT_ADDRESS_ADSB_SELF_ASSIGNED = 0; // 0 means its auto_generated
    bool hostGetIdent() { return false; }
    bool hostGetFaultStatus(uint8_t unused) { return false; }
    bool isRadioSyncdToUTC() { return (AP::gps().time_epoch_usec() != 0); }
    uint32_t hostGetGPSTime_s() { return (AP::gps().time_week_ms() * 0.001f); } // seconds since midnight
    static constexpr uint32_t SEC_IN_A_DAY = (24 * 60 * 60);
    uint16_t UINT16_ENDIAN(uint16_t value) { return value; }
    uint32_t UINT32_ENDIAN(uint16_t value) { return value; }    
    uint32_t hostGetICAOAddress() { return _frontend.out_state.cfg.ICAO_id; }
    int32_t hostGetLat_ddE7() { return _frontend._my_loc.lat; }
    int32_t hostGetLon_ddE7() { return _frontend._my_loc.lng; }
    int64_t hostGetBaroAlt_mm() { return AP::baro().get_altitude() * 1000; }
    static constexpr int32_t UAT_MAX_ALTITUDE_FT = 328084; // 100km is the Karman line where we're officially in space. Could also use (INT32_MAX - 1) but that's boring...
    static constexpr int32_t UAT_MIN_ALTITUDE_FT = -1000;
    ADSB_AIR_GROUND_STATE hostGetAGState() { return _frontend.out_state.is_flying ? ADSB_AIRBORNE_SUBSONIC : ADSB_ON_GROUND; }
    bool hostGetBaroAltitudeValid() { return AP::baro().healthy(); }
    static constexpr int32_t GDL90_ALTITUDE_INVALID = INT32_MAX;
    int32_t hostGetGroundSpeed_mmps() { return AP::ahrs().groundspeed() * 1000; }
    int32_t hostGetVerticalVelocity_mmps() { Vector3f vel; return AP::ahrs().get_velocity_NED(vel) ? vel.z * 1000.0f : 0; }
    int32_t hostGetCOG_dE2() { return AP::ahrs().groundspeed_vector().angle(); }
    uint8_t hostGetEmitCat() { return _frontend.out_state.cfg.emitterType.get(); }
    uint8_t hostGetEmergencyState() { return (uint8_t)ADSB_EMERGENCY_NONE; }
    char* hostGetCallsign() { return _frontend.out_state.cfg.callsign; }

    // ?????
    uint8_t hostGetNACp() { return 0; }
    uint8_t hostGetNIC() { return 0; }

    // uAvionix library calls
    uint16_t gdl90Transmit(HOST_CHANNEL channel, GDL90_TX_MESSAGE* message, uint16_t length);
    uint16_t gdl90SendHeartbeat(HOST_CHANNEL channel);
    uint16_t gdl90SendOwnshipReport(HOST_CHANNEL channel);
    bool gdl90VerifyMessageCrc(GDL90_RX_MESSAGE* message, uint16_t length);
    bool gdl90ParseByte(uint8_t data, GDL90_RX_MESSAGE* message, GDL90_RX_STATUS* status);


    uint8_t gdl90FrameBuffer[GDL90_TX_MAX_FRAME_LENGTH];

    // GDL90_RX_MESSAGE* message;
    // GDL90_RX_STATUS* status;

    // cached variables to compare against params so we can send msg on param change.
    uint16_t        last_operating_squawk;
    int32_t         last_operating_alt;
    uint8_t         last_operating_rf_select;
};
#endif // HAL_ADSB_GDL90_ENABLED

