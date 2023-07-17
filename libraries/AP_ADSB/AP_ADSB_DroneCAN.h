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

#if HAL_ADSB_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_ADSB_DroneCAN : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    void update() override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);

    static void handle_out_config(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutConfig& msg_can);
    static void handle_transceiver_health_report(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_TransceiverHealthReport& msg_can);
    static void handle_out_control(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutControl& msg_can);
    static void handle_out_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutStatus& msg_can);

private:
    static bool is_adsb_droneCan_ready();

    struct {
        uint32_t last_packet_Transponder_Status_ms;
    } run_state;

};

#endif // HAL_ADSB_DRONECAN_ENABLED
