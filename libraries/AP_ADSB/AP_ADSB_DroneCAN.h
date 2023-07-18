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

#if AP_ADSB_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#include <dronecan_msgs.h>

class AP_ADSB_DroneCAN : public AP_ADSB_Backend {
public:
    using AP_ADSB_Backend::AP_ADSB_Backend;

    void update() override {};

    bool init() override;

    void send_out_config(const mavlink_uavionix_adsb_out_cfg_t &msg_mavlink);
    void send_out_control(const mavlink_uavionix_adsb_out_control_t &msg_mavlink);

private:

    static void handle_out_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutStatus& msg_can);
};

#endif // AP_ADSB_DRONECAN_ENABLED
