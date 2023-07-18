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

#include "AP_ADSB_DroneCAN.h"

#if AP_ADSB_DRONECAN_ENABLED
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>


static Canard::Publisher<ardupilot_equipment_adsb_OutConfig>* dc_out_config[HAL_MAX_CAN_PROTOCOL_DRIVERS];
static Canard::Publisher<ardupilot_equipment_adsb_OutControl>* dc_out_control[HAL_MAX_CAN_PROTOCOL_DRIVERS];

bool AP_ADSB_DroneCAN::init()
{
    const char* alloc_err_str = "ADSB: DroneCAN alloc failed";
    const uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(i);
        if (ap_dronecan == nullptr) {
            continue;
        }

        const uint8_t ap_dronecan_index = ap_dronecan->get_driver_index();

        dc_out_config[ap_dronecan_index] = new Canard::Publisher<ardupilot_equipment_adsb_OutConfig>(ap_dronecan->get_canard_iface());
        if (dc_out_config[ap_dronecan_index] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s", alloc_err_str);
            return false;
        }
        dc_out_config[ap_dronecan_index]->set_timeout_ms(20);
        dc_out_config[ap_dronecan_index]->set_priority(CANARD_TRANSFER_PRIORITY_LOW);

        dc_out_control[ap_dronecan_index] = new Canard::Publisher<ardupilot_equipment_adsb_OutControl>(ap_dronecan->get_canard_iface());
        if (dc_out_control[ap_dronecan_index] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "%s", alloc_err_str);
            return false;
        }
        dc_out_control[ap_dronecan_index]->set_timeout_ms(20);
        dc_out_control[ap_dronecan_index]->set_priority(CANARD_TRANSFER_PRIORITY_LOW);

        if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_out_status, ap_dronecan_index) == nullptr) {
            AP_BoardConfig::allocation_error("adsb_OutStatus_sub");
            return false;
        }
    }

    _frontend.status_msg_received(true);
    return true;
}

void AP_ADSB_DroneCAN::handle_out_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutStatus& msg_can)
{
    if (ap_dronecan == nullptr ) {
        return;
    }

    auto adsb = AP::ADSB();
    if (adsb == nullptr || !adsb->enabled() || adsb->init_failed() || adsb->get_type(0) != AP_ADSB::Type::DroneCAN) {
        return;
    }

    mavlink_uavionix_adsb_out_status_t msg_mavlink {};
    msg_mavlink.state = msg_can.state;
    msg_mavlink.squawk = msg_can.squawk;
    msg_mavlink.NIC_NACp = msg_can.NIC_NACp;
    msg_mavlink.boardTemp = msg_can.boardTemp;
    msg_mavlink.fault = msg_can.fault;
    memcpy(msg_mavlink.flight_id, msg_can.flight_id, sizeof(msg_mavlink.flight_id));

    // copy the packet to the local state
    memcpy(&adsb->out_state.tx_status, &msg_mavlink, sizeof(adsb->out_state.tx_status));

    adsb->status_msg_received(true);
}


void AP_ADSB_DroneCAN::send_out_config(const mavlink_uavionix_adsb_out_cfg_t &msg_mavlink)
{
    ardupilot_equipment_adsb_OutConfig msg_can {};

    msg_can.ICAO = msg_mavlink.ICAO;
    memcpy(msg_can.callsign, msg_mavlink.callsign, sizeof(msg_can.callsign));
    msg_can.emitterType = msg_mavlink.emitterType;
    msg_can.aircraftSize = msg_mavlink.aircraftSize;
    msg_can.gpsOffsetLat = msg_mavlink.gpsOffsetLat;
    msg_can.gpsOffsetLon = msg_mavlink.gpsOffsetLon;
    msg_can.stallSpeed = msg_mavlink.stallSpeed;
    msg_can.rfSelect = msg_mavlink.rfSelect;
    msg_can.ICAO = msg_mavlink.ICAO;
    
    
    for (uint8_t i = 0; i < ARRAY_SIZE(dc_out_config); i++) {
        if (dc_out_config[i] != nullptr) {
            dc_out_config[i]->broadcast(msg_can);
        }
    }
}

void AP_ADSB_DroneCAN::send_out_control(const mavlink_uavionix_adsb_out_control_t &msg_mavlink)
{
    ardupilot_equipment_adsb_OutControl msg_can {};

    msg_can.baroAltMSL = msg_mavlink.baroAltMSL;
    msg_can.squawk = msg_mavlink.squawk;
    msg_can.state = msg_mavlink.state;
    msg_can.emergencyStatus = msg_mavlink.emergencyStatus;
    memcpy(msg_can.flight_id, msg_mavlink.flight_id, sizeof(msg_can.flight_id));
    msg_can.x_bit = msg_mavlink.x_bit;

    for (uint8_t i = 0; i < ARRAY_SIZE(dc_out_control); i++) {
        if (dc_out_control[i] != nullptr) {
            dc_out_control[i]->broadcast(msg_can);
        }
    }

}
#endif // AP_ADSB_DRONECAN_ENABLED
