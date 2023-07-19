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

void AP_ADSB_DroneCAN::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_frontend.out_state.last_config_ms == 0 || now_ms - _frontend.out_state.last_config_ms >= 1000) {
        _frontend.out_state.last_config_ms = now_ms;
        send_out_config();
    }

    if (_frontend.out_state.last_control_ms == 0 || now_ms - _frontend.out_state.last_control_ms >= 1000) {
        _frontend.out_state.last_control_ms = now_ms;
        send_out_control();
    }

}

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


void AP_ADSB_DroneCAN::send_out_config()
{
    ardupilot_equipment_adsb_OutConfig msg_can {};

    msg_can.ICAO = _frontend.out_state.cfg.ICAO_id;
    msg_can.emitterType = _frontend.out_state.cfg.emitterType;
    msg_can.aircraftSize = _frontend.out_state.cfg.lengthWidth;
    msg_can.gpsOffsetLat = _frontend.out_state.cfg.gpsOffsetLat;
    msg_can.gpsOffsetLon = _frontend.out_state.cfg.gpsOffsetLon;
    msg_can.rfSelect = _frontend.out_state.cfg.rfSelect;
    msg_can.stallSpeed = _frontend.out_state.cfg.stall_speed_cm;
    memcpy(msg_can.callsign, _frontend.out_state.cfg.callsign, sizeof(msg_can.callsign));
    
    for (uint8_t i = 0; i < ARRAY_SIZE(dc_out_config); i++) {
        if (dc_out_config[i] != nullptr) {
            dc_out_config[i]->broadcast(msg_can);
        }
    }
}

void AP_ADSB_DroneCAN::send_out_control()
{
    ardupilot_equipment_adsb_OutControl msg_can {};

    msg_can.state |= (_frontend.out_state.ctrl.baroCrossChecked ? UAVIONIX_ADSB_OUT_CONTROL_STATE_EXTERNAL_BARO_CROSSCHECKED : 0);
    msg_can.state |= (_frontend.out_state.ctrl.airGroundState ? UAVIONIX_ADSB_OUT_CONTROL_STATE_ON_GROUND : 0);
    msg_can.state |= (_frontend.out_state.ctrl.identActive ? UAVIONIX_ADSB_OUT_CONTROL_STATE_IDENT_BUTTON_ACTIVE : 0);
    msg_can.state |= (_frontend.out_state.ctrl.modeAEnabled ? UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_A_ENABLED : 0);
    msg_can.state |= (_frontend.out_state.ctrl.modeCEnabled ? UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_C_ENABLED : 0);
    msg_can.state |= (_frontend.out_state.ctrl.modeSEnabled ? UAVIONIX_ADSB_OUT_CONTROL_STATE_MODE_S_ENABLED : 0);
    msg_can.state |= (_frontend.out_state.ctrl.es1090TxEnabled ? UAVIONIX_ADSB_OUT_CONTROL_STATE_1090ES_TX_ENABLED : 0);

    msg_can.baroAltMSL = _frontend.out_state.ctrl.externalBaroAltitude_mm;
    msg_can.squawk = _frontend.out_state.ctrl.squawkCode;
    msg_can.emergencyStatus = _frontend.out_state.ctrl.emergencyState;
    memcpy(msg_can.flight_id, _frontend.out_state.ctrl.callsign, sizeof(msg_can.flight_id));
    msg_can.x_bit = _frontend.out_state.ctrl.x_bit;


    for (uint8_t i = 0; i < ARRAY_SIZE(dc_out_control); i++) {
        if (dc_out_control[i] != nullptr) {
            dc_out_control[i]->broadcast(msg_can);
        }
    }

}
#endif // AP_ADSB_DRONECAN_ENABLED
