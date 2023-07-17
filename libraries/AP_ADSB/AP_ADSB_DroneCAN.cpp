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

#if HAL_ADSB_DRONECAN_ENABLED
#include <AP_CANManager/AP_CANManager.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

void AP_ADSB_DroneCAN::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - run_state.last_packet_Transponder_Status_ms >= 10000 && run_state.last_packet_Transponder_Status_ms != 0) {
        _frontend.out_state.tx_status.fault |= UAVIONIX_ADSB_OUT_STATUS_FAULT_STATUS_MESSAGE_UNAVAIL;
    }

}

void AP_ADSB_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    // if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_out_config, ap_dronecan->get_driver_index()) == nullptr) {
    //     AP_BoardConfig::allocation_error("adsb_OutConfig_sub");
    // }
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_transceiver_health_report, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("adsb_TxcvrHealthReport_sub");
    }
    // if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_out_control, ap_dronecan->get_driver_index()) == nullptr) {
    //     AP_BoardConfig::allocation_error("adsb_OutConfig_sub");
    // }
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_out_status, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("adsb_OutStatus_sub");
    }
}

void AP_ADSB_DroneCAN::handle_out_config(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutConfig& msg_can)
{
    if (ap_dronecan == nullptr || !AP_ADSB_DroneCAN::is_adsb_droneCan_ready()) {
        return;
    }

    mavlink_uavionix_adsb_out_cfg_t msg_mavlink {};

    msg_mavlink.ICAO = msg_can.ICAO;
    memcpy(msg_mavlink.callsign, msg_can.callsign, sizeof(msg_mavlink.callsign));
    msg_mavlink.emitterType = msg_can.emitterType;
    msg_mavlink.aircraftSize = msg_can.aircraftSize;
    msg_mavlink.gpsOffsetLat = msg_can.gpsOffsetLat;
    msg_mavlink.gpsOffsetLon = msg_can.gpsOffsetLon;
    msg_mavlink.stallSpeed = msg_can.stallSpeed;
    msg_mavlink.rfSelect = msg_can.rfSelect;
    msg_mavlink.ICAO = msg_can.ICAO;
    
    AP::ADSB()->handle_out_cfg(msg_mavlink);
}

void AP_ADSB_DroneCAN::handle_transceiver_health_report(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_TransceiverHealthReport& msg_can)
{
    if (ap_dronecan == nullptr || !AP_ADSB_DroneCAN::is_adsb_droneCan_ready()) {
        return;
    }

    mavlink_uavionix_adsb_transceiver_health_report_t msg_mavlink {};

    msg_mavlink.rfHealth = msg_can.rfHealth;

    AP::ADSB()->handle_transceiver_report((mavlink_channel_t)0, msg_mavlink);
}

void AP_ADSB_DroneCAN::handle_out_control(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutControl& msg_can)
{
    if (ap_dronecan == nullptr || !AP_ADSB_DroneCAN::is_adsb_droneCan_ready()) {
        return;
    }

    mavlink_uavionix_adsb_out_control_t msg_mavlink {};

    msg_mavlink.state = msg_can.state;
    msg_mavlink.baroAltMSL = msg_can.baroAltMSL;
    msg_mavlink.squawk = msg_can.squawk;
    msg_mavlink.emergencyStatus = msg_can.emergencyStatus;
    memcpy(msg_mavlink.flight_id, msg_can.flight_id, sizeof(msg_mavlink.flight_id));
    msg_mavlink.x_bit = msg_can.x_bit;

    AP::ADSB()->handle_out_control(msg_mavlink);
}

void AP_ADSB_DroneCAN::handle_out_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const ardupilot_equipment_adsb_OutStatus& msg_can)
{
    if (ap_dronecan == nullptr || !AP_ADSB_DroneCAN::is_adsb_droneCan_ready()) {
        return;
    }

    mavlink_uavionix_adsb_out_status_t msg_mavlink {};

    msg_mavlink.state = msg_can.state;
    msg_mavlink.squawk = msg_can.squawk;
    msg_mavlink.NIC_NACp = msg_can.NIC_NACp;
    msg_mavlink.boardTemp = msg_can.boardTemp;
    msg_mavlink.fault = msg_can.fault;
    memcpy(msg_mavlink.flight_id, msg_can.flight_id, sizeof(msg_mavlink.flight_id));

    AP::ADSB()->handle_out_status(msg_mavlink);
}

bool AP_ADSB_DroneCAN::is_adsb_droneCan_ready()
{
    auto adsb = AP::ADSB();
    return (adsb != nullptr && adsb->enabled() && adsb->get_type(0) == AP_ADSB::Type::DroneCAN);
}

void AP_ADSB_DroneCAN::send_example()
{
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    uavcan_equipment_indication_BeepCommand msg;
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_DroneCAN *uavcan = AP_DroneCAN::get_dronecan(i);
        if (uavcan != nullptr && (AP::notify().get_buzzer_types() & AP_Notify::Notify_Buzz_UAVCAN)) {
            uavcan->set_buzzer_tone(frequency, _note_duration_us*1.0e-6);
            msg.frequency = frequency;
            msg.duration = _note_duration_us*1.0e-6;
            uavcan->buzzer.broadcast(msg);
        }
    }
}

#endif // HAL_ADSB_DRONECAN_ENABLED
