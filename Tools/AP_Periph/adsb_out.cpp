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
  AP_Periph ADSB support. This is designed to talk to a Ping ADSB
  module over the UART
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_ADSB_OUT

#include <AP_SerialManager/AP_SerialManager.h>
#include <dronecan_msgs.h>

/*
  init ADSB Out support
 */
void AP_Periph_FW::adsb_out_init(void)
{
#if !HAL_GCS_ENABLED
    serial_manager.set_protocol_and_baud(g.adsb_port, AP_SerialManager::SerialProtocol_ADSB, g.adsb_baudrate);
    auto uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ADSB, 0);
    if (uart != nullptr) {
        uart->begin(AP_SerialManager::map_baudrate(g.adsb_baudrate), 256, 256);

#ifdef HAL_PERIPH_ENABLE_ADSB_OUT_SERIAL_OPTIONS
        uart->set_options(HAL_PERIPH_ENABLE_ADSB_OUT_SERIAL_OPTIONS);
#endif
    }
#endif
}

void AP_Periph_FW::adsb_out_update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - adsb_update_last_ms < 100) {
        return;
    }
    adsb_update_last_ms = now_ms;

    adsb_lib.update(); // 10 Hz update

    bool send_msg = false;

    if (adsb_last_status_msg_received_ms == 0 || now_ms - adsb_last_status_msg_received_ms > 1000) {
        adsb_last_status_msg_received_ms = now_ms;
        send_msg = true;

    } else {
        const uint32_t last_status_ms = adsb_lib.get_last_status_msg_received_ms();
        if (last_status_ms != 0 && adsb_last_status_msg_received_ms != last_status_ms) {
            // we've gotten a status from the HW
            adsb_last_status_msg_received_ms = last_status_ms;
            send_msg = true;
        }
    }
    
    if (send_msg) {
        // the adsb library driver wants to send a status. This is usually done via gcs().send_message()

        ardupilot_equipment_adsb_OutStatus msg_can {};
        mavlink_uavionix_adsb_out_status_t msg_mavlink = adsb_lib.get_status();

        msg_can.state = msg_mavlink.state;
        msg_can.squawk = msg_mavlink.squawk;
        msg_can.NIC_NACp = msg_mavlink.NIC_NACp;
        msg_can.boardTemp = msg_mavlink.boardTemp;
        msg_can.fault = msg_mavlink.fault;
        memcpy(msg_can.flight_id, msg_mavlink.flight_id, sizeof(msg_can.flight_id));

        uint8_t buffer[ARDUPILOT_EQUIPMENT_ADSB_OUTSTATUS_MAX_SIZE] {};
        uint16_t total_size = ardupilot_equipment_adsb_OutStatus_encode(&msg_can, buffer, !periph.canfdout());

        periph.canard_broadcast(ARDUPILOT_EQUIPMENT_ADSB_OUTSTATUS_SIGNATURE,
                                ARDUPILOT_EQUIPMENT_ADSB_OUTSTATUS_ID,
                                CANARD_TRANSFER_PRIORITY_LOW,
                                &buffer[0],
                                total_size);
    }
}
#endif // HAL_PERIPH_ENABLE_ADSB_OUT