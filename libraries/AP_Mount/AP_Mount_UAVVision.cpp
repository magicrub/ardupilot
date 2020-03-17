#include "AP_Mount_UAVVision.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

void AP_Mount_UAVVision::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for UAV Vision Gimbal protcol
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_UAVVision, 0);

    if (_port != nullptr) {
        _port->set_unbuffered_writes(true);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _initialised = true;

        set_mode((enum MAV_MOUNT_MODE)_state._default_mode.get());

        send_command(AP_MOUNT_UAVVISION_ID_ENABLE_STREAM_MODE, 2, AP_MOUNT_UAVVISION_CURRENT_POS_STREAM_RATE_HZ);
    }
}

// update mount position - should be called periodically
void AP_Mount_UAVVision::update()
{
    if (!_initialised) {
        return;
    }

    read_incoming(); // read the incoming messages from the gimbal

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    switch(get_mode()) {
            // move mount to a "retracted" position.  we do not implement a separate servo based retract mechanism
        case MAV_MOUNT_MODE_RETRACT:
            _angle_ef_target_rad = _state._retract_angles.get() * DEG_TO_RAD;
            if (_stow_status != AP_MOUNT_UAVVISION_STOW_STATE_EXIT_or_NOT_STOWED) {
                _stow_status = AP_MOUNT_UAVVISION_STOW_STATE_EXIT_or_NOT_STOWED;
                send_command(AP_MOUNT_UAVVISION_ID_STOW_MODE, _stow_status, 0);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            _angle_ef_target_rad = _state._neutral_angles.get() * DEG_TO_RAD;
            if (_stow_status != AP_MOUNT_UAVVISION_STOW_STATE_ENTER_or_DO_STOW) {
                _stow_status = AP_MOUNT_UAVVISION_STOW_STATE_ENTER_or_DO_STOW;
                send_command(AP_MOUNT_UAVVISION_ID_STOW_MODE, _stow_status, 0);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            resend_now = true;
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            update_targets_from_rc();
            resend_now = true;
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if (calc_angle_to_sysid_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }

    // resend target angles at least once per second
    resend_now = resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_UAVVISION_SERIAL_MINIMUM_INTERVAL_MS);

    if (resend_now) {
        send_target_angles(_angle_ef_target_rad, false);
    }

    if (_stab_pan != _state._stab_pan || _stab_tilt != _state._stab_tilt) {
        _stab_pan = _state._stab_pan;
        _stab_tilt = _state._stab_tilt;
        send_command(AP_MOUNT_UAVVISION_ID_ENABLE_GYRO_STABILISATION, _stab_pan, _stab_tilt);
    }

}
void AP_Mount_UAVVision::send_target_angles(Vector3f angle, bool target_in_degrees)
{
    // convert to degrees if necessary
    Vector3f target_deg = angle;
    if (!target_in_degrees) {
        target_deg *= RAD_TO_DEG;
    }
    send_target_angles(target_deg.x, target_deg.y, target_deg.z);
}

// send_target_angles
void AP_Mount_UAVVision::send_target_angles(float pitch_deg, float roll_deg, float yaw_deg)
{
    // datasheet section 3.2.1
    // encode float degrees to encoded uint16_t derived by U16_VALUE = angle * (32768 / 360)
    const float degree_to_encoded_U16 = 91.02222;
    const uint16_t pitch = pitch_deg * degree_to_encoded_U16;
    const uint16_t yaw =   yaw_deg * degree_to_encoded_U16;

    send_command(AP_MOUNT_UAVVISION_ID_SET_PAN_TILT_POSITION,
            yaw >> 8,
            yaw,
            pitch >> 8,
            pitch);
}

/*
 * detect and read the header of the incoming message from the gimbal
 */
void AP_Mount_UAVVision::read_incoming()
{
    int16_t num_available = _port->available();

    while (num_available-- > 0) {        // Process bytes received
        const uint8_t data = _port->read();

        switch (_rx_step) {
        case PACKET_FORMAT::SYNC1_START:
            if (data == AP_MOUNT_UAVVISION_SYNC1) {
                _rx_step = PACKET_FORMAT::SYNC2;
                _rx_payload_index = 0;
            }
            break;

        case PACKET_FORMAT::SYNC2:
            if (data == AP_MOUNT_UAVVISION_SYNC2) {
                _rx_step = PACKET_FORMAT::SIZE;
            } else {
                _rx_step = PACKET_FORMAT::SYNC1_START;
            }
            break;

        case PACKET_FORMAT::SIZE:
            _rx_payload_size = data;
            _rx_step = PACKET_FORMAT::ID;
            // don't sanity check the size yet. Wait until we get the id so we can notify GCS use useful feedback
            break;

        case PACKET_FORMAT::ID:
            _rx_id = data;
            _rx_sum = data;
            if (_rx_payload_size == 0) {
                _rx_step = PACKET_FORMAT::CHECKSUM;
            } else if (_rx_payload_size < AP_MOUNT_UAVVISION_SERIAL_LARGEST_RX_PAYLOAD_SIZE) {
                _rx_step = PACKET_FORMAT::PAYLOAD;
            } else {
                _rx_step = PACKET_FORMAT::SYNC1_START;
                num_available -= _rx_payload_size + 1; // drop the rest of the packet in case we've queued it all
                // we just found a packet that our buffer is too small for. Notify GCS so we can bump up the hard-coded buffer size?
                gcs().send_text(MAV_SEVERITY_ERROR, "UAV Vision buffer too small for id %d", _rx_id);
            }
            break;

        case PACKET_FORMAT::PAYLOAD:
            _rx_payload[_rx_payload_index++] = data;
            _rx_sum += data;
            if (_rx_payload_index >= _rx_payload_size) {
                _rx_step = PACKET_FORMAT::CHECKSUM;
            }
            break;

        case PACKET_FORMAT::CHECKSUM:
            _rx_step = PACKET_FORMAT::SYNC1_START;
            if (_rx_sum == data) {
                decode_packet();
            }
            break;
        }
    }
}

void AP_Mount_UAVVision::decode_packet()
{
    const float position_scaler = 0.0109863; // 360 / 32768 per datasheet

    switch (_rx_id) {
    case AP_MOUNT_UAVVISION_ID_CURRENT_POSITION_AND_RATE:
        _current_angle_deg.x = (float)((int16_t)_rx_payload[0] * 256 + _rx_payload[1]) * position_scaler;   // pan
        _current_angle_deg.y = (float)((int16_t)_rx_payload[2] * 256 + _rx_payload[3]) * position_scaler;   // tilt
        break;
    case AP_MOUNT_UAVVISION_ID_ACK:
        //_rx_payload[0]; // Identifier of packet being acknowledged
        break;
    }

}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_UAVVision::send_mount_status(mavlink_channel_t chan)
{
    if (!_initialised) {
        return;
    }

    // return target angles as gimbal's actual attitude.
    mavlink_msg_mount_status_send(chan, 0, 0, _current_angle_deg.y, _current_angle_deg.x, _current_angle_deg.z);
}


/*
 send a command to the UAV Vision Serial API
*/
void AP_Mount_UAVVision::send_command(const uint8_t cmd, const uint8_t* data, const uint8_t size)
{
    if (_port->txspace() < (size + 5U)) {
        return;
    }
    uint8_t checksum = 0;

    _port->write( AP_MOUNT_UAVVISION_SYNC1 );
    _port->write( AP_MOUNT_UAVVISION_SYNC2 );
    _port->write( size );  // write body size
    _port->write( cmd );  // write packet identifier

    // per datasheet section 2.3 on page "2 of 61" which is pdf page 11:
    // "Sum the identifier byte and all bytes in the data field"
    checksum += cmd;

    for (uint8_t i = 0;  i != size ; i++) {
        checksum += data[i];
        _port->write( data[i] );
    }
    _port->write(checksum);

    // store time of send
    _last_send = AP_HAL::millis();
}

void AP_Mount_UAVVision::handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet)
{
    const uint8_t size = packet.payload[2];
    const uint8_t cmd = packet.payload[3];
    const uint8_t* data = &packet.payload[4];

    send_command(cmd, data, size);
}


