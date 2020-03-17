#include "AP_Mount_UAVVision.h"
#include <AP_GPS/AP_GPS.h>
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
    }
}

// update mount position - should be called periodically
void AP_Mount_UAVVision::update()
{
    if (!_initialised) {
        return;
    }
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_UAVVision::send_mount_status(mavlink_channel_t chan)
{
    if (!_initialised) {
        return;
    }
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

    _port->write( SYNC1 );
    _port->write( SYNC2 );
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
}

void AP_Mount_UAVVision::handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet)
{
    const uint8_t size = packet.payload[2];
    const uint8_t cmd = packet.payload[3];
    const uint8_t* data = &packet.payload[4];

    send_command(cmd, data, size);
}


