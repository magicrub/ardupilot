/*
  UAV Vision Serial controlled mount backend class
*/
#pragma once

#include "AP_Mount.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_Backend.h"

class AP_Mount_UAVVision : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_UAVVision(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return true; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override { _state._mode = mode; }

    // send_mount_status - called to allow mounts to send their status to GCS via MAVLink
    void send_mount_status(mavlink_channel_t chan) override;

    void handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet) override;

private:


    static constexpr const uint8_t SYNC1 = 0x24;
    static constexpr const uint8_t SYNC2 = 0x40;

    void send_command(const uint8_t cmd, const uint8_t* data, const uint8_t size);

    AP_HAL::UARTDriver *_port;
    bool _initialised : 1;

};
