/*
  Trllium Serial controlled mount backend class
*/
#pragma once

#include "AP_Mount_Backend.h"

#ifndef AP_MOUNT_TRILLIUM_ENABLED
    #define AP_MOUNT_TRILLIUM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if AP_MOUNT_TRILLIUM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_Backend.h"

#include "Trillium_protocol/OrionPublicPacketShim.h"



// config
#define AP_MOUNT_TRILLIUM_REQUIRE_ACKS                     (uint8_t)0      // 0 = FALSE, 1 = TRUE



class AP_Mount_Trillium : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_Trillium(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return true; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override { _mode = mode; }

    // send_mount_status - called to allow mounts to send their status to GCS via MAVLink
    void send_mount_status(mavlink_channel_t chan) override;



private:

    void send_target_angles(float pitch_deg, float roll_deg, float yaw_deg);
    void send_target_angles(Vector3f angle, bool target_in_degrees);

    void read_incoming();

    void handle_packet(OrionPkt_t &packet);

    void init_hw();

    AP_HAL::UARTDriver *_port;

    uint32_t    _last_send_ms;

    uint32_t    _last_send_isFlying_ms;

    Vector3f    _angle_ef_target_rad;

    struct {
        uint16_t    step;
        bool        done : 1;
    } _booting;


    // keep the last _current_angle values
    Vector3f _current_angle_deg; // in degrees
    const char* _trilliumGcsHeader = "Trillium: ";


    // Trillium SDK
    OrionPkt_t _PktIn;
    //OrionPkt_t _PktOut;
    OrionRetractStatus_t _retract_status;
    OrionNetworkByteSettings_t _network_settings_current;
    GeolocateTelemetryCore_t _telemetry_core;
    OrionUartConfig_t _uart_config;
    StareStart_t _stare_start;

    const OrionNetworkByteSettings_t _network_settings_desired = {
        .Ip         = {172,  20, 114,   5},
        .Mask       = {172,  20, 117, 238},
        .Gateway    = {255, 255,   0,   0},
        .LowDelay   = 1,
        .Mtu        = 0,
    };

    OrionDiagnostics_t _diagnostics;
    OrionPerformance_t _performance;

    size_t OrionCommSend(const OrionPkt_t *pPkt);
    void requestOrionMessageByID(uint8_t id);
    const char* get_packet_name(uint8_t id);
    void SendGeopointCmd(const Location targetLoc, const Vector3f targetVelNed_vector, const float joystickRange, const geopointOptions options);

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SocketAPM sock{true};
    bool sock_connected;
#endif // HAL_BOARD_SITL
};
#endif // AP_MOUNT_TRILLIUM_ENABLED

