#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

#define AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS         5000000 // sensor becomes unhealthy if no successful readings for 5 seconds

class BattInfoCb;
class MpptStreamCb;
#include <mppt/OutputEnable.hpp>

class AP_BattMonitor_UAVCAN : public AP_BattMonitor_Backend
{
public:
    enum BattMonitor_UAVCAN_Type {
        UAVCAN_BATTERY_INFO = 0
    };

    enum class MPPT_FaultFlags : uint8_t {
        NONE                = 0,
        OVER_VOLTAGE        = (1U<<0),
        UNDER_VOLTAGE       = (1U<<1),
        OVER_CURRENT        = (1U<<2),
        OVER_TEMPERATURE    = (1U<<3),
    };

    /// Constructor
    AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params);

    void init() override {}

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
    uint8_t capacity_remaining_pct() const override;

    bool has_temperature() const override { return _has_temperature; }

    bool has_current() const override {
        return true;
    }

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_BattMonitor_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id);
    static void handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb);
    static void handle_mppt_stream_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MpptStreamCb &cb);
    void handle_OutputEnable_Response(const uint8_t nodeId, const bool enabled);

    void set_hardware_to_powered_state(const AP_BattMonitor::PoweredState desired_state) override;

private:
    void handle_battery_info(const BattInfoCb &cb);

    void update_interim_state(const float voltage, const float current, const float temperature, const uint8_t soc);

    static bool match_battery_id(uint8_t instance, uint8_t battery_id) {
        // when serial number is negative, all batteries are accepted. Else, it must match
        return (AP::battery().get_serial_number(instance) < 0) || (AP::battery().get_serial_number(instance) == (int32_t)battery_id);
    }

    void handle_mppt_stream(const MpptStreamCb &cb);
    void mppt_send_enable_output(const bool enable);
    static const char* mppt_fault_string(const MPPT_FaultFlags fault);
    void mppt_check_and_report_faults(const uint8_t flags);

    AP_BattMonitor::BattMonitor_State _interim_state;
    BattMonitor_UAVCAN_Type _type;

    HAL_Semaphore _sem_battmon;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    uint8_t _soc;
    bool _has_temperature;
    
    // needed for MPPT
    bool _is_mppt_packet_digital;   // true if this UAVCAN device is a Packet Digital MPPT
    uint8_t _mppt_fault_flags;
    bool _mppt_powered_state;             // true if the mppt is powered on, false if powered off
    uint32_t _mppt_powered_state_remote_ms; // timestamp of when request was sent.
    uint32_t _mppt_set_attempt_retry_count;
    
    uint8_t _instance;
    uavcan::Node<0> *_node;
    int8_t _curr_pin_last = -2;
};
