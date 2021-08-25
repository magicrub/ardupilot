#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <mppt/Stream.hpp>
#include <mppt/OutputEnable.hpp>

extern const AP_HAL::HAL& hal;
#define debug_bm_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { printf(fmt, ##args); }} while (0)

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);
UC_REGISTRY_BINDER(MpptStreamCb, mppt::Stream);

#ifdef HAL_NO_GCS
#define GCS_SEND_TEXT(severity, format, args...)
#else
#define GCS_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#endif

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb> *battinfo_listener;
    battinfo_listener = new uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb>(*node);
    // Backend Msg Handler
    const int battinfo_listener_res = battinfo_listener->start(BattInfoCb(ap_uavcan, &handle_battery_info_trampoline));
    if (battinfo_listener_res < 0) {
        AP_HAL::panic("UAVCAN BatteryInfo subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<mppt::Stream, MpptStreamCb> *mppt_stream_listener;
    mppt_stream_listener = new uavcan::Subscriber<mppt::Stream, MpptStreamCb>(*node);
    const int mppt_stream_listener_res = mppt_stream_listener->start(MpptStreamCb(ap_uavcan, &handle_mppt_stream_trampoline));
    if (mppt_stream_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mppt::Stream subscriber start problem\n\r");
        return;
    }
}

AP_BattMonitor_UAVCAN* AP_BattMonitor_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] == nullptr ||
            (AP::battery().get_type(i) != AP_BattMonitor_Params::BattMonitor_TYPE_UAVCAN_BatteryInfo && AP::battery().get_type(i) != AP_BattMonitor_Params::BattMonitor_TYPE_UAVCAN_MPPT_PacketDigital)) {
            continue;
        }
        AP_BattMonitor_UAVCAN* driver = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
        if (driver->_ap_uavcan == ap_uavcan && driver->_node_id == node_id && match_battery_id(i, battery_id)) {
            return driver;
        }
    }
    // find empty uavcan driver
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] != nullptr &&
            (AP::battery().get_type(i) == AP_BattMonitor_Params::BattMonitor_TYPE_UAVCAN_BatteryInfo || AP::battery().get_type(i) == AP_BattMonitor_Params::BattMonitor_TYPE_UAVCAN_MPPT_PacketDigital) &&
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_UAVCAN* batmon = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
            if(batmon->_ap_uavcan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_uavcan = ap_uavcan;
            batmon->_node_id = node_id;
            batmon->_instance = i;
            batmon->init();
            debug_bm_uavcan(2,
                            ap_uavcan->get_driver_index(),
                            "Registered BattMonitor Node %d on Bus %d\n",
                            node_id,
                            ap_uavcan->get_driver_index());
            return batmon;
        }
    }
    return nullptr;
}

void AP_BattMonitor_UAVCAN::handle_battery_info(const BattInfoCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);

    const bool temp_is_valid = !isnanf(cb.msg->temperature) && cb.msg->temperature > 0;

    // Temperature reported from battery in kelvin and stored internally in Celsius.
    const float temperature = cb.msg->temperature - C_TO_KELVIN;

    update_interim_state(cb.msg->voltage, cb.msg->current, temperature, temp_is_valid, cb.msg->state_of_charge_pct);
}

void AP_BattMonitor_UAVCAN::update_interim_state(const float voltage, const float current, const float temperature, const bool temp_is_valid, const uint8_t soc)
{
    WITH_SEMAPHORE(_sem_battmon);

    _soc = soc;

    if (temp_is_valid) {
        _interim_state.temperature = temperature;
        _interim_state.temperature_time = AP_HAL::millis();
    }

    _interim_state.voltage = voltage;
    _interim_state.current_amps = current;
    uint32_t tnow = AP_HAL::micros();
    uint32_t dt = tnow - _interim_state.last_time_micros;

    // update total current drawn since startup
    if (_interim_state.last_time_micros != 0 && dt < 2000000) {
        // .0002778 is 1/3600 (conversion to hours)
        float mah = (float) ((double) _interim_state.current_amps * (double) dt * (double) 0.0000002778f);
        _interim_state.consumed_mah += mah;
        _interim_state.consumed_wh  += 0.001f * mah * _interim_state.voltage;
    }

    // record time
    _interim_state.last_time_micros = tnow;

    _interim_state.healthy = true;
}

void AP_BattMonitor_UAVCAN::handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(cb);
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream(const MpptStreamCb &cb)
{
    const bool use_input_value = (uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::UAVCAN_MPPT_Use_Input_Value)) != 0;
    const float voltage = use_input_value ? cb.msg->input_voltage : cb.msg->output_voltage;
    const float current = use_input_value ? cb.msg->input_current : cb.msg->output_current;

    update_interim_state(voltage, current, cb.msg->temperature, true, 0);
    
    if (_mppt_fault_flags != cb.msg->fault_flags) {
        _mppt_fault_flags = cb.msg->fault_flags;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u MPPT %u: Fault 0x%2X", (unsigned)_instance, (unsigned)_node_id, (unsigned)_mppt_fault_flags);
    }
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MpptStreamCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, node_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_mppt_stream(cb);
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
    }
    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;

    if (_type == UAVCAN_MPPT_PACKETDIGITAL) {
       update_mppt();
    }
}

void AP_BattMonitor_UAVCAN::update_mppt()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_ap_uavcan == nullptr) {
        return;
    }
    if (_node == nullptr) {
        _node = _ap_uavcan->get_node();
        if (_node == nullptr) {
            return;
        }
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "update_mppt() _node has been assigned, node %u", (unsigned)_node_id);
        mppt_OutputEnable_bootup_delay = now_ms;
    }

    if (now_ms - mppt_OutputEnable_bootup_delay < 2000) {
        // if we send commands too soon at boot, the hw does not process it
        return;
    } else if (_behavior_last < 0) {
        // init
        _params._behavior.set_and_notify(0);
    } else if (_behavior_last == _params._behavior) {
        return;
    }

    _behavior_last = _params._behavior;
    const bool enable = (_behavior_last == 1);
    
    mppt::OutputEnable::Request request;
    request.enable = enable ? true : false;
    request.disable = !request.enable;

    uavcan::ServiceClient<mppt::OutputEnable> client(*_node);
    client.setCallback([](const uavcan::ServiceCallResult<mppt::OutputEnable>& handle_mppt_enable_output_response){});
    client.call(_node_id, request);
}


void AP_BattMonitor_UAVCAN::handle_mppt_enable_output_response(const uavcan::ServiceCallResult<mppt::OutputEnable>& response)
{
    //const mppt::OutputEnable::Response r = response.getResponse();
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "handle_mppt_enable_output_response() enabled=%u", r.enabled);
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_UAVCAN::capacity_remaining_pct() const
{
    switch(_type) {
    case UAVCAN_BATTERY_INFO:
        if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
            _soc > 100) {
            // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
            // the user can set the option to use current integration in the backend instead.
            return AP_BattMonitor_Backend::capacity_remaining_pct();
        }
        return _soc;

    case UAVCAN_MPPT_PACKETDIGITAL:
        return AP_BattMonitor_Backend::capacity_remaining_pct();
    }
    return 0;
}

#endif
