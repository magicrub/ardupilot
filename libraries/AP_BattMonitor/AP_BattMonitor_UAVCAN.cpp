#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <mppt/Stream.hpp>
#include <mppt/OutputEnable.hpp>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);
UC_REGISTRY_BINDER(MpptStreamCb, mppt::Stream);

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
    // Backend Msg Handler
    const int mppt_stream_listener_res = mppt_stream_listener->start(MpptStreamCb(ap_uavcan, &handle_mppt_stream_trampoline));
    if (mppt_stream_listener_res < 0) {
        AP_HAL::panic("UAVCAN Mppt::Stream subscriber start problem");
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
            AP::battery().get_type(i) != AP_BattMonitor::Type::UAVCAN_BatteryInfo) {
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
            AP::battery().get_type(i) == AP_BattMonitor::Type::UAVCAN_BatteryInfo &&
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_UAVCAN* batmon = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
            if(batmon->_ap_uavcan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_uavcan = ap_uavcan;
            batmon->_node_id = node_id;
            batmon->_instance = i;
            batmon->_node = ap_uavcan->get_node();
            batmon->init();
            AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
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
    update_interim_state(cb.msg->voltage, cb.msg->current, cb.msg->temperature, cb.msg->state_of_charge_pct); 
}

void AP_BattMonitor_UAVCAN::update_interim_state(const float voltage, const float current, const float temperature, const uint8_t soc)
{

    WITH_SEMAPHORE(_sem_battmon);
    _interim_state.voltage = voltage;
    _interim_state.current_amps = current;
    _soc = soc;

    if (!isnanf(temperature) && temperature > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = temperature - C_TO_KELVIN;
        _interim_state.temperature_time = AP_HAL::millis();
    }

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
    const bool use_input_value = (uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::MPPT_Use_Input_Value)) != 0;
    const float voltage = use_input_value ? cb.msg->input_voltage : cb.msg->output_voltage;
    const float current = use_input_value ? cb.msg->input_current : cb.msg->output_current;

    // use an invalid soc so we use the library calculated one
    const uint8_t soc = 101;

    // convert C to Kelvin
    const float temperature_K = isnanf(cb.msg->temperature) ? 0 : cb.msg->temperature + C_TO_KELVIN;

    update_interim_state(voltage, current, temperature_K, soc); 

    if (!_is_mppt_packet_digital) {
        _is_mppt_packet_digital = true;
        set_bootup_powered_state();
    }

    mppt_check_and_report_faults(cb.msg->fault_flags);
}

void AP_BattMonitor_UAVCAN::mppt_check_and_report_faults(const uint8_t flags)
{
    if (_mppt_fault_flags == flags) {
        // only report flags when they change
        return;
    }
    _mppt_fault_flags = flags;

    for (uint8_t fault_bit=0x01; fault_bit <= 0x08; fault_bit <<= 1) {
        // this loop is to generate multiple messages if there are multiple concurrent faults, but also run once if there are no faults
        if ((fault_bit & flags) != 0 || flags == 0) {
            const MPPT_FaultFlags err = (flags == 0) ? MPPT_FaultFlags::NONE : (MPPT_FaultFlags)fault_bit;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: Fault: %s", (unsigned)_instance+1, mppt_fault_string(err));
        }
        if (flags == 0) {
            return;
        }
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
    set_powered_state(_interim_state.powered_state);

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

void AP_BattMonitor_UAVCAN::set_hardware_to_powered_state(const AP_BattMonitor::PoweredState desired_state)
{
    if (desired_state != AP_BattMonitor::PoweredState::Powered_On && desired_state != AP_BattMonitor::PoweredState::Powered_Off) {
        return;
    }

    const bool power_on = (desired_state == AP_BattMonitor::PoweredState::Powered_On);
    if (_is_mppt_packet_digital) {
        mppt_send_enable_output(power_on);
    }
}

void AP_BattMonitor_UAVCAN::mppt_send_enable_output(const bool enable)
{
    if (_ap_uavcan == nullptr || _node == nullptr) {
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: Setting power to: %s", (unsigned)_instance+1, enable?"ON":"OFF");

    mppt::OutputEnable::Request request;
    request.enable = enable ? true : false;
    request.disable = !request.enable;

    uavcan::ServiceClient<mppt::OutputEnable> client(*_node);
    client.setCallback([](const uavcan::ServiceCallResult<mppt::OutputEnable>& handle_mppt_enable_output_response){});
    client.call(_node_id, request);
}

void AP_BattMonitor_UAVCAN::handle_mppt_enable_output_response(const uavcan::ServiceCallResult<mppt::OutputEnable>& response)
{
    // Not supported
}

// returns string description of MPPT fault bit. Only handles single bit faults
const char* AP_BattMonitor_UAVCAN::mppt_fault_string(const MPPT_FaultFlags fault)
{
    switch (fault) {
        case MPPT_FaultFlags::NONE:             return "None";
        case MPPT_FaultFlags::OVER_VOLTAGE:     return "Over-Voltage";
        case MPPT_FaultFlags::UNDER_VOLTAGE:    return "Under-Voltage";
        case MPPT_FaultFlags::OVER_CURRENT:     return "Over-Current";
        case MPPT_FaultFlags::OVER_TEMPERATURE: return "Over-Temp";
    }
    return "Unknown";
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_UAVCAN::capacity_remaining_pct() const
{
    if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
        _is_mppt_packet_digital ||
        _soc > 100) {
        // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
        // the user can set the option to use current integration in the backend instead.
        return AP_BattMonitor_Backend::capacity_remaining_pct();
    }
    return _soc;
}

#endif
