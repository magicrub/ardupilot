#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
//#include <AP_UAVCAN/AP_UAVCAN.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <mppt/Stream.hpp>
#include <mppt/OutputEnable.hpp>

#define LOG_TAG "BattMon"

#ifndef AP_BATTMONITOR_UAVCAN_MPPT_FAULT_ANNOUNCE
#define AP_BATTMONITOR_UAVCAN_MPPT_FAULT_ANNOUNCE 0
#endif

extern const AP_HAL::HAL& hal;

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);
UC_REGISTRY_BINDER(MpptStreamCb, mppt::Stream);

static void trampoline_handleOutputEnable(const uavcan::ServiceCallResult<mppt::OutputEnable>& resp);

static uavcan::ServiceClient<mppt::OutputEnable>* outputEnable_client[HAL_MAX_CAN_PROTOCOL_DRIVERS];

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

    const uint8_t driver_index = ap_uavcan->get_driver_index();
    outputEnable_client[driver_index] = new uavcan::ServiceClient<mppt::OutputEnable>(*node);
    if (outputEnable_client[driver_index] == nullptr) {
        AP_HAL::panic("AP_UAVCAN_DNA: outputEnable_client[%d]", driver_index);
    }
    int res = outputEnable_client[driver_index]->init();
    if (res < 0) {
        return;
    }
    outputEnable_client[driver_index]->setCallback(trampoline_handleOutputEnable);

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
#if AP_BATTMONITOR_UAVCAN_MPPT_FAULT_ANNOUNCE
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
#endif
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

    if (_is_mppt_packet_digital == true) {
        if (_mppt_powered_state_remote_ms != 0) {
            // at first, retry fast and then reduce retry attempts by a second, every second, but no slower tha n 1 minute.
            const uint32_t retry_interval_ms = constrain_int32(_mppt_set_attempt_retry_count, 1, 60) * 1000;
            if (AP_HAL::millis() - _mppt_powered_state_remote_ms >= retry_interval_ms) {
                _mppt_set_attempt_retry_count++;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: MPPT Retry %u", (unsigned)_instance+1, (unsigned)_mppt_set_attempt_retry_count);
                mppt_send_enable_output(_params._curr_pin != 0);
            }
        }

        // KHA HACK:
        // BATTx_CURR_PIN = current power state
        if ((_curr_pin_last != _params._curr_pin) &&
            (_params._curr_pin == 0 || _params._curr_pin == 1))
        {
            const AP_BattMonitor::PoweredState new_state = (_params._curr_pin == 0) ? AP_BattMonitor::PoweredState::Powered_Off : AP_BattMonitor::PoweredState::Powered_On;
            set_powered_state(new_state);
        }
    }
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

    _params._curr_pin.set_and_notify(enable?1:0);
    _curr_pin_last = _params._curr_pin;

    // set up a request /w a status callback
    mppt::OutputEnable::Request request;
    request.enable = enable ? true : false;
    request.disable = !request.enable;
    _mppt_powered_state = request.enable;

    _mppt_powered_state_remote_ms = AP_HAL::millis();
    const uint8_t driver_index = _ap_uavcan->get_driver_index();
    outputEnable_client[driver_index]->call(_node_id, request);
}

void trampoline_handleOutputEnable(const uavcan::ServiceCallResult<mppt::OutputEnable>& resp)
{
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_UAVCAN *uavcan = AP_UAVCAN::get_uavcan(i);
        if (uavcan == nullptr) {
            continue;
        }

        const uint8_t node_id = resp.getResponse().getSrcNodeID().get();
        AP_BattMonitor_UAVCAN* driver = AP_BattMonitor_UAVCAN::get_uavcan_backend(uavcan, node_id, node_id);
        if (driver == nullptr) {
            continue;
        }

        const auto &response = resp.getResponse();
        const uint8_t nodeId = response.getSrcNodeID().get();
        const bool enabled = response.enabled;
        driver->handle_OutputEnable_Response(nodeId, enabled);
    }
}

// void AP_BattMonitor_UAVCAN::handle_OutputEnable_Response(mppt::OutputEnable::Response response)
void AP_BattMonitor_UAVCAN::handle_OutputEnable_Response(const uint8_t nodeId, const bool enabled)
{
    if (nodeId != _node_id) {
        // this response is not from the node we are looking for
        return;
    }

    if (enabled == _mppt_powered_state) {
        _mppt_powered_state_remote_ms = 0;
        _mppt_set_attempt_retry_count = 0;
    }
}

#if AP_BATTMONITOR_UAVCAN_MPPT_FAULT_ANNOUNCE
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
#endif

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
