#include "AP_BattMonitor_MAVLink.h"

#if AP_BATTMON_MAVLINK_ENABLED
#include <GCS_MAVLink/GCS.h>

#define AP_BATTMONITOR_MAVLINK_TIMEOUT_MICROS        (AP_BATT_MONITOR_TIMEOUT * 1000UL)

void AP_BattMonitor_MAVLink::read()
{
    if (_interim_state.last_time_micros == 0) {
        return;
    }

    // timeout after a few seconds
    if ((AP_HAL::micros() - _interim_state.last_time_micros) > AP_BATTMONITOR_MAVLINK_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
        _has_temperature = false;
        _interim_state.last_time_micros = 0;
        return;
    }

    if (AP_HAL::millis() - _state.temperature_time > AP_BATT_MONITOR_TIMEOUT) {
        _has_temperature = false;
    }

    WITH_SEMAPHORE(_sem_battmon);

    // Copy over relevant states over to main state
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.healthy = _interim_state.healthy;
    _state.time_remaining = _interim_state.time_remaining;
    _state.has_time_remaining = _interim_state.has_time_remaining;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));
}

void AP_BattMonitor_MAVLink::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            {
            mavlink_battery_status_t msg_battery_status {};
            mavlink_msg_battery_status_decode(&msg, &msg_battery_status);
            if (allow_packet(msg.sysid, msg.compid, msg_battery_status.id)) {
                handle_message_battery_status(msg_battery_status);
            }
            }
            break;

        case MAVLINK_MSG_ID_SMART_BATTERY_INFO:
            {
            mavlink_smart_battery_info_t msg_smart_battery_info {};
            mavlink_msg_smart_battery_info_decode(&msg, &msg_smart_battery_info);
            if (allow_packet(msg.sysid, msg.compid, msg_smart_battery_info.id)) {
                handle_message_smart_battery_info(msg_smart_battery_info);
            }
            }
            break;

        default:
            // unhandled
            return;
    }
}

void AP_BattMonitor_MAVLink::handle_message_battery_status(const mavlink_battery_status_t &msg)
{
    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.last_time_micros = AP_HAL::micros();

    const uint8_t cell_count_mav = MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN;
    const uint8_t cell_count_mav_ext = MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN;
    const uint8_t cell_count = MIN(cell_count_mav + cell_count_mav_ext, AP_BATT_MONITOR_CELLS_MAX);

    uint32_t volt_sum_mV = 0;
    for (uint8_t i= 0; i<cell_count; i++) {
        // determine if we're grabbing the first main 8 cells or the extended 4 cells
        const uint16_t voltage = (i<cell_count_mav) ? msg.voltages[i] : msg.voltages_ext[i-cell_count_mav];

        if (voltage == UINT16_MAX) {
            _interim_state.cell_voltages.cells[i] = 0;
        } else {
            volt_sum_mV += voltage ;
            _interim_state.cell_voltages.cells[i] = voltage;
        }
    }
    
    _fault_bitmask = msg.fault_bitmask;
    _interim_state.voltage = volt_sum_mV * 0.001f;
    _interim_state.current_amps = msg.current_battery * 0.01f;
    _interim_state.consumed_mah = msg.current_consumed;
    _interim_state.consumed_wh = msg.energy_consumed * 0.027777778f;

    _has_temperature = (msg.temperature != INT16_MAX);
    if (_has_temperature) {
        _interim_state.temperature_time = AP_HAL::millis();
        _interim_state.temperature = msg.temperature * 0.01f;
    }

    _interim_state.time_remaining = labs(msg.time_remaining);
    _interim_state.has_time_remaining = msg.time_remaining != 0;

    _capacity_rem_pct = msg.battery_remaining;
    
    _interim_state.healthy = true;
}

void AP_BattMonitor_MAVLink::handle_message_smart_battery_info(const mavlink_smart_battery_info_t &msg)
{
    if (msg.id == 0) {
        return;
    }

    if (msg.capacity_full > 0) {
        _params._pack_capacity.set_and_notify(msg.capacity_full);
    }
    if (msg.discharge_minimum_voltage != UINT16_MAX) {
        _params._critical_voltage.set_default(msg.discharge_minimum_voltage);
    }

    // TODO: print device_name[] and serial_number[] but inhibit id=0 spamming multiple instances
}

bool AP_BattMonitor_MAVLink::capacity_remaining_pct(uint8_t &cap_remain_pct) const
{
    if (_capacity_rem_pct == INT8_MIN) {
        return false;
    }
    cap_remain_pct = _capacity_rem_pct;
    return true;
}

bool AP_BattMonitor_MAVLink::get_cycle_count(uint16_t &cycles) const
{
    if (_cycle_count == UINT16_MAX) {
        return false;
    }
    cycles = _cycle_count;
    return true;
}

bool AP_BattMonitor_MAVLink::allow_packet(const uint8_t sysid, const uint8_t compid, const uint8_t battery_id) const
{
    if (sysid != mavlink_system.sysid) {
        // this packet is intended for someone else
        return false;
    }

    if (compid != MAV_COMP_ID_BATTERY && compid != MAV_COMP_ID_BATTERY2) {
        // Not sure who sent this to.. We only accept battery packets from battery components
        return false;
    }

    // when serial number is negative, all battery IDs are accepted. Else, it must match
    if (_params._serial_number != -1 && _params._serial_number.get() != (int32_t)battery_id) {
        return false;
    }
    
    return true;
}

#endif // AP_BATTMON_MAVLINK_ENABLED
