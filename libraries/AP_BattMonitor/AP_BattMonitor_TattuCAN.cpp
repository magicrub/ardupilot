/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tom Pittenger
 */

#include "AP_BattMonitor_TattuCAN.h"

#if AP_BATT_MONITOR_TATTU_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_TattuCAN::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_TattuCAN, _curr_mult, 1.0),

    // @Param: PORT_LOCK
    // @DisplayName: DroneCAN Port is battery ID
    // @Description: Use the DroneCAN physical port number to distinguish between multiple CAN batteries. Example: BATT2_PORT_LOCK = 1 and BATT2_PORT_LOCK = 2 means BATT1 will only listen to batteries on CAN1 and BATT2 will only listen to batteries on CAN2. Param BATTx_SERIAL_NUM is still in effect. Use a value of 0 to disable.
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("PORT_LOCK", 31, AP_BattMonitor_TattuCAN, _port_must_match, 0),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_TattuCAN::AP_BattMonitor_TattuCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    CANSensor("Tattu"),
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
    register_driver(AP_CANManager::Driver_Type_Tattu);
}

void AP_BattMonitor_TattuCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (frame.dlc == 0) {
        // sanity check for if there's no payload
        return;
    }
    if (!match_port()) {
        // this packet came from a port that we want to ignore (multi-battery system)
        return;
    }

    const uint8_t payload_len = (frame.dlc - 1); // last payload byte is tail which is not _message data
    const uint8_t tail_byte = frame.data[frame.dlc-1];
    const bool start = (tail_byte & TAIL_BYTE_BITS_START_OF_TRANSFER) != 0;
    const bool end = (tail_byte & TAIL_BYTE_BITS_END_OF_TRANSFER) != 0;
    //const bool toggle = (tail_byte & TAIL_BYTE_BITS_TOGGLE) != 0;
    //const bool id = (tail_byte & TAIL_BYTE_TRANSFER_ID_MASK);


    if ((start && end) || (_buffer_offset == 0 && !start)) {
        // invalid or we're expecting a start and did't get one (re-sync)
        _message_resync++;
        return;

    } else if (start) {
        // start of a frame
        _buffer_offset = 0;
        _message_toggle_expected = 0;
        _crc = 0xFFFF;
        
    } else if ((_buffer_offset + frame.dlc) >= sizeof(_buffer)) {
        // buffer overflow, reset
        _buffer_offset = 0;
        _message_resync++;
        return;
    }

    for (uint8_t i=0; i<frame.dlc; i++) {
        _crc = crc_xmodem_update(_crc, frame.data[i]);
    }

    memcpy(&_buffer.data[_buffer_offset], frame.data, payload_len);
    _buffer_offset += payload_len;
    _message_toggle_expected = !_message_toggle_expected;

    if (!end) {
        return;
    }

    // message complete!
    _buffer_offset = 0;

//  ********************
//  ********************
// debug to test crc check
//  ********************
//  ********************
    if (_params._serial_number == 12345) {
        // force crc to pass
        _crc = _buffer.pkt.crc;
    }
//  ********************
//  ********************
//  ********************
//  ********************

    if (_crc != _buffer.pkt.crc) {
        // CRC fail
        return;
    }

    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.last_time_micros = AP_HAL::micros();
    _interim_state.voltage = _buffer.pkt.voltage_mV * 0.001f;
    _interim_state.current_amps = _curr_mult * _buffer.pkt.current_mA * -0.01f;

    _interim_state.temperature_time = AP_HAL::millis();
    _interim_state.temperature = _buffer.pkt.temperature_C;

    // A fully charged battery has been known to report a remaining capacity
    // slightly larger (~90mAh) than it's standard capacity, so lets constrain that
    // so we don't start off reporting consumed_mah as negative.
    const int32_t reported_consumed = _buffer.pkt.standard_capacity_mAh - _buffer.pkt.remaining_capacity_mAh;
    _interim_state.consumed_mah = constrain_int32(reported_consumed, 0,  _buffer.pkt.standard_capacity_mAh) * _curr_mult;

    for (uint8_t i=0; i<ARRAY_SIZE(_buffer.pkt.cell_voltage_mV); i++) {
        _interim_state.cell_voltages.cells[i] = _buffer.pkt.cell_voltage_mV[i];
    }

    _remaining_percent = _buffer.pkt.remaining_percent;
    _cycle_life = _buffer.pkt.cycle_life;

    _interim_state.healthy = true; // _buffer.pkt.health_status

    _have_received_a_msg = true;
    _message_count_good++;
}

// read - read the voltage and current
void AP_BattMonitor_TattuCAN::read()
{
    if (!_have_received_a_msg) {
        return;
    }

    // timeout after 5 seconds
    if (_interim_state.healthy && (AP_HAL::micros() - _interim_state.last_time_micros) > 5000000) {
        _interim_state.healthy = false;
    }

    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    //_state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    //_state.time_remaining = _interim_state.time_remaining;
    //_state.has_time_remaining = _interim_state.has_time_remaining;
    //_state.is_powering_off = _interim_state.is_powering_off;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_TattuCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!_have_received_a_msg) {
        return false;
    }

    percentage = _remaining_percent;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_TattuCAN::get_cycle_count(uint16_t &cycles) const
{
    if (!_have_received_a_msg) {
        return false;
    }

    cycles = _cycle_life;
    return true;
}

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_TattuCAN::get_mavlink_fault_bitmask() const
{
    // TODO: populate errors
    return 0;
}

#endif
