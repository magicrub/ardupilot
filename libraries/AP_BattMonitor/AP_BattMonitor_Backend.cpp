/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

extern const AP_HAL::HAL& hal;

#if BATTERY_EKF_ENABLED
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
static float soc_ocv_x[] = {0.0, 0.005063014925373088, 0.01613838805970147, 0.02905964179104481, 0.04382680597014932, 0.060439850746268675, 0.07705289552238803, 0.09920364179104468, 0.1268920298507462, 0.15642635820895523, 0.19334423880597018, 0.2357997910447761, 0.2708717910447762, 0.2967142985074628, 0.3244027164179104, 0.34839934328358213, 0.3779336417910447, 0.4037761791044776, 0.4388481492537314, 0.462844776119403, 0.4868414029850746, 0.5182216119402985, 0.5551394925373134, 0.5920573731343284, 0.6289752537313433, 0.6695849253731343, 0.7194240895522388, 0.7581878507462687, 0.7932598507462687, 0.8283318507462687, 0.8615579402985074, 0.9058594029850746, 0.9446231641791045, 0.9815410447761194, 1.0};

static float soc_ocv_y[] = {2.5180000000000002, 2.6487000000000003, 2.75, 2.8668, 2.9681, 3.0693, 3.1550000000000002, 3.2406, 3.3107, 3.373, 3.4198, 3.4587, 3.4899, 3.5132, 3.5288, 3.5444, 3.5678, 3.5911, 3.6145, 3.6456, 3.6612, 3.7001, 3.7313, 3.7702, 3.8014, 3.8403, 3.8793, 3.9104, 3.9494000000000002, 3.9961, 4.027299999999999, 4.0584, 4.074, 4.1051, 4.158};

static BatteryChemistryModelLinearInterpolated chemistry_model(soc_ocv_x, soc_ocv_y, ARRAY_SIZE(soc_ocv_x));
#endif


/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params) :
        _mon(mon),
        _state(mon_state),
#if BATTERY_EKF_ENABLED
        _ekf(_ekf_params, chemistry_model),
#endif
        _params(params)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _params._pack_capacity - _state.consumed_mah;
    if ( _params._pack_capacity > 10 ) { // a very very small battery
        return MIN(MAX((100 * (mah_remaining) / _params._pack_capacity), 0), UINT8_MAX);
    } else {
        return 0;
    }
}

// update battery resistance estimate
// faster rates of change of the current and voltage readings cause faster updates to the resistance estimate
// the battery resistance is calculated by comparing the latest current and voltage readings to a low-pass filtered current and voltage
// high current steps are integrated into the resistance estimate by varying the time constant of the resistance filter
void AP_BattMonitor_Backend::update_resistance_estimate()
{
    // return immediately if no current
    if (!has_current() || !is_positive(_state.current_amps)) {
        return;
    }

    // update maximum current seen since startup and protect against divide by zero
    _current_max_amps = MAX(_current_max_amps, _state.current_amps);
    float current_delta = _state.current_amps - _current_filt_amps;
    if (is_zero(current_delta)) {
        return;
    }

    // update reference voltage and current
    if (_state.voltage > _resistance_voltage_ref) {
        _resistance_voltage_ref = _state.voltage;
        _resistance_current_ref = _state.current_amps;
    }

    // calculate time since last update
    uint32_t now = AP_HAL::millis();
    float loop_interval = (now - _resistance_timer_ms) / 1000.0f;
    _resistance_timer_ms = now;

    // estimate short-term resistance
    float filt_alpha = constrain_float(loop_interval/(loop_interval + AP_BATT_MONITOR_RES_EST_TC_1), 0.0f, 0.5f);
    float resistance_alpha = MIN(1, AP_BATT_MONITOR_RES_EST_TC_2*fabsf((_state.current_amps-_current_filt_amps)/_current_max_amps));
    float resistance_estimate = -(_state.voltage-_voltage_filt)/current_delta;
    if (is_positive(resistance_estimate)) {
        _state.resistance = _state.resistance*(1-resistance_alpha) + resistance_estimate*resistance_alpha;
    }

    // calculate maximum resistance
    if ((_resistance_voltage_ref > _state.voltage) && (_state.current_amps > _resistance_current_ref)) {
        float resistance_max = (_resistance_voltage_ref - _state.voltage) / (_state.current_amps - _resistance_current_ref);
        _state.resistance = MIN(_state.resistance, resistance_max);
    }

    // update the filtered voltage and currents
    _voltage_filt = _voltage_filt*(1-filt_alpha) + _state.voltage*filt_alpha;
    _current_filt_amps = _current_filt_amps*(1-filt_alpha) + _state.current_amps*filt_alpha;

    // update estimated voltage without sag
    _state.voltage_resting_estimate = _state.voltage + _state.current_amps * _state.resistance;
}

float AP_BattMonitor_Backend::voltage_resting_estimate() const
{
    // resting voltage should always be greater than or equal to the raw voltage
    return MAX(_state.voltage, _state.voltage_resting_estimate);
}

#if BATTERY_EKF_ENABLED
void AP_BattMonitor_Backend::run_ekf_battery_estimation(const uint8_t instance)
{
    if (!has_current() || !_state.healthy || !option_is_set(AP_BattMonitor_Params::Options::Enable_EKF_SoC_Estimation)) {
        return;
    }

    // get V and I
    float V = _state.voltage;
    float I = _state.current_amps;
    
    if (_params._cell_count > 0) {
        V /= _params._cell_count;
    }
    
    // get dt
    const uint32_t dt_us = _state.last_time_micros - _ekf_timestamp_last_us;
    if (dt_us == 0 || dt_us > 1e6f) {
        _ekf_timestamp_last_us = _state.last_time_micros;
        return;
    }
    const float dt = dt_us * 1E-6f;
    
    _ekf_timestamp_last_us = _state.last_time_micros;

    // get tempC
    float temp_C = 25;
    if (has_temperature()) {
        temp_C = _state.temperature;
    }

    bool V_in_range = V > _ekf.get_chemistry_model().min_valid_OCV(temp_C)*0.8 && V < _ekf.get_chemistry_model().max_valid_OCV(temp_C)*1.1;

    // update params at 1Hz and initialize if needed
    const uint32_t now_ms = AP_HAL::millis();
    if (!_ekf.initialized() || (now_ms - _ekf_param_update_ms > 1000)) {
        _ekf_param_update_ms = now_ms;
        
        _ekf_params =
        (BatteryEKF::Params){
        .SOH_init =     _params._ekf.SOH_init,
        .R0_init =      0,
        .R1_init =      0,
        .R2_init =      0,
        .RC1 =          _params._ekf.RC1,
        .RC2 =          _params._ekf.RC2,
        .I_sigma =      _params._ekf.I_sigma,
        .SOH_sigma =    _params._ekf.SOH_sigma,
        .R0_sigma =     6250.0f/_params._pack_capacity,
        .R1_sigma =     1250.0f/_params._pack_capacity,
        .R2_sigma =     1250.0f/_params._pack_capacity,
        .I_step_sigma = _params._pack_capacity*1e-3f,
        .V_sigma =      _params._ekf.V_sigma,
        .Q =            _params._pack_capacity*1e-3f*3600,
        .R0_pnoise =    12.5f/_params._pack_capacity,
        .R1_pnoise =    1.5f/_params._pack_capacity,
        .R2_pnoise =    1.5f/_params._pack_capacity,
        .SOC_pnoise =   _params._ekf.SOC_pnoise
        };

        if (!_ekf.initialized() && V_in_range) {
            _ekf.initialize(V, I, temp_C);
            hal.console->printf("Battery EKF init\n");
        }
    }
    
    float y = 0;
    float NIS = 0;

    if (_ekf.initialized()) {
        _ekf.predict(dt,I);

        if (V_in_range) {
            if (_ekf.update(V, I, temp_C, y, NIS)) {
                _ekf_last_successful_update_ms = now_ms;
            }
            
            if (now_ms - _ekf_last_successful_update_ms > 3000) {
                hal.console->printf("Battery EKF reset\n");
                _ekf_last_successful_update_ms = now_ms;
                _ekf.initialize(V, I, temp_C);
            }
        }
    }
    
#ifndef HAL_NO_GCS
    if (instance == 0 && _ekf.initialized() && now_ms-_ekf_last_print_ms > 2000) {
        const auto& x = _ekf.get_state();
        
        gcs().send_named_float("BatERem", _ekf.get_remaining_energy_J(temp_C)*_params._cell_count);
        gcs().send_named_float("BatERemSD", _ekf.get_remaining_energy_J_sigma(temp_C)*_params._cell_count);
        gcs().send_named_float("BatSOC", x(STATE_IDX_SOC));
        gcs().send_named_float("BatSOH", is_zero(x(STATE_IDX_SOH_INV)) ? 0 : 1/x(STATE_IDX_SOH_INV));
        
        _ekf_last_print_ms = now_ms;
    }
#endif

#if HAL_LOGGING_ENABLED

    if (_ekf.initialized()) {
        AP::logger().Write("BKF1", "TimeUS,Instance,dt,V,I,TempC,y,NIS,E,ESig",
            "QBffffffff",
            AP_HAL::micros64(),
            instance,
            (double)dt,
            (double)V,
            (double)I,
            (double)temp_C,
            (double)y,
            (double)NIS,
            (double)_ekf.get_remaining_energy_Wh(temp_C)*_params._cell_count,
            (double)_ekf.get_remaining_energy_Wh_sigma(temp_C)*_params._cell_count);
        
        const auto& x = _ekf.get_state();
        AP::logger().Write("BKF2", "TimeUS,Instance,x0,x1,x2,x3,x4,x5,x6",
            "QBfffffff",
            AP_HAL::micros64(),
            instance,
            (double)x(0),
            (double)x(1),
            (double)x(2),
            (double)x(3),
            (double)x(4),
            (double)x(5),
            (double)x(6));
        
        const auto& P = _ekf.get_covariance();
        AP::logger().Write("BKF3", "TimeUS,Instance,s0,s1,s2,s3,s4,s5,s6",
            "QBfffffff",
            AP_HAL::micros64(),
            instance,
            (double)sqrtf(P(0,0)),
            (double)sqrtf(P(1,1)),
            (double)sqrtf(P(2,2)),
            (double)sqrtf(P(3,3)),
            (double)sqrtf(P(4,4)),
            (double)sqrtf(P(5,5)),
            (double)sqrtf(P(6,6)));
    }
#endif // HAL_LOGGING_ENABLED
}
#endif

AP_BattMonitor::Failsafe AP_BattMonitor_Backend::update_failsafes(void)
{
    const uint32_t now = AP_HAL::millis();

    bool low_voltage, low_capacity, critical_voltage, critical_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity);

    if (critical_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.critical_voltage_start_ms == 0) {
            _state.critical_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.critical_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::Failsafe::Critical;
        }
    } else {
        // acceptable voltage so reset timer
        _state.critical_voltage_start_ms = 0;
    }

    if (critical_capacity) {
        return AP_BattMonitor::Failsafe::Critical;
    }

    if (low_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.low_voltage_start_ms == 0) {
            _state.low_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.low_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::Failsafe::Low;
        }
    } else {
        // acceptable voltage so reset timer
        _state.low_voltage_start_ms = 0;
    }

    if (low_capacity) {
        return AP_BattMonitor::Failsafe::Low;
    }

    // if we've gotten this far then battery is ok
    return AP_BattMonitor::Failsafe::None;
}

static bool update_check(size_t buflen, char *buffer, bool failed, const char *message)
{
    if (failed) {
        strncpy(buffer, message, buflen);
        return false;
    }
    return true;
}

bool AP_BattMonitor_Backend::arming_checks(char * buffer, size_t buflen) const
{
    bool low_voltage, low_capacity, critical_voltage, critical_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity);

    bool below_arming_voltage = is_positive(_params._arming_minimum_voltage) &&
                                (_state.voltage < _params._arming_minimum_voltage);
    bool below_arming_capacity = (_params._arming_minimum_capacity > 0) &&
                                 ((_params._pack_capacity - _state.consumed_mah) < _params._arming_minimum_capacity);
    bool fs_capacity_inversion = is_positive(_params._critical_capacity) &&
                                 is_positive(_params._low_capacity) &&
                                 (_params._low_capacity < _params._critical_capacity);
    bool fs_voltage_inversion = is_positive(_params._critical_voltage) &&
                                is_positive(_params._low_voltage) &&
                                (_params._low_voltage < _params._critical_voltage);

    bool result =      update_check(buflen, buffer, below_arming_voltage, "below minimum arming voltage");
    result = result && update_check(buflen, buffer, below_arming_capacity, "below minimum arming capacity");
    result = result && update_check(buflen, buffer, low_voltage,  "low voltage failsafe");
    result = result && update_check(buflen, buffer, low_capacity, "low capacity failsafe");
    result = result && update_check(buflen, buffer, critical_voltage, "critical voltage failsafe");
    result = result && update_check(buflen, buffer, critical_capacity, "critical capacity failsafe");
    result = result && update_check(buflen, buffer, fs_capacity_inversion, "capacity failsafe critical > low");
    result = result && update_check(buflen, buffer, fs_voltage_inversion, "voltage failsafe critical > low");

    return result;
}

void AP_BattMonitor_Backend::check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity) const
{
    // use voltage or sag compensated voltage
    float voltage_used;
    switch (_params.failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage_used = _state.voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage_used = voltage_resting_estimate();
            break;
    }

    // check critical battery levels
    if ((voltage_used > 0) && (_params._critical_voltage > 0) && (voltage_used < _params._critical_voltage)) {
        critical_voltage = true;
    } else {
        critical_voltage = false;
    }

    // check capacity failsafe if current monitoring is enabled
    if (has_current() && (_params._critical_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._critical_capacity)) {
        critical_capacity = true;
    } else {
        critical_capacity = false;
    }

    if ((voltage_used > 0) && (_params._low_voltage > 0) && (voltage_used < _params._low_voltage)) {
        low_voltage = true;
    } else {
        low_voltage = false;
    }

    // check capacity if current monitoring is enabled
    if (has_current() && (_params._low_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._low_capacity)) {
        low_capacity = true;
    } else {
        low_capacity = false;
    }
}

/*
  default implementation for reset_remaining(). This sets consumed_wh
  and consumed_mah based on the given percentage. Use percentage=100
  for a full battery
*/
bool AP_BattMonitor_Backend::reset_remaining(float percentage)
{
    percentage = constrain_float(percentage, 0, 100);
    const float used_proportion = (100.0f - percentage) * 0.01f;
    _state.consumed_mah = used_proportion * _params._pack_capacity;
    // without knowing the history we can't do consumed_wh
    // accurately. Best estimate is based on current voltage. This
    // will be good when resetting the battery to a value close to
    // full charge
    _state.consumed_wh = _state.consumed_mah * 0.001f * _state.voltage;

    // reset failsafe state for this backend
    _state.failsafe = update_failsafes();

    return true;
}

void AP_BattMonitor_Backend::set_bootup_powered_state()
{
    const uint32_t options = uint32_t(_params._options.get());
    const bool on_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::Power_On_At_Boot)) != 0;
    const bool off_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::Power_Off_At_Boot)) != 0;
    const bool force_update = true;

    if (off_at_boot) {
        set_powered_state(AP_BattMonitor::PoweredState::Powered_Off, force_update);
    } else if (on_at_boot) {
        set_powered_state(AP_BattMonitor::PoweredState::Powered_On, force_update);
    }
}

void AP_BattMonitor_Backend::set_powered_state(const AP_BattMonitor::PoweredState new_state, const bool force)
{
    if ((new_state != AP_BattMonitor::PoweredState::Powered_On) && (new_state != AP_BattMonitor::PoweredState::Powered_Off)) {
        return;
    } else if (_state.powered_state == new_state && !force) {
        return;
    }
    _state.powered_state = new_state;
    _state.powered_state_changed = true;
}

