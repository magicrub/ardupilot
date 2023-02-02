#pragma once

#include <AP_Param/AP_Param.h>
#include "AP_BattMonitor_config.h"

class AP_BattMonitor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_BattMonitor_Params(void);

    /* Do not allow copies */
    AP_BattMonitor_Params(const AP_BattMonitor_Params &other) = delete;
    AP_BattMonitor_Params &operator=(const AP_BattMonitor_Params&) = delete;

    // low voltage sources (used for BATT_LOW_TYPE parameter)
    enum BattMonitor_LowVoltage_Source {
        BattMonitor_LowVoltageSource_Raw            = 0,
        BattMonitor_LowVoltageSource_SagCompensated = 1
    };
    enum class Options : uint8_t {
        Ignore_UAVCAN_SoC                   = (1U<<0),  // Ignore UAVCAN State-of-Charge (charge %) supplied value from the device and use the internally calculated one
        MPPT_Use_Input_Value                = (1U<<1),  // MPPT reports voltage and current from Input (usually solar panel) instead of the output
        Power_Off_At_Disarm                 = (1U<<2),  // Disabled when vehicle is disarmed, if HW supports it
        Power_On_At_Arm                     = (1U<<3),  // Enabled when vehicle is armed, if HW supports it
        Power_Off_At_Boot                   = (1U<<4),  // Disabled at startup (aka boot), if HW supports it
        Power_On_At_Boot                    = (1U<<5),  // Enabled at startup (aka boot), if HW supports it. If Power_Off_at_Boot is also set, the behavior is Power_Off_at_Boot
    };

    BattMonitor_LowVoltage_Source failsafe_voltage_source(void) const { return (enum BattMonitor_LowVoltage_Source)_failsafe_voltage_source.get(); }

    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _curr_amp_per_volt;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _curr_amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Int32 _pack_capacity;            /// battery pack capacity less reserve in mAh
    AP_Int32 _serial_number;            /// battery serial number, automatically filled in on SMBus batteries
    AP_Float _low_voltage;              /// voltage level used to trigger a low battery failsafe
    AP_Float _low_capacity;             /// capacity level used to trigger a low battery failsafe
    AP_Float _critical_voltage;         /// voltage level used to trigger a critical battery failsafe
    AP_Float _critical_capacity;        /// capacity level used to trigger a critical battery failsafe
    AP_Int32 _arming_minimum_capacity;  /// capacity level required to arm
    AP_Float _arming_minimum_voltage;   /// voltage level required to arm
    AP_Int32 _options;                  /// Options
    AP_Int16 _watt_max;                 /// max battery power allowed. Reduce max throttle to reduce current to satisfy t    his limit
    AP_Int8  _type;                     /// 0=disabled, 3=voltage only, 4=voltage and current
    AP_Int8  _volt_pin;                 /// board pin used to measure battery voltage
    AP_Int8  _curr_pin;                 /// board pin used to measure battery current
    AP_Int8  _low_voltage_timeout;      /// timeout in seconds before a low voltage event will be triggered
    AP_Int8  _i2c_bus;                  /// I2C bus number
    AP_Int8  _failsafe_voltage_source;  /// voltage type used for detection of low voltage event
    AP_Int8  _failsafe_low_action;      /// action to preform on a low battery failsafe
    AP_Int8  _failsafe_critical_action; /// action to preform on a critical battery failsafe

#if BATTERY_EKF_ENABLED
    // TODO: move all these to a new EKFBattery object so it has it's own memory space

    AP_Int8  _cell_count;               /// count of cells, like "4S" or "6S"
    struct {
        AP_Float SOH_init;

        // TODO: change to AP_Vector3f
        AP_Float R0_init;
        AP_Float R1_init;
        AP_Float R2_init;
        AP_Float RC1;
        AP_Float RC2;
        AP_Float I_sigma;
        AP_Float I_scale_sigma;
        AP_Float SOH_sigma;
        // TODO: change to AP_Vector3f
        AP_Float R0_sigma;
        AP_Float R1_sigma;
        AP_Float R2_sigma;
        AP_Float I_step_sigma;
        AP_Float V_sigma;
        AP_Float Q;
        AP_Float SOH_pnoise;
        // TODO: change to AP_Vector3f
        AP_Float R0_pnoise;
        AP_Float R1_pnoise;
        AP_Float R2_pnoise;
    } _ekf;
#endif

};
