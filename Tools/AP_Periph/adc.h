#pragma once

#ifdef HAL_PERIPH_ENABLE_ADC

#include <AP_ADC/AP_ADC_ADS1115.h>
#include <AC_PID/AC_PID.h>

class Periph_ADC {
public:
    friend class AP_Periph_FW;

    Periph_ADC() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    void init();
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:

    AC_PID pid {
        // Initial Conditions
        AC_PID::Defaults{
            .p         = 1,
            .i         = 0,
            .d         = 0,
            .ff        = 0.0f,
            .imax      = 1000,
            .filt_T_hz = 50,
            .filt_E_hz = 0.0f,
            .filt_D_hz = 50,
            .srmax     = 0,
            .srtau     = 1.0
        }
    };


    struct {
        AP_Int32 send_rate;
        AP_Int16 servo_index;
    } params;

    AP_ADC_ADS1115 adc_lib;
    uint32_t _last_send_ms;
    uint32_t _last_sample_timestamp_ms;
    uint32_t _last_debug_ms;
    uint32_t _last_pid_update_ms;

};

#endif // HAL_PERIPH_ENABLE_ADC

