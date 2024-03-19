#pragma once

#ifdef HAL_PERIPH_ENABLE_ADC

#include <AP_ADC/AP_ADC_ADS1115.h>

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

    struct {
        AP_Int32 send_rate;
    } params;

    AP_ADC_ADS1115 lib;
    adc_report_s _samples[ADS1115_CHANNELS_COUNT];
    uint32_t _last_update_ms;
    uint32_t _last_send_ms;
    uint32_t _last_sample_timestamp_ms;
    uint32_t _last_debug_ms;

};

#endif // HAL_PERIPH_ENABLE_ADC

