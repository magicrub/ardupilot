#pragma once

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>

#ifndef AP_ADC_ADS1115_ENABLED
#define AP_ADC_ADS1115_ENABLED 0
#endif

#define ADS1115_CHANNELS_COUNT           3

#if AP_ADC_ADS1115_ENABLED

#include <AP_HAL/Semaphores.h>
#include <AP_HAL/I2CDevice.h>

struct adc_report_s
{
    uint8_t id;
    float data;
};

class AP_ADC_ADS1115
{
public:
    AP_ADC_ADS1115();
    ~AP_ADC_ADS1115();

    bool init();
    float read_by_channel(uint32_t id) const;
    size_t read(adc_report_s *report, size_t length) const;
    uint32_t get_last_sample_timestamp_ms() const { return _last_sample_timestamp_ms;}

    uint8_t get_channels_number() const
    {
        return _channels_number;
    }

private:
    static const uint8_t _channels_number;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint16_t            _gain;
    int                 _channel_to_read;
    adc_report_s        *_samples;
    uint32_t            _last_sample_timestamp_ms;

    void _update();
    bool _start_conversion(uint8_t channel);

    float _convert_register_data_to_mv(int16_t word) const;
};
#endif // AP_ADC_ADS1115_ENABLED
