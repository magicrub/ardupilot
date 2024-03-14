#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef AP_DAC_DACx0501_ENABLED
#define AP_DAC_DACx0501_ENABLED 0
#endif

#if AP_DAC_DACx0501_ENABLED
#include <AP_HAL/I2CDevice.h>
#include <AP_Param/AP_Param.h>

class AP_DAC_DACx0501
{
public:
    AP_DAC_DACx0501() {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_DAC_DACx0501);

    static const struct AP_Param::GroupInfo var_info[];

    void init();

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    struct {
        AP_Int8 i2c_address;
        AP_Int8 i2c_bus;
        AP_Int16 servo_index;
        AP_Int16 bit_resolution;
        AP_Float gain;
    } params;
    
    void thread();
};

#endif // AP_DAC_DACx0501_ENABLED
