
#include "AP_DAC_DACx0501.h"

#if AP_DAC_DACx0501_ENABLED
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#define DACx0501_ADDRESS_ADDR_GND   0x48 // address pin low (GND)
#define DACx0501_ADDRESS_ADDR_VDD   0x49 // address pin high (VDD)
#define DACx0501_ADDRESS_ADDR_SDA   0x4A // address pin tied to SDA pin
#define DACx0501_ADDRESS_ADDR_SCL   0x4B // address pin tied to SCL pin

#ifndef DACx0501_I2C_ADDR
#define DACx0501_I2C_ADDR           DACx0501_ADDRESS_ADDR_GND
#endif

#ifndef DACx0501_I2C_BUS
#define DACx0501_I2C_BUS           0
#endif

// Registers
#define DACx0501_REG_NOOP       0x00     // no operation, this should never be used intentionally
#define DACx0501_REG_DEVID      0x01     // same as WHO_AM_I
#define DACx0501_REG_SYNC       0x02
#define DACx0501_REG_CONFIG     0x03
#define DACx0501_REG_GAIN       0x04
#define DACx0501_REG_TRIGGER    0x05
//#define DACx0501_REG_UNKNOWN  0x06
#define DACx0501_REG_STATUS     0x07
#define DACx0501_REG_DAC_DATA   0x08

#define DACx0501_DEVID          0x0015 // base ID. More bit sget set for other flavors of the chip

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_DAC_DACx0501::var_info[] = {

    // @Param: ADDR
    // @DisplayName: I2C address
    // @Description: I2C address
    AP_GROUPINFO("ADDR", 1, AP_DAC_DACx0501, params.i2c_address, DACx0501_I2C_ADDR),

    // @Param: BUS
    // @DisplayName: I2C bus
    // @Description: I2C bus
    AP_GROUPINFO("BUS", 2, AP_DAC_DACx0501, params.i2c_bus, DACx0501_I2C_BUS),

    // @Param: SRV_INDEX
    // @DisplayName: Servo Index
    // @Description: Servo Index to map to the DAC output
    AP_GROUPINFO("SRV_INDEX", 3, AP_DAC_DACx0501, params.servo_index, -1),

    // @Param: BIT_RES
    // @DisplayName: BIT_RES
    // @Description: BIT_RESolution of the DAC. This is decided by the part number
    // @Values: 12:12-bit, 14:14-bit, 16:16-bit
    AP_GROUPINFO("BIT_RES", 4, AP_DAC_DACx0501, params.bit_resolution, 12),

    // @Param: GAIN
    // @DisplayName: Gain
    // @Description: Gain
    // @Range: 0.01 100
    // @Incremenet: 0.01
    AP_GROUPINFO("GAIN", 5, AP_DAC_DACx0501, params.gain, 1),

    // @Param: INITIAL
    // @DisplayName: Initial Voltage
    // @Description: Initial Voltage at boot if there is no Servo input
    // @Range: 0.0 5.0
    // @Incremenet: 0.01
    AP_GROUPINFO("INITIAL", 6, AP_DAC_DACx0501, params.initial_voltage, 0),

    AP_GROUPEND
};

void AP_DAC_DACx0501::init()
{
    _dev = hal.i2c_mgr->get_device(params.i2c_bus, params.i2c_address);
    if (!_dev) {
        return;
    }
    target_voltage = params.initial_voltage.get();

    _dev->set_retries(10);
    // Set gain to unity
    uint8_t buf[3];
    buf[0] = DACx0501_REG_GAIN;
    buf[1] = 0;
    buf[2] = 0;
    UNUSED_RESULT(_dev->transfer(buf, sizeof(buf), nullptr, 0));
    _dev->set_retries(2);

    const float samples_per_second_per_channel_Hz = 50;
    const float interval_us = (1.0f/samples_per_second_per_channel_Hz) * AP_USEC_PER_SEC;
    _dev->register_periodic_callback(interval_us, FUNCTOR_BIND_MEMBER(&AP_DAC_DACx0501::thread, void));
}

void AP_DAC_DACx0501::thread()
{
    const uint16_t bit_resolution = constrain_int16(params.bit_resolution.get(), 12, 16);
    const uint16_t max_output = (1U << bit_resolution) - 1;
    float value_float = 0;

    const SRV_Channel *c = SRV_Channels::srv_channel(params.servo_index);
    if (c != nullptr && c->valid_function()) {
       value_float = linear_interpolate(0, max_output, c->get_output_pwm(), c->get_output_min(), c->get_output_max());
    } else {
        value_float = linear_interpolate(0, max_output, target_voltage, get_voltage_min(), get_voltage_max());
    }

    const uint16_t output_value = constrain_float(value_float * params.gain.get(), 0, UINT16_MAX);

    uint8_t buf[3];
    buf[0] = DACx0501_REG_DAC_DATA;
    buf[1] = HIGHBYTE(output_value);
    buf[2] = LOWBYTE(output_value);

    UNUSED_RESULT(_dev->transfer(buf, sizeof(buf), nullptr, 0));
}
#endif // AP_DAC_DACx0501_ENABLED
