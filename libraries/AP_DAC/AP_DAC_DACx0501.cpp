
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
    // @DisplayName: SRV_INDEX
    // @Description: SRV_INDEX
    AP_GROUPINFO("SRV_INDEX", 3, AP_DAC_DACx0501, params.servo_index, -1),

    // @Param: BIT_RES
    // @DisplayName: BIT_RES
    // @Description: BIT_RESolution of the DAC.
    // @Values: 12:12-bit, 14:14-bit, 16:16-bit
    AP_GROUPINFO("BIT_RES", 4, AP_DAC_DACx0501, params.bit_resolution, 12),

    // @Param: GAIN
    // @DisplayName: GAIN
    // @Description: GAIN
    // @Range: 0.01 100
    // @Incremenet: 0.01
    AP_GROUPINFO("GAIN", 5, AP_DAC_DACx0501, params.gain, 1),

    AP_GROUPEND
};

void AP_DAC_DACx0501::init()
{
    _dev = hal.i2c_mgr->get_device(params.i2c_bus, params.i2c_address);
    if (!_dev) {
        return;
    }

    _dev->set_retries(3);
    _dev->register_periodic_callback(20 * 1000, FUNCTOR_BIND_MEMBER(&AP_DAC_DACx0501::thread, void));
}

void AP_DAC_DACx0501::thread()
{
    const SRV_Channel *c = SRV_Channels::srv_channel(params.servo_index);
    if (c == nullptr || !c->valid_function()) {
        return;
    }

    const uint16_t bit_resolution = constrain_int16(params.bit_resolution.get(), 12, 16);
    const uint16_t max_output = (1U << bit_resolution) - 1;

    uint16_t value = linear_interpolate(0, max_output, c->get_output_pwm(), c->get_output_min(), c->get_output_max());
    value *= MAX(0.01f, params.gain.get());

    uint8_t buf[3];
    buf[0] = DACx0501_REG_DAC_DATA;
    buf[1] = HIGHBYTE(value);
    buf[2] = LOWBYTE(value);

    UNUSED_RESULT(_dev->transfer(buf, sizeof(buf), nullptr, 0));
}
#endif // AP_DAC_DACx0501_ENABLED
