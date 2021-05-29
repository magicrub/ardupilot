/*
 * I2C driver for Microchip MCP9600 digital thermocouple EMF to Temperature converter
 */

#include "MCP9600.h"

#include <utility>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

static const uint8_t MCP9600_CMD_HOT_JUNCT_TEMP     = 0x00;     // thermocoupler temp
static const uint8_t MCP9600_CMD_JUNCT_TEMP_DELTA   = 0x01;
static const uint8_t MCP9600_CMD_COLD_JUNCT_TEMP    = 0x02;     // ambient temp
static const uint8_t MCP9600_CMD_RAW_DATA_ADC       = 0x03;
static const uint8_t MCP9600_CMD_STATUS             = 0x04;
static const uint8_t MCP9600_CMD_SENSOR_CFG         = 0x05;
static const uint8_t MCP9600_CMD_DEVICE_CFG         = 0x06;
static const uint8_t MCP9600_CMD_DEVICE_ID_REV      = 0x20; // to fetch WHOAMI


static const uint8_t MCP9600_FILTER_COEF            = 0x07;    // avg 128 samples
static const uint8_t MCP9600_WHOAMI                 = 0x40;



bool MCP9600::init(uint8_t bus, uint8_t address)
{
    _dev = std::move(hal.i2c_mgr->get_device(bus, address));
    if (!_dev) {
        printf("MCP9600 device is null!");
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    uint8_t buf[2];
    if (!_dev->read_registers(MCP9600_CMD_DEVICE_ID_REV, buf, 2)) {
        printf("MCP9600 failed to get WHOAMI");
        return false;
    }
    if (buf[0] != MCP9600_WHOAMI) {
        printf("MCP9600 Got wrong WHOAMI: detected 0x%2X but expected 0x%2X", buf[0], MCP9600_WHOAMI);
        return false;
    }
    printf("MCP9600 Detected! Rev %u.%u", buf[1] >> 4, buf[1] & 0x0F);

    set_config(ThermocoupleType::K, 7);

    // lower retries for run
    _dev->set_retries(2);

    /* Request 2Hz update */
    _dev->register_periodic_callback(2 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&MCP9600::_timer, void));
    return true;
}

void MCP9600::_timer(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    if (read_temperature(_temperature)) {
        _healthy = true;
        last_reading_ms = now_ms;
    } else if (now_ms - last_reading_ms > 5000) {
        _healthy = false;
        _temperature = 0;
    }
}

bool MCP9600::set_config(const ThermocoupleType type, const uint8_t filter) const
{
    const uint8_t data = (uint8_t)type << 4 | (filter & 0x7);
    if (!_dev->write_register(MCP9600_CMD_SENSOR_CFG, data)) {
        return false;
    }
    return true;
}

bool MCP9600::read_temperature(float &temperature)
{
    uint8_t data[2];
    if (!_dev->read_registers(MCP9600_CMD_HOT_JUNCT_TEMP, data, 2)) {
        return false;
    }

    const uint8_t data_swapped[2] = {data[1], data[0]};
    temperature = (float)*((int16_t*) data_swapped) * 0.0625f;

    return true;
}

