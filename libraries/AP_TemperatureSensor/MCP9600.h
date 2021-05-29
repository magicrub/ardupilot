/*
 * I2C driver for Microchip MCP9600 digital thermocouple EMF to Temperature converter
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#define MCP9600_ADDR        0x60
#define MCP9600_ADDR_ALT    0x66

class MCP9600 {
public:

    bool init(uint8_t bus, uint8_t address = MCP9600_ADDR);
    float temperature(void) const { return _temperature; } // temperature in degrees C
    bool healthy(void) const { // do we have a valid temperature reading?
        return _healthy;
    }

    enum class ThermocoupleType : uint8_t {
        K = 0,
        J = 1,
        T = 2,
        N = 3,
        S = 4,
        E = 5,
        B = 6,
        R = 7,
    } ;

private:

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    ThermocoupleType _thermoType;
    uint32_t last_reading_ms;

    bool set_config(const ThermocoupleType type, const uint8_t filter) const;

    float _temperature; // degrees C
    bool _healthy; // we have a valid temperature reading to report
    void _timer(void); // update the temperature, called at 20Hz

    bool read_temperature(float &temperature);
};
