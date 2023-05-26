/*
  ESC Telemetry for the APD HV Pro ESC

  Protocol is here: https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
 */
#include "esc_apd_telem.h"

#if AP_APD_ESC_ENABLED

#ifndef HAL_WITH_ESC_TELEM
#define HAL_WITH_ESC_TELEM 1
#elif HAL_WITH_ESC_TELEM == 0
#error "AP_APD_ESC_ENABLED requires HAL_WITH_ESC_TELEM" 
#endif

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/definitions.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

AP_APD_ESC_Telem::AP_APD_ESC_Telem(AP_HAL::UARTDriver *_uart, const int8_t _num_poles, const uint8_t _esc_index) :
    pole_count(_num_poles),
    esc_index(_esc_index),    
    uart(_uart) {
    uart->begin(115200);
}

bool AP_APD_ESC_Telem::update() {
    uint32_t n = MIN(uart->available(), 50);
    if (n == 0) {
        return false;
    }

    bool ret = false;

    while (n--) {
        uint8_t b = uart->read();
        received.bytes[len++] = b;

        // check the packet size first
        if ((size_t)len >= sizeof(received.packet)) {
            // we have a full packet, check the stop byte
            if (received.packet.stop == 65535) {
                // valid stop byte, check the CRC
                if (crc_fletcher16(received.bytes, 18) == received.packet.checksum) {
                    // valid packet, copy the data we need and reset length
                    handle_packet();
                    ret = true;
                    len = 0;
                } else {
                    // we have an invalid packet, shift it back a byte
                    shift_buffer();
                }
            } else {
                // invalid stop byte, we've lost sync, shift the packet by 1 byte
                shift_buffer();
            }
            
        }
    }
    return ret;
}

// handle a full received.bytes packet and update AP_ESC_Telem
void AP_APD_ESC_Telem::handle_packet()
{
    const int32_t rpm = le32toh(received.packet.erpm) / pole_count;
    update_rpm(esc_index, rpm);

    const float temperature_k = convert_temperature(le16toh(received.packet.temperature));
    TelemetryData t {
        .temperature_cdeg = (int16_t)(KELVIN_TO_C(temperature_k) * 100.0),
        .voltage = le16toh(received.packet.voltage) * 1e-2f,
        //.power_rating_pct = le16toh(received.packet.motor_duty) * 1e-2f,
        .current = le16toh(received.packet.bus_current) * (1 / 12.5f),
    };

    update_telem_data(esc_index, t,
        AP_ESC_Telem_Backend::TelemetryType::CURRENT
        | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
        | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
}

// shift the decode buffer left by 1 byte, and rewind the progress
void AP_APD_ESC_Telem::shift_buffer(void) {
    memmove(received.bytes, received.bytes + 1, sizeof(received.bytes) - 1);
    len--;
}

// convert the raw ESC temperature to a useful value (in Kelvin)
// based on the 1.1 example C code found here https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
float AP_APD_ESC_Telem::convert_temperature(uint16_t raw) const {
    const float series_resistor     = 10000;
    const float nominal_resistance  = 10000;
    const float nominal_temperature = 25;
    const float b_coefficent        = 3455;


    const float Rntc = series_resistor / ((4096 / float(raw)) - 1);

    float temperature = Rntc / nominal_resistance;          // (R/Ro)
    temperature = logf(temperature);                        // ln(R/Ro)
    temperature /= b_coefficent;                            // 1/B * ln(R/Ro)
    temperature += 1 / C_TO_KELVIN(nominal_temperature); // + (1/To)
    temperature = 1 / temperature;                          // invert

    // the example code rejected anything below 0C, or above 200C, the 200C clamp makes some sense, the below 0C is harder to accept
    return temperature;
}

#endif // AP_APD_ESC_ENABLED
