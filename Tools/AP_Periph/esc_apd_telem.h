/*
  ESC Telemetry for APD ESC.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#if AP_APD_ESC_ENABLED
#include <AP_ESC_Telem/AP_ESC_Telem.h>

class AP_APD_ESC_Telem : public AP_ESC_Telem_Backend {
public:
    AP_APD_ESC_Telem (AP_HAL::UARTDriver *_uart, const int8_t _num_poles, const uint8_t _esc_index);
    bool update();

    CLASS_NO_COPY(AP_APD_ESC_Telem);

private:
    AP_HAL::UARTDriver *uart;

    void handle_packet();

    union {
        struct PACKED {
            uint16_t voltage;
            uint16_t temperature;
            int16_t bus_current;
            uint16_t reserved0;
            uint32_t erpm;
            uint16_t input_duty;
            uint16_t motor_duty;
            uint16_t reserved1;
            uint16_t checksum; // 16 bit fletcher checksum
            uint16_t stop; // should always be 65535 on a valid packet
        } packet;
        uint8_t bytes[22];
    } received;

    static_assert(sizeof(received.packet) == sizeof(received.bytes), "The packet must be the same size as the raw buffer");

    uint8_t len;
    uint8_t esc_index;
    int8_t pole_count;

    float convert_temperature(uint16_t raw) const;
    void shift_buffer(void);
};

#endif // AP_APD_ESC_ENABLED
