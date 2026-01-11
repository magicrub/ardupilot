#pragma once

#include "AP_FreeflyAltaX_config.h"

#if AP_FREEFLY_ALTA_X_ENABLED
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_FreeflyAltaX_CAN : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    AP_FreeflyAltaX_CAN();
    void handle_frame(AP_HAL::CANFrame &frame) override;

    uint8_t esc_count_max() const { return AP_FREEFLY_ALTA_X_ESC_COUNT_MAX; }

    uint8_t esc_count_detected() const {
        uint8_t count = 0;
        for (uint8_t i=0; i<AP_FREEFLY_ALTA_X_ESC_COUNT_MAX; i++) {
            if (esc[i].timestamp_ms > 0) {
                count++;
            }
        }
        return count;
    };

    bool esc_healthy(const uint8_t index) const { return (index < AP_FREEFLY_ALTA_X_ESC_COUNT_MAX) ? esc[index].is_healthy : false; };

    bool esc_healthy_all() const {
        for (uint8_t i=0; i<AP_FREEFLY_ALTA_X_ESC_COUNT_MAX; i++) {
            if (esc[i].timestamp_ms > 0 && !esc[i].is_healthy) {
                return false;
            }
        }
        return true;
    };

private:
    void thread();
    void send_init_msg();
    void check_timeouts_and_re_init_as_needed();

    struct {
        uint32_t timestamp_ms;
        bool is_healthy;
    } esc[AP_FREEFLY_ALTA_X_ESC_COUNT_MAX];

    uint32_t init_last_ms;

    HAL_Semaphore sem;
};

class AP_FreeflyAltaX
{
public:
    AP_FreeflyAltaX() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FreeflyAltaX);

    void init();

    uint8_t esc_count_max() const { return (_driver == nullptr) ? 0 : _driver->esc_count_max(); };
    uint8_t esc_count_detected() const { return (_driver == nullptr) ? 0 : _driver->esc_count_detected(); };
    bool esc_healthy(const uint8_t index) const { return (_driver == nullptr) ? false : _driver->esc_healthy(index); };
    bool esc_healthy_all() const { return (_driver == nullptr) ? false : _driver->esc_healthy_all(); };

private:
    AP_FreeflyAltaX_CAN *_driver;
};
#endif // AP_FREEFLY_ALTA_X_ENABLED
