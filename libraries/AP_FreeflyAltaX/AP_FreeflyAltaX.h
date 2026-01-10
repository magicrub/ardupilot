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

private:
    void thread();
    void send_init_msg();
    void check_timeouts_for_re_init();

    uint32_t esc_feedback_timestamp_ms[AP_FREEFLY_ALTA_X_ESC_COUNT_MAX];

    HAL_Semaphore sem;
};

class AP_FreeflyAltaX
{
public:
    AP_FreeflyAltaX() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FreeflyAltaX);

    void init();

private:
    AP_FreeflyAltaX_CAN *_driver;
};
#endif // AP_FREEFLY_ALTA_X_ENABLED
