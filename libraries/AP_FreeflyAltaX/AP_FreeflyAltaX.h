#pragma once

#include "AP_FreeflyAltaX_config.h"

#if AP_Freefly_Alta_X_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>


#define AP_FREEFLY_ALTA_X_ESC_COUNT_MAX 4


class AP_FreeflyAltaX_CAN : public CANSensor, public AP_ESC_Telem_Backend
{
public:
    AP_FreeflyAltaX_CAN();
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    void thread();

    uint32_t timestamp_ms[AP_FREEFLY_ALTA_X_ESC_COUNT_MAX];

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
#endif // AP_Freefly_Alta_X_ENABLED
