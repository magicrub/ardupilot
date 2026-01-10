#pragma once

#include "AP_FreeflyAltaX_config.h"

#if AP_Freefly_Alta_X_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_FreeflyAltaX_CAN : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    AP_FreeflyAltaX_CAN();
    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    void thread();
    void send_init_messages();
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
