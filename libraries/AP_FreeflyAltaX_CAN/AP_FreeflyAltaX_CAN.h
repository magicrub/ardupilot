#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

class AP_FreeflyAltaX_CAN_driver : public CANSensor
#if HAL_WITH_ESC_TELEM
, public AP_ESC_Telem_Backend
#endif
{
public:
    AP_FreeflyAltaX_CAN_driver();

    void handle_frame(AP_HAL::CANFrame &frame) override;

private:
    void thread();
};

class AP_FreeflyAltaX_CAN
{
public:
    AP_FreeflyAltaX_CAN() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_FreeflyAltaX_CAN);

    void init();

private:
    AP_FreeflyAltaX_CAN_driver *_driver;
};

