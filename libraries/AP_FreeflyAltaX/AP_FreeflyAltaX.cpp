
#include "AP_FreeflyAltaX_config.h"

#if AP_Freefly_Alta_X_ENABLED
#include "AP_FreeflyAltaX.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

extern const AP_HAL::HAL& hal;


void AP_FreeflyAltaX::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        if (CANSensor::get_driver_type(i) == AP_CAN::Protocol::FreeflyAltaX) {
            _driver = NEW_NOTHROW AP_FreeflyAltaX_CAN();
            return;
        }
    }
}

AP_FreeflyAltaX_CAN::AP_FreeflyAltaX_CAN() : CANSensor("ALTA_X")
{
    register_driver(AP_CAN::Protocol::FreeflyAltaX);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_FreeflyAltaX_CAN::thread, void), "alta_X_can", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

void AP_FreeflyAltaX_CAN::send_init_messages()
{
    const uint8_t frames[][5] = {
        {0x4C, 0x00, 0x80, 0x00, 0x08},
        {0x55, 0x00, 0x80, 0x00, 0x08},
        {0x77, 0x00, 0x70, 0x00, 0x08}};

    for (uint8_t i=0; i<3; i++) {
        AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x010, frames[i], 5);
        write_frame(frame, 1000);
    }
}

void AP_FreeflyAltaX_CAN::thread()
{
    hal.scheduler->delay(1000);
    send_init_messages();

    while(true) {
        hal.scheduler->delay(100);

        for (uint8_t i=1; i<=4; i++) {
            uint8_t data[1] = {i};
            AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x02A, data, 1);
            write_frame(frame, 1000);
        }
    } // while
}


void AP_FreeflyAltaX_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (frame.isExtended()) {
        return;
    }
// RX    22:32:15.144685    NFD         04D    02 15 B3 01 00 00 00 00
// RX    22:32:15.144685    NFD         04E    02 00 00 00 00 00 00 00
// RX    22:32:15.144685    NFD         04D    03 15 AF 01 00 00 00 00
// RX    22:32:15.144685    NFD         04E    03 00 00 00 00 00 00 00
// RX    22:32:15.145685    NFD         04D    04 16 B1 01 00 00 00 00
// RX    22:32:15.145685    NFD         04E    04 00 00 00 00 00 00 00
// RX    22:32:15.207698    NFD         04D    01 15 B3 01 AA 02 43 00
// RX    22:32:15.207698    NFD         04E    51 07 B0 00 00 00 00 00

    const uint8_t idx = (frame.data[0] & 0x0F) - 1;

    TelemetryData t;
    if (frame.id == 0x04D) {
        t.voltage = float(UINT16_VALUE(frame.data[2], frame.data[3])) * 0.001f;
        update_telem_data(idx, t, AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);

    } else if (frame.id == 0x04E) {
        t.current = float(UINT16_VALUE(frame.data[1], frame.data[2])) * 0.001f;
        update_telem_data(idx, t, AP_ESC_Telem_Backend::TelemetryType::CURRENT);
    }
}
#endif // AP_Freefly_Alta_X_ENABLED
