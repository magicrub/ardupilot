
#include "AP_FreeflyAltaX_config.h"

#if AP_FREEFLY_ALTA_X_ENABLED
#include "AP_FreeflyAltaX.h"

extern const AP_HAL::HAL& hal;

#define AP_FREEFLY_ALTA_X_CAN_ESC_FEEDBACK_TIMEOUT_MS 1000
#define AP_FREEFLY_ALTA_X_CAN_ESC_REINIT_INTERVAL_MS 5000
#define AP_FREEFLY_ALTA_X_CAN_ESC_SEND_INTERVAL_MS 100 // 50 Hz

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
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_FreeflyAltaX_CAN::thread, void), "alta_X_can", 1024, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

void AP_FreeflyAltaX_CAN::send_init_msg()
{
    const uint8_t init_frames[][5] = {
        {0x4C, 0x00, 0x80, 0x00, 0x08},
        {0x55, 0x00, 0x80, 0x00, 0x08},
        {0x77, 0x00, 0x70, 0x00, 0x08}};

    for (uint8_t i=0; i<3; i++) {
        AP_HAL::CANFrame frame = AP_HAL::CANFrame(0x010, init_frames[i], 5);
        write_frame(frame, 1000);
    }
}

void AP_FreeflyAltaX_CAN::check_timeouts_and_re_init_as_needed()
{
    bool send_init = false;
    const uint32_t now_ms = AP_HAL::millis();

    {
        WITH_SEMAPHORE(sem);
        for (uint8_t i=0; i<ARRAY_SIZE(esc); i++) {
            if (now_ms - esc[i].timestamp_ms > AP_FREEFLY_ALTA_X_CAN_ESC_FEEDBACK_TIMEOUT_MS) {
                esc[i].is_healthy = false;
                send_init = true;
            }
        }
    }

    if (send_init && now_ms - init_last_ms > AP_FREEFLY_ALTA_X_CAN_ESC_REINIT_INTERVAL_MS) {
        send_init_msg();
        init_last_ms = now_ms;
    }
}

void AP_FreeflyAltaX_CAN::thread()
{
    while(true) {
        hal.scheduler->delay(AP_FREEFLY_ALTA_X_CAN_ESC_SEND_INTERVAL_MS);

        // Check for rx timeouts to trigger a re-init at runtime. At boot
        // all timestamps are zero so this triggers the bootup init
        check_timeouts_and_re_init_as_needed();

        // Send get-feedback msg to each ESCs
        for (uint8_t i=1; i<=ARRAY_SIZE(esc); i++) {
            const uint8_t data[1] = {i}; // NOTE: value is 1-indexed motor index
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

// Example frames:
// RX    22:32:15.144685    NFD         04D    02 15 B3 01 00 00 00 00
// RX    22:32:15.144685    NFD         04E    02 00 00 00 00 00 00 00
// RX    22:32:15.144685    NFD         04D    03 15 AF 01 00 00 00 00
// RX    22:32:15.144685    NFD         04E    03 00 00 00 00 00 00 00
// RX    22:32:15.145685    NFD         04D    04 16 B1 01 00 00 00 00
// RX    22:32:15.145685    NFD         04E    04 00 00 00 00 00 00 00
// RX    22:32:15.207698    NFD         04D    01 15 B3 01 AA 02 43 00
// RX    22:32:15.207698    NFD         04E    51 07 B0 00 00 00 00 00

    const uint8_t esc_index = (frame.data[0] & 0x0F) - 1;
    if (esc_index >= ARRAY_SIZE(esc)) {
        return;
    }

#define UINT16_to_float(indexMSB, indexLSB) (float(UINT16_VALUE(frame.data[indexMSB], frame.data[indexLSB])))

    if (frame.id == 0x04D) {
        // Voltage and RPM
        const TelemetryData t { .voltage = UINT16_to_float(3,2) * 0.1f };
        update_telem_data(esc_index, t, AP_ESC_Telem_Backend::TelemetryType::VOLTAGE);
        update_rpm(esc_index, UINT16_to_float(5,4));

    } else if (frame.id == 0x04E) {
        // Current
        const TelemetryData t { .current = UINT16_to_float(1,2) * 0.0001f };
        update_telem_data(esc_index, t, AP_ESC_Telem_Backend::TelemetryType::CURRENT);

    } else {
        // don't set timestamp/health for unhandled frames
        return;
    }

    {
        const uint32_t now_ms = AP_HAL::millis(); // fetching the time outside the semaphore to minimize time spent inside
        WITH_SEMAPHORE(sem);
        esc[esc_index].is_healthy = true;
        esc[esc_index].timestamp_ms = now_ms;
    }
}
#endif // AP_FREEFLY_ALTA_X_ENABLED
