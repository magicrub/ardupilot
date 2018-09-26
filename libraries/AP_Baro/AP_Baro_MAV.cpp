
#include "AP_Baro_MAV.h"
#include <GCS_MAVLink/GCS.h>

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_MAV::AP_Baro_MAV(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
}

void AP_Baro_MAV::handle_msg(const mavlink_message_t *msg)
{
    uint32_t now_ms = AP_HAL::millis();

    float pressure, temperature;

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_RAW_PRESSURE:
        {
            mavlink_raw_pressure_t packet;
            mavlink_msg_raw_pressure_decode(msg, &packet);

            _last_timestamp = now_ms;
            pressure = (float)packet.press_abs * 100.0f;// hectopascal
            temperature = (float)packet.temperature * 0.01f; // C*100 -> C
        }
        break;

        case MAVLINK_MSG_ID_SCALED_PRESSURE:
        {
            mavlink_scaled_pressure_t packet;
            mavlink_msg_scaled_pressure_decode(msg, &packet);

            _last_timestamp = now_ms;
            pressure = packet.press_abs * 100;// hectopascal
            temperature = (float)packet.temperature * 0.01f; // C*100 -> C
        }
    break;

        default:
        return;
    }

    _pressure = pressure;
    _temperature = temperature;
    new_pressure = true;

#if 0
    static uint32_t debug_out_ms = 0;
    static uint32_t debug_samples_per_second = 0;
    debug_samples_per_second++;
    if (now_ms - debug_out_ms >= 1000) {
        debug_out_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_DEBUG,"%d RX Baro_MAV: %d, %.1fC, %.2f",now_ms, debug_samples_per_second, temperature, pressure);
        debug_samples_per_second = 0;
    }
#endif
}

// Copy sensor data to frontend
void AP_Baro_MAV::update(void)
{
    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        _copy_to_frontend(_instance, _pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

