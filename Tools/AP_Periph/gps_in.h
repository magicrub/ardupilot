#include <AP_HAL/AP_HAL.h>

#ifdef HAL_PERIPH_ENABLE_GPS_IN
#include <AP_GPS/AP_GPS_UAVCANARD.h>
#include <dronecan_msgs.h>

class Periph_GPS_In {
public:
    Periph_GPS_In() {}

    void uavcan_handle_fix(const uavcan_equipment_gnss_Fix &msg);
    void uavcan_handle_fix2(const uavcan_equipment_gnss_Fix2 &msg);
    void uavcan_handle_aux(const uavcan_equipment_gnss_Auxiliary &msg);
    void uavcan_handle_heading(const ardupilot_gnss_Heading &msg);
    void uavcan_handle_status(const ardupilot_gnss_Status &msg);

    bool seen_fix2;
    bool seen_aux;
    bool seen_message;

    AP_GPS::GPS_State interim_state;
    AP_GPS::GPS_Status status;
};

#endif // HAL_PERIPH_ENABLE_GPS_IN
