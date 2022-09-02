#include "AP_BattMonitor_SMBus_Rotoye.h"

#include <AP_HAL/AP_HAL.h>

// Specific to Rotoye Batmon
#define BATTMONITOR_SMBUS_TEMP_EXT 0x07

// return the maximum of the internal and external temperature sensors
void AP_BattMonitor_SMBus_Rotoye::read_temp(void) {
    
    uint16_t t_int, t_ext;
    
    /*  Both internal and external values will always be sent by Batmon. 
        If no external thermistor is used, a zero-value is sent. */
    if (read_word(BATTMONITOR_SMBUS_TEMP, t_int) && 
        read_word(BATTMONITOR_SMBUS_TEMP_EXT, t_ext)) {
        uint16_t t;
        t = ((t_ext > t_int) ? t_ext : t_int);
        set_temperature(KELVIN_TO_C(0.1f * (float)t));
    }
}
