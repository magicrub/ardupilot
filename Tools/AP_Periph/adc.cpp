/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_ADC

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Periph_ADC::var_info[] {

    // @Param: ADC_RATE
    // @DisplayName: ADC Send Rate Interval
    // @Description: ADC Send Rate Interval
    // @Range: -1 2000
    // @Increment: 1
    // @Units: ms
    AP_GROUPINFO("ADC_RATE", 1, Periph_ADC, params.send_rate, 1000),
        
    AP_GROUPEND
};

void Periph_ADC::init()
{
    lib.init();
}

void Periph_ADC::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_update_ms >= 10) {
        _last_update_ms = now_ms;
        IGNORE_RETURN(lib.read(_samples, ARRAY_SIZE(_samples)));
    }

    if (params.send_rate >= 1 || params.send_rate == -1) {
        const uint32_t last_sample_timestamp_ms = lib.get_last_sample_timestamp_ms();
        const bool new_sample_available = (last_sample_timestamp_ms != 0) && (last_sample_timestamp_ms != _last_sample_timestamp_ms);

        if (params.send_rate == -1) {
            if (now_ms - _last_debug_ms >= 1000) {
                _last_debug_ms = now_ms;
                _last_sample_timestamp_ms = last_sample_timestamp_ms;

                can_printf("ADC%s %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
                    new_sample_available ? "" : " STALE",
                    _samples[0].data,
                    _samples[1].data,
                    _samples[2].data,
                    _samples[3].data,
                    _samples[4].data,
                    _samples[5].data);
            }

        } else if (new_sample_available && now_ms - _last_send_ms >= params.send_rate) {
            _last_sample_timestamp_ms = last_sample_timestamp_ms;
            _last_send_ms = now_ms;

            ardupilot_equipment_power_BatteryCells pkt1 {};
            pkt1.voltages.len = MIN(ARRAY_SIZE(_samples),ARRAY_SIZE(pkt1.voltages.data));
            for (uint8_t i=0; i<pkt1.voltages.len; i++) {
                pkt1.voltages.data[i] = _samples[i].data;
            }

            uint8_t buffer1[ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_MAX_SIZE] {};
            uint16_t total_size1 = ardupilot_equipment_power_BatteryCells_encode(&pkt1, buffer1, !periph.canfdout());
            periph.canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_SIGNATURE,
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer1[0],
                            total_size1);



            rb_ADC pkt2 {};
            pkt2.voltages.len = MIN(ARRAY_SIZE(_samples),ARRAY_SIZE(pkt2.voltages.data));
            for (uint8_t i=0; i<pkt2.voltages.len; i++) {
                pkt2.voltages.data[i] = _samples[i].data;
            }

            uint8_t buffer2[RB_ADC_MAX_SIZE] {};
            uint16_t total_size2 = rb_ADC_encode(&pkt2, buffer2, !periph.canfdout());
            periph.canard_broadcast(RB_ADC_SIGNATURE,
                            RB_ADC_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer2[0],
                            total_size2);
        }
    }
}

#endif  // HAL_PERIPH_ENABLE_ADC

