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
    
    // @Group: GUIDED_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(pid, "PID_", 2, Periph_ADC, AC_PID),

    // @Param: SRV_INDEX
    // @DisplayName: SRV_INDEX
    // @Description: SRV_INDEX
    AP_GROUPINFO("SRV_INDEX", 3, Periph_ADC, params.servo_index, -1),


    AP_GROUPEND
};

void Periph_ADC::init()
{
    // this creates a thread in the background to read the ADC via I2C
    adc_lib.init();
}

void Periph_ADC::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float dt = (now_ms - _last_pid_update_ms) * 1e-3f;
    _last_pid_update_ms = now_ms;


    // get a new sample
    const uint32_t last_sample_timestamp_ms = adc_lib.get_last_sample_timestamp_ms();
    const bool new_sample_available = (last_sample_timestamp_ms != 0) && (last_sample_timestamp_ms != _last_sample_timestamp_ms);


    // params.servo_index
    float target_srv_in = 0;
    const SRV_Channel *c = SRV_Channels::srv_channel(params.servo_index);
    if (c != nullptr && c->valid_function()) {
        target_srv_in = linear_interpolate(0, 5.0,  c->get_output_pwm(), c->get_output_min(), c->get_output_max());
    }

    float measurement_adc_in = adc_lib.read_by_channel(2);
    const float output_dac_out = pid.update_all(target_srv_in, measurement_adc_in, dt);

    periph.dac.set_output_voltage(output_dac_out);

    // TODO: push to 4 other servo outputs but with true PWM output at 20kHz







    // debug stuff
    if (params.send_rate >= 1 || params.send_rate == -1) {
    if (params.send_rate == -1) {
            if (now_ms - _last_debug_ms >= 1000) {
                _last_debug_ms = now_ms;
                _last_sample_timestamp_ms = last_sample_timestamp_ms;

                can_printf("ADC%s AN0SE:%.2f, AN1SE:%.2f, AN23DIF:%.2f",
                    new_sample_available ? "" : " STALE",
                    adc_lib.read_by_channel(0),
                    adc_lib.read_by_channel(1),
                    adc_lib.read_by_channel(2));
            }

        } else if (new_sample_available && now_ms - _last_send_ms >= params.send_rate) {
            _last_sample_timestamp_ms = last_sample_timestamp_ms;
            _last_send_ms = now_ms;

            ardupilot_equipment_power_BatteryCells pkt1 {};
            pkt1.voltages.len = MIN(ADS1115_CHANNELS_COUNT,ARRAY_SIZE(pkt1.voltages.data));
            for (uint8_t i=0; i<pkt1.voltages.len; i++) {
                pkt1.voltages.data[i] = adc_lib.read_by_channel(i);
            }

            uint8_t buffer1[ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_MAX_SIZE] {};
            uint16_t total_size1 = ardupilot_equipment_power_BatteryCells_encode(&pkt1, buffer1, !periph.canfdout());
            periph.canard_broadcast(ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_SIGNATURE,
                            ARDUPILOT_EQUIPMENT_POWER_BATTERYCELLS_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer1[0],
                            total_size1);



            rb_ADC pkt2 {};
            pkt2.voltages.len = MIN(ADS1115_CHANNELS_COUNT,ARRAY_SIZE(pkt2.voltages.data));
            for (uint8_t i=0; i<pkt2.voltages.len; i++) {
                pkt2.voltages.data[i] = adc_lib.read_by_channel(i);
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

