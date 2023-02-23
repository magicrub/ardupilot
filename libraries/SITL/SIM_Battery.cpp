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
/*
  battery model for electric aircraft
*/

#include "SIM_Battery.h"

using namespace SITL;

/*
  state of charge table for a single cell battery.
 */
static const struct {
    float volt_per_cell;
    float soc_pct;
} soc_table[] = {
{ 4.158, 100.0 },
{ 4.1051, 98.15410447761194 },
{ 4.074, 94.46231641791046 },
{ 4.0584, 90.58594029850747 },
{ 4.027299999999999, 86.15579402985074 },
{ 3.9961, 82.83318507462687 },
{ 3.9494000000000002, 79.32598507462687 },
{ 3.9104, 75.81878507462687 },
{ 3.8793, 71.94240895522388 },
{ 3.8403, 66.95849253731343 },
{ 3.8014, 62.897525373134336 },
{ 3.7702, 59.205737313432834 },
{ 3.7313, 55.51394925373134 },
{ 3.7001, 51.822161194029846 },
{ 3.6612, 48.68414029850746 },
{ 3.6456, 46.284477611940304 },
{ 3.6145, 43.88481492537314 },
{ 3.5911, 40.37761791044776 },
{ 3.5678, 37.79336417910447 },
{ 3.5444, 34.83993432835821 },
{ 3.5288, 32.44027164179104 },
{ 3.5132, 29.67142985074628 },
{ 3.4899, 27.08717910447762 },
{ 3.4587, 23.579979104477612 },
{ 3.4198, 19.33442388059702 },
{ 3.373, 15.642635820895523 },
{ 3.3107, 12.68920298507462 },
{ 3.2406, 9.920364179104467 },
{ 3.1550000000000002, 7.705289552238803 },
{ 3.0693, 6.0439850746268675 },
{ 2.9681, 4.382680597014932 },
{ 2.8668, 2.905964179104481 },
{ 2.75, 1.613838805970147 },
{ 2.6487000000000003, 0.5063014925373088 },
{ 0.001, 0.001 }
};

/*
  use table to get resting voltage from remaining capacity
 */
float Battery::get_resting_voltage(float charge_pct) const
{
    const float max_cell_voltage = soc_table[0].volt_per_cell;
    for (uint8_t i=1; i<ARRAY_SIZE(soc_table); i++) {
        if (charge_pct >= soc_table[i].soc_pct) {
            // linear interpolation between table rows
            float dv1 = charge_pct - soc_table[i].soc_pct;
            float dv2 = soc_table[i-1].soc_pct - soc_table[i].soc_pct;
            float vpc1 = soc_table[i].volt_per_cell;
            float vpc2 = soc_table[i-1].volt_per_cell;
            float cell_volt = vpc1 + (dv1 / dv2) * (vpc2 - vpc1);
            return (cell_volt / max_cell_voltage) * max_voltage;
        }
    }
    // off the bottom of the table, return a small non-zero to prevent math errors
    return 0.001;
}

/*
  use table to set initial state of charge from voltage
 */
void Battery::set_initial_SoC(float voltage)
{
    const float max_cell_voltage = soc_table[0].volt_per_cell;
    float cell_volt = (voltage / max_voltage) * max_cell_voltage;

    for (uint8_t i=1; i<ARRAY_SIZE(soc_table); i++) {
        if (cell_volt >= soc_table[i].volt_per_cell) {
            // linear interpolation between table rows
            float dv1 = cell_volt - soc_table[i].volt_per_cell;
            float dv2 = soc_table[i-1].volt_per_cell - soc_table[i].volt_per_cell;
            float soc1 = soc_table[i].soc_pct;
            float soc2 = soc_table[i-1].soc_pct;
            float soc = soc1 + (dv1 / dv2) * (soc2 - soc1);
            remaining_Ah = capacity_Ah * soc * 0.01;
            return;
        }
    }

    // off the bottom of the table
    remaining_Ah = 0;
}

void Battery::setup(float _capacity_Ah, float _resistance, float _max_voltage)
{
    capacity_Ah = _capacity_Ah;
    resistance = _resistance;
    max_voltage = _max_voltage;
}

void Battery::init_voltage(float voltage)
{
    voltage_filter.reset(voltage);
    voltage_set = voltage;
    set_initial_SoC(voltage);
}

void Battery::set_current(float current)
{
    uint64_t now = AP_HAL::micros64();
    float dt = (now - last_us) * 1.0e-6;
    if (dt > 0.1) {
        // we stopped updating
        dt = 0;
    }
    last_us = now;
    float delta_Ah = current * dt / 3600;
    remaining_Ah -= delta_Ah;
    remaining_Ah = MAX(0, remaining_Ah);

    float voltage_delta = current * resistance;
    float voltage;
    if (!is_positive(capacity_Ah)) {
        voltage = voltage_set;
    } else {
        voltage = get_resting_voltage(100 * remaining_Ah / capacity_Ah) - voltage_delta;
    }

    voltage_filter.apply(voltage);
}

float Battery::get_voltage(void) const
{
    return voltage_filter.get();
}
