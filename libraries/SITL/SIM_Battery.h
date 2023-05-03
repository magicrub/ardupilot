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

#pragma once

#include <Filter/LowPassFilter.h>
#include <AP_BattMonitor/BatteryChemistryModel.h>

namespace SITL {

/*
  class to describe a motor position
 */
class Battery {
public:
    void setup(float _capacity_Ah, float _resistance, float _max_voltage);

    void init_voltage(float voltage);

    void set_current(float current_amps);
    float get_voltage(void) const;

private:
    bool initialized;
    float SOC; // state of charge
    float V1;  // polarization voltage 1
    float V2;  // polarization voltage 2
    float R0;
    float Q;
    float I;
    float num_cells;
    
    const float R1 {.001};
    const float R2 {.001};
    const float RC1 {40};
    const float RC2 {4};
    
    uint64_t last_us;
};
}
