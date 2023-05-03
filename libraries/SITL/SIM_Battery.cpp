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

static const float soc_ocv_x[] = {0.0, 0.005063014925373088, 0.01613838805970147, 0.02905964179104481, 0.04382680597014932, 0.060439850746268675, 0.07705289552238803, 0.09920364179104468, 0.1268920298507462, 0.15642635820895523, 0.19334423880597018, 0.2357997910447761, 0.2708717910447762, 0.2967142985074628, 0.3244027164179104, 0.34839934328358213, 0.3779336417910447, 0.4037761791044776, 0.4388481492537314, 0.462844776119403, 0.4868414029850746, 0.5182216119402985, 0.5551394925373134, 0.5920573731343284, 0.6289752537313433, 0.6695849253731343, 0.7194240895522388, 0.7581878507462687, 0.7932598507462687, 0.8283318507462687, 0.8615579402985074, 0.9058594029850746, 0.9446231641791045, 0.9815410447761194, 1.0};

static const float soc_ocv_y[] = {2.5180000000000002, 2.6487000000000003, 2.75, 2.8668, 2.9681, 3.0693, 3.1550000000000002, 3.2406, 3.3107, 3.373, 3.4198, 3.4587, 3.4899, 3.5132, 3.5288, 3.5444, 3.5678, 3.5911, 3.6145, 3.6456, 3.6612, 3.7001, 3.7313, 3.7702, 3.8014, 3.8403, 3.8793, 3.9104, 3.9494000000000002, 3.9961, 4.027299999999999, 4.0584, 4.074, 4.1051, 4.158};


static BatteryChemistryModelLinearInterpolated chemistry_model{soc_ocv_x, soc_ocv_y, ARRAY_SIZE(soc_ocv_x)};

void Battery::setup(float _capacity_Ah, float _resistance, float _max_voltage)
{
    initialized = true;
    SOC = 1;
    Q = _capacity_Ah*3600;
    num_cells = _max_voltage/chemistry_model.OCV_from_SOC(1.0,25);
    R0 = _resistance/num_cells;
}

void Battery::init_voltage(float voltage)
{        
    SOC = chemistry_model.SOC_from_OCV(voltage/num_cells,25);
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

    I = current;
    
    if (!is_equal(Q,0.0f)) {
        SOC = MAX(SOC-I*dt/Q,0);
    }
    V1 = I*R1*(1 - exp(-dt/RC1)) + V1*exp(-dt/RC1);
    V2 = I*R2*(1 - exp(-dt/RC2)) + V2*exp(-dt/RC2);
}

float Battery::get_voltage(void) const
{
    if (!initialized) {
        return 0;
    }
    
    float V = chemistry_model.OCV_from_SOC(SOC,25)-R0*I-V1-V2;
    return V*num_cells;
}
