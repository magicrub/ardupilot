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
  a stratospheric blimp simulator class
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_SA_GD2000_ENABLED

#include "SIM_Plane.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  a Silent Arrow GD2000 simulator
 */

class SA_GD2000 : public Plane {
public:
    SA_GD2000(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW SA_GD2000(frame_str);
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Float param_mass;
};

}
#endif // AP_SIM_SA_GD2000_ENABLED