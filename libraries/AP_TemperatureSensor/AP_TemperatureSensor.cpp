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


#include "AP_TemperatureSensor.h"

#if HAL_TEMPERATURE_SENSOR_ENABLED
#include "AP_TemperatureSensor_TSYS01.h"
#include "AP_TemperatureSensor_MCP9600.h"


extern const AP_HAL::HAL& hal;

AP_TemperatureSensor *AP_TemperatureSensor::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_TemperatureSensor::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Temperature Sensor Type
    // @Description: Type of Temperature Sensor connected
    // @Values: 0:Disabled,1:TSYS01,2:MCP9600
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE",     0, AP_TemperatureSensor, _device[0].type,    0, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// constructor
AP_TemperatureSensor::AP_TemperatureSensor()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_TemperatureSensor must be singleton");
    }
#endif
    _singleton = this;
}

/*
 * Initialize, run once
 */
void AP_TemperatureSensor::init(void)
{
    for (uint8_t i=0; i<TEMPERATURE_SENSOR_MAX_INSTANCES; i++) {
        detect_instance(i);
    }
}

//  detect if an instance of a sensor is connected.
void AP_TemperatureSensor::detect_instance(const uint8_t instance)
{
    switch (_device[instance].type) {
    case TemperatureSensorType::None:
        return;

    case TemperatureSensorType::TSYS01:
        //     FOREACH_I2C(i) {
        //     if (_add_backend(AP_RangeFinder_PulsedLightLRF::detect(i, state[instance], params[instance], _type),
        //                      instance)) {
        //         break;
        //     }
        // }

        if (AP_TemperatureSensor_TSYS01::detect()) {
            _backend[instance] = new AP_TemperatureSensor_TSYS01(*this, instance);
            return;
        }
        break;

    case TemperatureSensorType::MCP9600:
        if (AP_TemperatureSensor_MCP9600::detect()) {
            _backend[instance] = new AP_TemperatureSensor_MCP9600(*this, instance);
            return;
        }
        break;
    }
}

/*
 * periodic update
 */
void AP_TemperatureSensor::update(void)
{
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i=0; i<TEMPERATURE_SENSOR_MAX_INSTANCES; i++) {
        if (_backend[i] != nullptr && _type[i].get() != (int8_t)Type::NONE) {
            _backend[i]->update();
        }
    }
}


AP_TemperatureSensor *AP::TemperatureSensor()
{
    return AP_TemperatureSensor::get_singleton();
}
#endif // HAL_TEMPERATURE_SENSOR_ENABLED
