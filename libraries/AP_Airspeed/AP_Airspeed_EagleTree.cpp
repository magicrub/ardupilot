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

#include "AP_Airspeed_EagleTree.h"
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// Driver info from EagleTree:
// https://www.eagletreesystems.com/Manuals/microsensor-i2c.pdf

#define EAGLETREE_AIRSPEED_I2C_ADDR         0x75 // (7bit version of 0xEA)
#define EAGLETREE_AIRSPEED_I2C_READ_CMD     0x07

/* Measurement rate is 100Hz */
#define EAGLETREE_AIRSPEED_CONVERSION_INTERVAL (1000000UL / 100U) /* microseconds */


AP_Airspeed_EagleTree::AP_Airspeed_EagleTree(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

// probe and initialise the sensor
bool AP_Airspeed_EagleTree::init()
{
    dev = hal.i2c_mgr->get_device(get_bus(), EAGLETREE_AIRSPEED_I2C_ADDR);
    if (!dev) {
        return false;
    }
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    dev->set_retries(2);
    dev->get_semaphore()->give();
    dev->register_periodic_callback(EAGLETREE_AIRSPEED_CONVERSION_INTERVAL,
            FUNCTOR_BIND_MEMBER(&AP_Airspeed_EagleTree::timer, void));
            
    sem->give();
    return true;
}

// 100Hz timer
void AP_Airspeed_EagleTree::timer()
{
    if (!sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    uint8_t val[2];
    if (!dev->read_registers(EAGLETREE_AIRSPEED_I2C_READ_CMD, (uint8_t *)&val, sizeof(val))) {
        sem->give();
        return;
    }

    uint16_t sample16 = (uint16_t)(val[1] << 8 | val[0]);
    if (sample16 == 0) {
        sem->give();
        return;
    }

    pressure_sum += (float)sample16;
    press_count++;
    last_sample_time_ms = AP_HAL::millis();
    sem->give();
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_EagleTree::get_differential_pressure(float &_pressure)
{
    if ((AP_HAL::millis() - last_sample_time_ms) > 100) {
        return false;
    }

    if (!sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }
    sem->give();

    // _pressure = pressure;
     return true;
}
