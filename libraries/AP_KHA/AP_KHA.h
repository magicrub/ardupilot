#pragma once
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
  temperature calibration library. This monitors temperature changes
  and opportunistically calibrates sensors when the vehicle is still
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

class AP_KHA
{
public:
    // constructor.  This remains because construction of Copter's g2
    // object becomes problematic if we don't have at least one object
    // to initialise
    AP_KHA() {}

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    void update(void);

    /* Do not allow copies */
    AP_KHA(const AP_KHA &other) = delete;
    AP_KHA &operator=(const AP_KHA&) = delete;

    
private:

    void init();
    void service_loopback();

    struct {
        uint32_t timer_ms;
        bool done;
        uint8_t state;
    } _init;

    struct {
        AP_Int8 enabled;
        AP_Int8 serial_loopback_broadcast;
        
    } _param;
    
    AP_HAL::UARTDriver *_port_loopback[SERIALMANAGER_NUM_PORTS];

    
};
