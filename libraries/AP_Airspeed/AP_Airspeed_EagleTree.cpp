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

extern const AP_HAL::HAL &hal;

// Driver info from EagleTree:
// https://www.eagletreesystems.com/Manuals/microsensor-i2c.pdf

// EagleTree Airspeed V3 driver Instructions:
// https://www.eagletreesystems.com/Manuals/airspeed-v3.pdf

#define EAGLETREE_AIRSPEED_I2C_ADDR         0x75 // (0xEA>>1)
#define EAGLETREE_AIRSPEED_I2C_READ_CMD     0x07

#ifdef DEBUG
    #undef DEBUG
#endif

#define DEBUG 1
#if DEBUG
#include <GCS_MAVLink/GCS.h>
#endif

AP_Airspeed_EagleTree::AP_Airspeed_EagleTree(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{

}

// probe and initialise the sensor
bool AP_Airspeed_EagleTree::init()
{
    dev = hal.i2c_mgr->get_device(get_bus(), EAGLETREE_AIRSPEED_I2C_ADDR);
    if (!dev) {
        return false;
    }

    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);

    // lots of retries during probe

    measure();
    hal.scheduler->delay(10);
    collect();

    const bool found = (last_sample_time_ms != 0);
    if (!found) {
        return false;
    }

#if DEBUG
    // use zero retries for cleaner I2C traffic analysis
    dev->set_retries(0);
#else
    dev->set_retries(2);
#endif

    // TODO: determine fastest stable sample interval
    dev->register_periodic_callback(2000,
            FUNCTOR_BIND_MEMBER(&AP_Airspeed_EagleTree::timer, void));

    return true;
}

void AP_Airspeed_EagleTree::timer()
{
// EagleTree Airspeed V3 driver psuedo-code Instructions:
// https://www.eagletreesystems.com/Manuals/airspeed-v3.pdf
//    byte data[2];
//    signed short reading = 0xFFFF;
//    i2c_start();
//    // select sensor in write mode
//    if (!(i2c_write(SENSOR_ADDRESS | I2C_WRITE_BIT))) {
//    // send "read data" command to sensor
//        if (!i2c_write(0x07)) {
//            i2c_restart(); // perform I2C restart
//            // select sensor in read mode
//            if (!i2c_write(| SENSOR_ADDRESS | I2C_READ_BIT)) {
//                // read two bytes of sensor data
//                data[0] = i2c_read(1);
//                data[1] = i2c_read(0);
//                reading = *((signed short *)(&data[0]));
//            }
//        }
//    }


    if (measurement_started_ms == 0) {
        measure();
    } else if (AP_HAL::millis() - measurement_started_ms > 20) {
        // TODO: determine fastest stable delay after we measure before we can collect
        collect();

        // TODO: determine fastest stable delay after we collect before we can
        // re-trigger a measure. Usually this is zero, but not for this sensor
        //measure();
    }
}

// start a measurement
void AP_Airspeed_EagleTree::measure()
{
    measurement_started_ms = 0;

    const uint8_t cmd = EAGLETREE_AIRSPEED_I2C_READ_CMD;
    if (dev->transfer(&cmd, 1, nullptr, 0)) {
        measurement_started_ms = AP_HAL::millis();
#if DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "%d ET - measure() success", AP_HAL::millis());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "%d ET - measure() failed", AP_HAL::millis());
#endif
    }
}


// read the values from the sensor
void AP_Airspeed_EagleTree::collect()
{

    const uint32_t now_ms = AP_HAL::millis();

    uint8_t data[2] = {0, 0};
    measurement_started_ms = 0;

    if (!dev->transfer(nullptr, 0, data, sizeof(data))) {
#if DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "%d ET - collect() failed", now_ms);
#endif
        return;
    }

    uint16_t sample16 = (uint16_t)(data[1] << 8 | data[0]);
    if (sample16 == 0) {
#if DEBUG
        gcs().send_text(MAV_SEVERITY_INFO, "%d ET - _collect() sample = 0", now_ms);
#endif
        return;
    }

    WITH_SEMAPHORE(sem);
    pressure_sum += (float)sample16;
    press_count++;
    last_sample_time_ms = now_ms;

#if DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "%d ET - _collect() pressure_sum = %.3f", now_ms, pressure_sum);
#endif
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_EagleTree::get_differential_pressure(float &pressure_result)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_sample_time_ms > 100) {
        return false;
    }

    WITH_SEMAPHORE(sem);
    if (press_count > 0) {
        pressure = pressure_sum / press_count;
        press_count = 0;
        pressure_sum = 0;
    }
    pressure_result = pressure;

#if DEBUG
    gcs().send_text(MAV_SEVERITY_INFO, "ET - get_differential_pressure() _pressure = %.3f", pressure_result);
#endif
    return true;
}

