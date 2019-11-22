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
  rover simulator class
*/

#include "SIM_Rover.h"
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ICEngine/AP_ICEngine.h>

#include <string.h>
#include <stdio.h>

namespace SITL {

SimRover::SimRover(const char *frame_str) :
    Aircraft(frame_str),
    max_speed(20),
    max_accel(10),
    max_wheel_turn(35),
    turning_circle(1.8),
    skid_turn_rate(140), // degrees/sec
    skid_steering(false)
{
    skid_steering = strstr(frame_str, "skid") != nullptr;

    if (skid_steering) {
        printf("SKID Steering Rover Simulation Started\n");
        // these are taken from a 6V wild thumper with skid steering,
        // with a sabertooth controller
        max_accel = 14;
        max_speed = 4;
    }

    mass = 1.0f;
}



/*
  return turning circle (diameter) in meters for steering angle proportion in degrees
*/
float SimRover::turn_circle(float steering)
{
    if (fabsf(steering) < 1.0e-6) {
        return 0;
    }
    return turning_circle * sinf(radians(max_wheel_turn)) / sinf(radians(steering*max_wheel_turn));
}

/*
   return yaw rate in degrees/second given steering_angle and speed
*/
float SimRover::calc_yaw_rate(float steering, float speed)
{
    if (skid_steering) {
        return steering * skid_turn_rate;
    }
    if (fabsf(steering) < 1.0e-6 or fabsf(speed) < 1.0e-6) {
        return 0;
    }
    float d = turn_circle(steering);
    float c = M_PI * d;
    float t = c / speed;
    float rate = 360.0f / t;
    return rate;
}

/*
  return lateral acceleration in m/s/s
*/
float SimRover::calc_lat_accel(float steering_angle, float speed)
{
    float yaw_rate = calc_yaw_rate(steering_angle, speed);
    float accel = radians(yaw_rate) * speed;
    return accel;
}

/*
  update the rover simulation by one time step
 */
void SimRover::update(const struct sitl_input &input)
{
    float steering, throttle;
    AP_ICEngine *ice = AP::ice();
    const bool gear_is_park = (ice != nullptr && ice->has_gears() && ice->gear_is_park());
    const bool gear_is_neutral = (ice != nullptr && ice->has_gears() && ice->gear_is_neutral());
    const bool gear_is_reverse = (ice != nullptr && ice->has_gears() && ice->gear_is_reverse());

    // if in skid steering mode the steering and throttle values are used for motor1 and motor2
    if (skid_steering) {
        float motor1 = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
        float motor2 = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
        steering = motor1 - motor2;
        throttle = 0.5*(motor1 + motor2);
    } else {
        steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
        throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);
    }
    
    icengine->update(input, throttle);

    // how much time has passed?
    float delta_time = frame_time_us * 1.0e-6f;

    if (gear_is_park) {
        throttle = 0;
        velocity_ef.x = 0;
        velocity_ef.y = 0;
    } else if (gear_is_neutral) {
        throttle = 0;
    } else if (gear_is_reverse) {
        throttle *= -1;
    }

    // speed in m/s in body frame
    Vector3f velocity_body = dcm.transposed() * velocity_ef;

    // speed along x axis, +ve is forward
    float speed = velocity_body.x;

    // yaw rate in degrees/s
    float yaw_rate = calc_yaw_rate(steering, speed);

    // target speed with current throttle
    float target_speed = throttle * max_speed;

    // linear acceleration in m/s/s - very crude model
    float accel = max_accel * (target_speed - speed) / max_speed;

    gyro = Vector3f(0,0,radians(yaw_rate));

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    if (gear_is_park) {
        // We're in park, thus, not moving
        accel_body = Vector3f(0, 0, 0);

    } else {
        // if braking is configured, apply it assuming 1G
        float braking = 0;
        if (SRV_Channels::function_assigned(SRV_Channel::k_brake)) {
            braking = MAX(0,GRAVITY_MSS * (SRV_Channels::get_output_scaled(SRV_Channel::k_brake) * 0.01f));

            // if you have brakes, assume you need them!
            mass = 4.0f;
        }

        if (is_negative(speed)) {
            // negative acceleration when going backwards. It's all relative!
            braking *= -1;
        }

        // accel in body frame due to motor minus brake
        accel_body = Vector3f(accel - braking, 0, 0);
    }

    // apply inertia
    accel_body /= mass;

    // add in accel due to direction change
    accel_body.y += radians(yaw_rate) * speed;

    // now in earth frame
    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0, 0, GRAVITY_MSS);

    // we are on the ground, so our vertical accel is zero
    accel_earth.z = 0;

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    // new position vector
    position += velocity_ef * delta_time;

    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
