/*
 * Copyright (C) 2021  Kraus Hamdani Aerospace Inc. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.

 * Author: Tom Pittenger, Samuel Tabor
 */


#include "Plane.h"

#if HAL_STALL_RECOVERY_ENABLED
/*
  stall detection algorithm and anti-stall management
 */
/*
  Do we think we are stalled?
  Probabilistic method where a bool is low-passed and considered a probability.
*/
void Plane::StallState::detection_update(void)
{
    if (plane.control_mode == &plane.mode_stallrecovery) {
        // we are actively recovering from a stall. Let the mode control all stall values, we'll take over when it's done.
        detection_log();
        return;
    }
    if (!plane.arming.is_armed() ||
        !plane.is_flying() ||
        plane.crash_state.is_crashed)
    {
        reset();
        detection_log();
        return;
    }

    const bool is_stalled_initial = is_stalled();

    last_detection = detect_stall();

    // LPF confidence
    confidence = ((float)last_detection * LPFcoef) + (confidence * (1.0f - LPFcoef));

    const bool is_stalled_state_has_not_changed = (is_stalled_initial == is_stalled());
    
    if (plane.mode_stallrecovery.detection_bitmask == 0 || is_stalled_state_has_not_changed) {
        // we're not trying to detect anything or, so nothing to do 
        detection_log();
        return;
    }

    if (!is_stalled()) {
        // recovered from a stall
        stall_clear();
        detection_log();
        return;
    }

    // we just stalled
    stall_start();

    plane.gcs().send_text(MAV_SEVERITY_WARNING, "STALL DETECTED!");

    count++;
    const bool mode_supports_mode_switch = (plane.control_mode != &plane.mode_manual) && (plane.control_mode != &plane.mode_stabilize);
    if (plane.mode_stallrecovery.recovery_enable > 0 && mode_supports_mode_switch) {
        // back up a few things that we'll resume and climb to after the recovery for a smoother resume
        plane.mode_stallrecovery.backup_mode_details();
        plane.set_mode(plane.mode_stallrecovery, ModeReason::STALL_DETECTED);

        // If the recovery_enable param is >= 2, then every "recovery_enable" times it will nudge up airspeed min and cruise speed
        if ((plane.mode_stallrecovery.recovery_enable >= 2) && (count % plane.mode_stallrecovery.recovery_enable) == 0) {
            // bump up the min and cruise airspeeds after every couple stalls

            plane.aparm.airspeed_min = plane.aparm.airspeed_min.get() + 1;
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "STALL has set ARSPD_FBW_MIN: %d", (int)plane.aparm.airspeed_min);

            plane.aparm.airspeed_cruise_cm = plane.aparm.airspeed_cruise_cm.get() * 1.02f;
            plane.gcs().send_text(MAV_SEVERITY_WARNING, "STALL has set TRIM_ARSPD_CM: %d", (int)plane.aparm.airspeed_cruise_cm);
        }
    }
    
    detection_log();
}

void Plane::StallState::detection_log()
{
    // log to AP_Logger
    AP::logger().Write(
       "STAL",
       "TimeUS,Actv,Raw,Any,Conf,Rec,Ex",
       "s------",
       "F------",
       "QBBIfBI",
       AP_HAL::micros64(),
       (uint8_t)is_stalled(),
       (uint8_t)last_detection,
       (uint32_t)detection_result_if_everything_enabled,
       (double)confidence,
       (uint8_t)is_recovering(),
       (uint32_t)definitely_not_stalling_exception_flags);
}

// return true if we think we're stalling
bool Plane::StallState::detect_stall()
{
    const uint32_t now_ms = AP_HAL::millis();

    // Time since last update.
    const float deltaT_us = 0.001f * (now_ms - last_update_ms);

    // Directly take nav_roll_cd, used if time since last update is too long indicating a reset.
    float limited_nav_roll = 0.01f*plane.nav_roll_cd;

    if (deltaT_us < 0.5) {
        // Difference between current nav roll and previous limited value.
        const float delta_angle = limited_nav_roll - last_limited_nav_roll;

        // New limited nav roll
        limited_nav_roll = last_limited_nav_roll + constrain_float(delta_angle, -50.0f*deltaT_us, 50.0f*deltaT_us);
    }

    // Save into old values
    last_update_ms = now_ms;
    last_limited_nav_roll = limited_nav_roll;

    uint32_t flags = 0;

    // check bitmask options
    // ----------------
    // TECS is not able to hold the desired altitude
    const bool uncommanded_altitude_loss = plane.TECS_controller.uncommanded_altitude_loss();
    flags |= detection_single_check(Stall_Detect::BAD_DESCENT, uncommanded_altitude_loss);

    // we're sinking faster than a controlled sink is allowed to be
    const float sink_rate = plane.TECS_controller.get_max_sinkrate();
    flags |= detection_single_check(Stall_Detect::SINKRATE_2X_MAX, plane.auto_state.sink_rate > (sink_rate * 2));
    flags |= detection_single_check(Stall_Detect::SINKRATE_4X_MAX, plane.auto_state.sink_rate > (sink_rate * 4));

    // we're rolling faster than a controlled roll is allowed to be
    const uint32_t roll_error_cd = fabsf(100.0f*limited_nav_roll - plane.ahrs.roll_sensor);
    flags |= detection_single_check(Stall_Detect::BAD_ROLL_20DEG, roll_error_cd > 2000);
    flags |= detection_single_check(Stall_Detect::BAD_ROLL_30DEG, roll_error_cd > 3000);

    // we're pitching faster than a controlled pitch is allowed to be
    const uint32_t pitch_error_cd = labs(labs(plane.nav_pitch_cd) - labs(plane.ahrs.pitch_sensor));
    flags |= detection_single_check(Stall_Detect::BAD_PITCH_10DEG, pitch_error_cd > 1000);
    flags |= detection_single_check(Stall_Detect::BAD_PITCH_20DEG, pitch_error_cd > 2000);
    flags |= detection_single_check(Stall_Detect::BAD_PITCH_30DEG, pitch_error_cd > 3000);
    flags |= detection_single_check(Stall_Detect::BAD_PITCH_40DEG, pitch_error_cd > 4000);

    // We don't use plane.altitude_error because the target component of this is not
    // limited by the maximum climb/sink rates.
    // positive plane.altitude_error_cm means too low
    const float altitude_error = plane.TECS_controller.get_altitude_error();
    flags |= detection_single_check(Stall_Detect::BAD_ALT_10m, altitude_error > 10.0f);
    flags |= detection_single_check(Stall_Detect::BAD_ALT_20m, altitude_error > 20.0f);
    flags |= detection_single_check(Stall_Detect::BAD_ALT_40m, altitude_error > 40.0f);
    flags |= detection_single_check(Stall_Detect::BAD_ALT_60m, altitude_error > 60.0f);

    detection_result_if_everything_enabled = flags;

    // check some generic things that are true for all aircraft types
    // ----------------
    definitely_not_stalling_exception_flags = 0;
    if (plane.auto_state.sink_rate < -3) {
        // Note: sink_rate is NED
        // we're going up fast, this is strong evidence that we're not stalling
        definitely_not_stalling_exception_flags |= uint32_t(Stall_Detect_Exceptions::CLIMBING_3mps);
    }
    // TODO: add more exceptions as we think og them...


    if (definitely_not_stalling_exception_flags != 0) {
        stall_clear();
        return false;
    }

    // to be stalled, all bits in STALL_DETECT must be hitting their thresholds
    const uint32_t enabled_bits = plane.mode_stallrecovery.detection_bitmask;
    return ((flags & enabled_bits) == enabled_bits);
}

uint32_t Plane::StallState::detection_single_check(const Stall_Detect _bitmask, const bool check) const
{
    if (check) {
        return static_cast<uint32_t>(_bitmask);
    }
    return 0;
}

#endif // HAL_STALL_RECOVERY_ENABLED