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

#include "mode.h"
#include "Plane.h"

#if HAL_STALL_RECOVERY_ENABLED
const AP_Param::GroupInfo ModeStallRecovery::var_info[] = {
    // @Param: DETECT
    // @DisplayName: Stall detection type
    // @Description: Stall detection type. Different aircraft stall in different ways.
    // @User: Advanced
    // @Bitmask: 0:CantHoldAltitude,1:SinkRate*2Max,2:SinkRate*4Max,3:RollError20deg,4:RollError30deg,5:RollError45deg,6:PitchError10deg,7:PitchError20deg,8:PitchError20deg,9:PitchError30deg,10:PitchError40deg,11:AltError10m,12:AltError20m,13:AltError40m,14:AltError60m
    AP_GROUPINFO("DETECT", 0, ModeStallRecovery, detection_bitmask, 0),

    // @Param: RECOVERY
    // @DisplayName: Stall recovery Enable
    // @Description: Stall recovery Enable. If the value is >=2 then it will nudge up the airspeed_min and cruise speed every STALL_RECOVERY times we stall.
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("RECOVERY", 1, ModeStallRecovery, recovery_enable, 0),

    // @Param: ELEV
    // @DisplayName: Stall recovery elevator
    // @Description: The fixed elevator percent (not degree) to apply when trying to recovery from a stall. Usually negative for pitch down. This is a percent because the stabilization is not running so it's not trying to hold a desired pitch angle, we're just applying a little down pitch command
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("ELEV", 2, ModeStallRecovery, param.elevator, -10),

    // @Param: THR1
    // @DisplayName: Stall throttle
    // @Description: The throttle to apply when stalled
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("THR1", 3, ModeStallRecovery, param.throttle1, 100),

    // @Param: THR2
    // @DisplayName: Stall recovery throttle
    // @Description: The throttle to apply when trying to recovery from a stall while leveling the wings. Use -1 for auto controlled throttle, otherwise its a fixed percent.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("THR2", 4, ModeStallRecovery, param.throttle2, 100),

    // @Param: DUR1MAX
    // @DisplayName: Stall recovery duration1 max
    // @Description: The maximum duration that we'll attempt to recover from a stall before leveling wings. Use -1 to never timeout.
    // @Units: s
    // @Range: -1 30000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("DUR1MAX", 5, ModeStallRecovery, param.duration1_max, 10.0f),

    // @Param: DUR2MAX
    // @DisplayName: Stall recovery duration2 max
    // @Description: The maximum duration that we'll attempt to level wings after a stall to gain airspeed before resuming previous mode. Use -1 to never timeout.
    // @Units: s
    // @Range: -1 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("DUR2MAX", 6, ModeStallRecovery, param.duration2_max, 10.0f),

    // @Param: ALGORITHM1
    // @DisplayName: Stall recovery algorithm1
    // @Description: Stall recovery algorithm1. Each enabled option is checked during Phase 1 of the stall recovery. If the aircraft meets all of the criteria it will bypass the timer and proceed to phase 2
    // @Bitmask: 0:Airspeed min reached,1:yaw rate is less than STALL_SPIN_RATE,2:sink rate is less than STALL_SINK_RATE
    // @User: Advanced
    AP_GROUPINFO("ALGORITHM1", 7, ModeStallRecovery, param.algorithm1, 0),

    // @Param: ALGORITHM2
    // @DisplayName: Stall recovery algorithm2
    // @Description: Stall recovery algorithm2. Each enabled option is checked during Phase 2 of the stall recovery. If the aircraft meets all of the criteria it will bypass the timer and proceed to
    // @Bitmask: 0:Airspeed above cruise (TRIM_ARSPD_CM),1:Airspeed above 95% of cruise,2:Airspeed above 90% of cruise
    // @User: Advanced
    AP_GROUPINFO("ALGORITHM2", 8, ModeStallRecovery, param.algorithm2, 0),

    // @Param: DUR1MIN
    // @DisplayName: Stall recovery duration1 min
    // @Description: The minimum duration that we'll force a stall recovery before attempting to level the wings. Use -1 to disable.
    // @Units: s
    // @Range: -1 30
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("DUR1MIN", 9, ModeStallRecovery, param.duration1_min, 2.0f),

    // @Param: DUR2MIN
    // @DisplayName: Stall recovery duration2 min
    // @Description: The minimum duration that we'll force attempt to level wings after a stall before we allow a timeout to happen. Use -1 to disable.
    // @Units: s
    // @Range: -1 30
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("DUR2MIN", 10, ModeStallRecovery, param.duration2_min, 2.0f),

    // @Param: SPIN_RATE
    // @DisplayName: Stall recovery spin rate
    // @Description: Stall recovery spin rate for early exit
    // @Units: deg/s
    // @Range: -1 360
    // @User: Advanced
    AP_GROUPINFO("SPIN_RATE", 11, ModeStallRecovery, param.spin_rate, -1),

    // @Param: SINK_RATE
    // @DisplayName: Stall recovery sink rate
    // @Description: Stall recovery sink rate for early exit.
    // @Units: m/s
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("SINK_RATE", 12, ModeStallRecovery, param.sink_rate, -1),

    AP_GROUPEND
};

bool ModeStallRecovery::_enter()
{
    // this is just in case the stall detection isn't who triggered the mode change.
    plane.stall_state.stall_start();

    gcs().send_text(MAV_SEVERITY_INFO, "STALL: start");

    return true;
}

void ModeStallRecovery::_exit()
{
    plane.stall_state.stall_clear();
    plane.stall_state.recovering_clear();
}

void ModeStallRecovery::update()
{
    int32_t min_ms, max_ms;
    uint32_t duration_ms;

    if (plane.stall_state.is_stalled()) {
        min_ms = param.duration1_min * 1000;
        max_ms = param.duration1_max * 1000;
        duration_ms = plane.stall_state.stall_duration_ms();
    } else {
        min_ms = param.duration2_min * 1000;
        max_ms = param.duration2_max * 1000;
        duration_ms = plane.stall_state.recovering_duration_ms();
    }

    // sanity check params so we never get stuck here
    if (min_ms > max_ms) {
        min_ms = 2000;
        max_ms = 5000;
    } else {
        min_ms = constrain_int32(min_ms, -1, max_ms);
        max_ms = constrain_int32(max_ms, min_ms, 30000);
    }

    // during minimum time, we never timeout and force staying in this mode for a minimum duration
    // when true, minimum duration, if any, has been met
    const bool allow_next_state = (min_ms < 0) || (duration_ms >= (uint32_t)min_ms);

    if (allow_next_state) {
        // when max timeout is reached, we've timed out
        const bool timed_out = (max_ms >= 0) && (duration_ms >= (uint32_t)max_ms);

        if (timed_out || is_recovered_early()) {
            if (plane.stall_state.is_stalled()) {
                // we've successfully recovered from the stall, now lets do some level flight for phase 2
                plane.stall_state.stall_clear();
                plane.stall_state.recovering_start();

            } else {
                // Phase 2 complete, we've successfully recovered from the stall finished performing some
                // level flight. Now lets go back to what we were doing before the stall
                resume_previous_mode();
                return;
            }
        }
    }
    set_servo_behavior();
}

// back up a few things that we'll resume and climb to after the recovery
void ModeStallRecovery::backup_mode_details()
{
    backup.wp_next = plane.next_WP_loc;
    backup.wp_prev = plane.prev_WP_loc;
}

void ModeStallRecovery::resume_previous_mode()
{
    // lets front-load these, some modes need them in their _enter()
    plane.prev_WP_loc = backup.wp_prev;
    plane.next_WP_loc = backup.wp_next;

    const bool mode_success = plane.set_mode(*plane.previous_mode, ModeReason::STALL_RECOVERY_RESUME);
    if (mode_success) {
        gcs().send_text(MAV_SEVERITY_INFO, "STALL: resuming mode %s", plane.control_mode->name());
            
        // try to resume the same path and altitude
        if (plane.control_mode->is_guided_mode()) {
            plane.set_target_location(backup.wp_next);
        } else {
            plane.set_next_WP(backup.wp_next);
        }
        plane.prev_WP_loc = backup.wp_prev;
        plane.auto_state.crosstrack = true;

    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "STALL: failed switch to mode %s", plane.previous_mode->name());
        plane.set_mode(plane.mode_rtl, ModeReason::STALL_RECOVERY_RESUME_FAIL);
    }
}

bool ModeStallRecovery::is_recovered_early() const
{
    uint32_t is_recovered_bits = 0;
    const uint32_t algorithm_bits = (plane.stall_state.is_stalled()) ? param.algorithm1 : param.algorithm2;

    if (algorithm_bits == 0) {
        return false;
    }

    float airspeed = -1.0f;
    const bool has_airspeed = plane.ahrs.airspeed_estimate(airspeed) && (airspeed > 0);

    if (plane.stall_state.is_stalled()) {
        // We're stalled and have lost control of the aircraft, this is STAGE 1.
        // We will stay in this stage for a minium time of param.duration1_min and a maximum time of param.duration1_max but 
        // the various checks below can also let us progress to Stage 2 sooner before we hit the max time.
        if (algorithm_bits & uint32_t(STALL_RECOVERY_1::AIRSPEED_MIN)) {
            if (has_airspeed && (plane.aparm.airspeed_min > 0) && (airspeed >= plane.aparm.airspeed_min)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_1::AIRSPEED_MIN);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 1 arspd %.1f >= %d", (double)airspeed, plane.aparm.airspeed_min.get());
            }
        }
        if (algorithm_bits & uint32_t(STALL_RECOVERY_1::SPIN_RATE_PARAM)) {
            const float threshold = param.spin_rate;
            const float spin_rate = fabsf(degrees(plane.ahrs.get_gyro().z));
            if (threshold > 0 && (spin_rate <= threshold)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_1::SPIN_RATE_PARAM);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 1 spin %.1f <= %.1f", (double)spin_rate, (double)threshold);
            }
        }
        if (algorithm_bits & uint32_t(STALL_RECOVERY_1::SINK_RATE_PARAM)) {
            const float threshold = param.sink_rate;
            if (threshold > 0 && (plane.auto_state.sink_rate <= threshold)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_1::SINK_RATE_PARAM);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 1 sink %.1f <= %.1f", (double)plane.auto_state.sink_rate, (double)threshold);
            }
        }

    } else {
        // We've recovered and presume to have at least minimum control of the aircraft to level off and hold the horizon.
        // We will stay in this stage for a minium time of param.duration2_min and a maximum time of param.duration2_max but 
        // the various checks below can also let us progress sooner before we hit the max time.
        const float airspeed_cruise = plane.aparm.airspeed_cruise_cm * 0.01f;
        if (algorithm_bits & uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_100PCT)) {
            const float airspeed_cruise_thresh = airspeed_cruise;
            if (has_airspeed && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_100PCT);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 2A arspd %.1f >= %.1f", (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (algorithm_bits & uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_95PCT)) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.95f;
            if (has_airspeed && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_95PCT);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 2B arspd %.1f >= %.1f", (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
        if (algorithm_bits & uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_90PCT)) {
            const float airspeed_cruise_thresh = airspeed_cruise * 0.90f;
            if (has_airspeed && (airspeed_cruise_thresh > 0) && (airspeed >= airspeed_cruise_thresh)) {
                is_recovered_bits |= uint32_t(STALL_RECOVERY_2::AIRSPEED_CRUISE_90PCT);
                gcs().send_text(MAV_SEVERITY_INFO, "STALL: 2C arspd %.1f >= %.1f", (double)airspeed, (double)airspeed_cruise_thresh);
            }
        }
    }

    return (is_recovered_bits & algorithm_bits) == algorithm_bits;
}

void ModeStallRecovery::set_servo_behavior()
{
    int16_t throttle = 0;

    if (plane.stall_state.is_stalled()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);

        const int16_t scaled_elev = constrain_int16(param.elevator,-100,100) * 45; // convert +/- percent to +/-4500
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, scaled_elev);

        throttle = param.throttle1;

    } else {
        // hold wings level
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;

        // assign throttle. Negative means auto-throttle
        throttle = param.throttle2;
    }

    // set throttle
    if (plane.auto_state.last_flying_ms == 0) {
        // if we've never flown, don't allow throttle. This inhibits
        // the throttle while on the ground testing the feature
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);

    } else if (does_auto_throttle()) {
        // throttle param is negative and we're recovering
        plane.calc_throttle();

    } else {
        // ensure we don't apply any negative thrust right now
        throttle = constrain_int16(throttle, 0, plane.aparm.throttle_max);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }
}

// false during STAGE 1, True during STAGE 2 for negative throttle
bool ModeStallRecovery::does_auto_throttle() const
{
    return !plane.stall_state.is_stalled() && (param.throttle2 < 0);
}

#endif // HAL_STALL_RECOVERY_ENABLED
