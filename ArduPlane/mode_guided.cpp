#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        loc.offset_bearing(degrees(ahrs.groundspeed_vector().angle()),
                           plane.quadplane.stopping_distance());
    }
#endif

    // set guided radius to WP_LOITER_RAD on mode change.
    active_radius_m = 0;

    // reset the subMode states
    _guided_mode = SubMode::Waypoint;
    trajectory_exit();
    plane.set_guided_WP(loc);

    return true;
}

void ModeGuided::update()
{
#if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll in the last 3 seconds?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < 3000) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < 3000) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Throttle output
    if (plane.guided_throttle_passthru) {
        // manual passthrough of throttle in fence breach
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < 3000) {
        // Received an external msg that guides throttle in the last 3 seconds?
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS control
        plane.calc_throttle();
    }
}

void ModeGuided::navigate()
{
    switch (_guided_mode) {
    case SubMode::Waypoint:
        plane.update_loiter(active_radius_m);
        break;

    case SubMode::Trajectory:
        navigate_trajectory();
        break;

    default:
        gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED SubMode %u", (unsigned)_guided_mode);
        _guided_mode = SubMode::Waypoint;
        break;
    }
}

void ModeGuided::navigate_trajectory()
{
#if 0
    // DEBUG: set param BRD_SERIAL_NUM to 1 to enable crosstrack
    plane.auto_state.next_wp_crosstrack = (AP::boardConfig()->get_serial_number() == 1);
#endif

    const AP_Mission::Mission_Command mission_cmd = trajectory_to_mission_cmd();

    // perform the AUTO command for navgation
    const bool wp_has_been_reached = plane.verify_command(mission_cmd);
    if (!wp_has_been_reached) {
        // still going...
        return;
    }

    // we have reached it
    plane.gcs().send_mission_item_reached_message(0);

    // TODO: notify AP_DDS of waypoint reached

    trajectory.pop_front();

    // trajectory_start() will initialize the next trajectory point.
    if (!trajectory_start()) {
        // We just reached the last point or something went wrong.
        // Act as if we just entered Guided and loiter around the last waypoint
        plane.set_guided_WP(mission_cmd.content.location);
        _guided_mode = SubMode::Waypoint;
    }
}

bool ModeGuided::trajectory_start()
{
    if (trajectory.empty()) {
        trajectory_exit();
        return false;
    }

    const AP_Mission::Mission_Command mission_cmd = trajectory_to_mission_cmd();

    if (!plane.start_command(mission_cmd)) {
        // start failed, start loitering where we're at
        trajectory_exit();
        return false;
    }

    _guided_mode = SubMode::Trajectory;
    return true;
}

void ModeGuided::trajectory_exit()
{
    // there are either no more cmds or starting the next cmd failed
    trajectory.clear();

    plane.set_guided_WP(plane.current_loc);

    _guided_mode = SubMode::Waypoint;
}

AP_Mission::Mission_Command ModeGuided::trajectory_to_mission_cmd() const
{
    AP_Mission::Mission_Command mission_cmd {};

    mission_cmd.id = MAV_CMD_NAV_WAYPOINT;
    mission_cmd.p1 = 1;  // acceptance radius in meters is lowest, no pass by distance
    mission_cmd.content.location = trajectory.front();

    return mission_cmd;
}

bool ModeGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    plane.set_guided_WP(target_loc);

    // use waypoint navigation sub-mode
    _guided_mode = SubMode::Waypoint;

    return true;
}

void ModeGuided::set_radius_and_direction(const float radius, const bool direction_is_ccw)
{
    // constrain to (uint16_t) range for update_loiter()
    active_radius_m = constrain_int32(fabsf(radius), 0, UINT16_MAX);
    plane.loiter.direction = direction_is_ccw ? -1 : 1;
}

void ModeGuided::update_target_altitude()
{
#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // target altitude can be negative (e.g. flying below home altitude from the top of a mountain)
    if (((plane.guided_state.target_alt_time_ms != 0) || plane.guided_state.target_location.alt != -1 )) { // target_alt now defaults to -1, and _time_ms defaults to zero.
        // offboard altitude demanded
        uint32_t now = AP_HAL::millis();
        float delta = 1e-3f * (now - plane.guided_state.target_alt_time_ms);
        plane.guided_state.target_alt_time_ms = now;
        // determine delta accurately as a float
        float delta_amt_f = delta * plane.guided_state.target_alt_rate;
        // then scale x100 to match last_target_alt and convert to a signed int32_t as it may be negative
        int32_t delta_amt_i = (int32_t)(100.0 * delta_amt_f); 
        // To calculate the required velocity (up or down), we need to target and current altitudes in the target frame
        const Location::AltFrame target_frame = plane.guided_state.target_location.get_alt_frame();
        int32_t target_alt_previous_cm;
        if (plane.current_loc.initialised() && plane.guided_state.target_location.initialised() && 
            plane.current_loc.get_alt_cm(target_frame, target_alt_previous_cm)) {
            // create a new interim target location that that takes current_location and moves delta_amt_i in the right direction
            int32_t temp_alt_cm = constrain_int32(plane.guided_state.target_location.alt, target_alt_previous_cm - delta_amt_i,  target_alt_previous_cm + delta_amt_i);
            Location temp_location = plane.guided_state.target_location;
            temp_location.set_alt_cm(temp_alt_cm, target_frame);

            // incrementally step the altitude towards the target            
            plane.set_target_altitude_location(temp_location);
        }
    } else 
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
        {
        Mode::update_target_altitude();
    }
}
