/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/// @Author: Tom Pittenger 2015
/// @File: airphrame.cpp

/*
    Airphrame specific source code. Defines are found in airphrame.h
 */


/*
 * in order to always land into the wind, this function will update the location of
 * the land command to enforce an always-into-the-wind landing approach.
 */
void Plane::do_rotate_landing_direction(const AP_Mission::Mission_Command& cmd)
{
    AP_Mission::Mission_Command land_cmd;
    AP_Mission::Mission_Command prev_nav_cmd;
    float approach_bearing = -1;
    uint16_t index;

    // disable by default, enable on success at the end
    land_approach_rotation.disable();

    // and there exists a NAV_LAND
    prev_nav_cmd = mission.get_current_nav_cmd();
    index = cmd.index+1; // start with looking at the command after the DO_ROTATE_LAND
    while (mission.get_next_nav_cmd(index, land_cmd)) {
        if (land_cmd.id == MAV_CMD_NAV_LAND) {
            // LAND waypoint found, compute approach heading and continue on
            approach_bearing = get_bearing_cd(prev_nav_cmd.content.location, land_cmd.content.location) * 0.01f;
            break;
        } else {
            prev_nav_cmd = land_cmd;
            // account for grabbing a DO cmd which returned the nav index
            // of the next index so simply incrementing index will not work
            index = land_cmd.index+1;
        }
    }

    // check that we have a properly calculated approach angle of way-point to-> land point
    if (approach_bearing < 0) {
        // We hit the end of the mission and never saw a LAND point or
        // something wrong with the current wp+land bearing calc
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: could not find NAV_LAND");
        return;
    }

    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_DEBUG, "found land index %d", land_cmd.index);


    // get wind values
    Vector3f wind = ahrs.wind_estimate();
    float wind_length = wind.length();
    float wind_rad = atan2f(wind.y, wind.x);
    float wind_deg = degrees(wind_rad);

    // with wind: (wind_deg - approach_bearing) == 0   (wind dir matches)
    // into wind: (wind_deg - approach_bearing) == 180 (wind dir opposite)
    // with wind if (wind_dot_approach > 0)
    // perpendicular to wind if (wind_dot_approach == 0)
    // into wind if (wind_dot_approach < 0)
    float wind_dot_approach = wind_length * cosf(radians(wind_deg - approach_bearing));

    // check if we're landing with the wind. Negative would mean into the wind.
    bool do_rotate_approach_Wp = (wind_dot_approach > 0.1f);

    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_DEBUG, "BiDirectional Land dot product: %.2f, %d", (double)wind_dot_approach, do_rotate_approach_Wp);

    if (!do_rotate_approach_Wp && cmd.content.rotate_landing.offset == 0) {
        // we're already landing into the wind and there's no offset to apply
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: no change to landing");
        return;
    }

    // offset all waypoints between DO_ROTATE_LAND and NAV_LAND
    int16_t offset =  cmd.content.rotate_landing.offset;

    switch (cmd.content.rotate_landing.action) {
    case MAV_ROTATE_LANDING_DIR_ACTION_AFTER_HERE:
        index = cmd.index;
        break;

    case MAV_ROTATE_LANDING_DIR_ACTION_APPROACH_POINT_ONLY:
        index = prev_nav_cmd.index;
        break;

    case MAV_ROTATE_LANDING_DIR_ACTION_AFTER_DO_LAND_START:
        index = mission.get_landing_sequence_start();
        if (index == 0) {
            // if no DO_LAND_START, use the current DO_ROTATE_LAND index
            index = cmd.index;
        }
        break;

    default:
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: unknown action %d", cmd.content.rotate_landing.action);
        return;
    }

    float rotation_angle;

    switch (cmd.content.rotate_landing.type) {
    case MAV_ROTATE_LANDING_DIR_TYPE_180_DEG:
        if (do_rotate_approach_Wp) {
            rotation_angle = 180;
        } else {
            rotation_angle = 0;
        }
        break;

    case MAV_ROTATE_LANDING_DIR_TYPE_MATCH_WIND:
        rotation_angle = wind_deg + approach_bearing + 180;
        break;

    case MAV_ROTATE_LANDING_DIR_TYPE_UP_TO_10_DEG:
    case MAV_ROTATE_LANDING_DIR_TYPE_UP_TO_30_DEG:
    case MAV_ROTATE_LANDING_DIR_TYPE_UP_TO_45_DEG:
    case MAV_ROTATE_LANDING_DIR_TYPE_UP_TO_90_DEG:
    case MAV_ROTATE_LANDING_DIR_TYPE_90_DEG_ONLY:
    case MAV_ROTATE_LANDING_DIR_TYPE_CROSS:
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: unsupported type %d", cmd.content.rotate_landing.type);
        return;

    default:
        gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: unknown type %d", cmd.content.rotate_landing.type);
        return;
    }


    // PROCEDURE:
    // new_land_wp = land_wp + offset
    // rotate new_land_wp around land_wp
    // overwrite land_wp with new_land_wp
    // LOOP for all nav_wp
    // -> temp_wp = nav_wp + offset
    // -> rotate temp_wp around new_land_wp
    // -> overwrite nav_wp with temp_to

    // generate a land_cmd by rotating and offsetting it by itself
    offset_then_rotate(rotation_angle, approach_bearing, offset, land_cmd.content.location, land_cmd.content.location);
    gcs_send_text_fmt(MAV_SEVERITY_CRITICAL, "do_rotate_landing: new bearing %.1f, offset %dm", (double)approach_bearing, offset);

    land_approach_rotation.start_index = index;
    land_approach_rotation.land_index = land_cmd.index;
    land_approach_rotation.angle = rotation_angle;
    land_approach_rotation.offset = offset;
    land_approach_rotation.bearing = approach_bearing;
    land_approach_rotation.land_wp = land_cmd.content.location;
}

void Plane::offset_then_rotate( const float rotation_angle,
                                const float approach_bearing,
                                const int16_t offset,
                                const Location locA,
                                Location& locB) {
    // PROCEDURE:
    // new_land_wp = land_wp + offset
    // rotate new_land_wp around land_wp
    // overwrite land_wp with new_land_wp
    // LOOP for all nav_wp
    // -> temp_wp = nav_wp + offset
    // -> rotate temp_wp around new_land_wp
    // -> overwrite nav_wp with temp_to

    // apply offset then rotate
    location_update(locB, approach_bearing, offset);
    rotate_location_around_another_location(rotation_angle, locA, locB);
}

/*
 * rotate Location B around Location A (stationary).
 * bearing = 0 or 360 means location B does not change
 */
void Plane::rotate_location_around_another_location(const float rotation_angle, const Location locA, Location& locB)
{
    // traverse locB to locA
    float bearing_cd = get_bearing_cd(locB, locA);
    float distance = get_distance(locA, locB);
    location_update(locB, bearing_cd * 0.01f, distance);

    // rotate locB by (rotation_angle-180) degrees. The -180 is because at this point
    // "0" means no change (keep traversing forward) which is an effective 180deg
    // reflection and 180 means go back where we started from
    float effective_rotation_angle = rotation_angle - 180;
    bearing_cd += (int32_t)(effective_rotation_angle * 100);
    bearing_cd = wrap_180_cd(bearing_cd);

    // traverse locB away from locA
    location_update(locB, bearing_cd * 0.01f, distance);
}


