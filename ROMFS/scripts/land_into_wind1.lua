--[[
DO_LAND_START
WAYPOINT (high alt, cruise alt)
LOITER_TO_ALT (low alt, 60m)

WAYPOINT_START (same pos as WAYPOINT_END)
WAYPOINT
WAYPOINT

WAYPOINT (same pos as LAND but at 60m. DO_LAND are index+11 and index+21)
	(start lidar)
WAYPOINT
	(end lidar, read-add-write to BARO_ALT_OFFSET. Don't set it, must add to it)
WAYPOINT
WAYPOINT
WAYPOINT
WAYPOINT_END (same pos as WAYPOINT_START)


-------------
LOITER_TO_ALT
WAYPOINT
WAYPOINT
WAYPOINT
WAYPOINT
	(calculate heading between these two points)
LAND
CONTINUE_AND_CHANGE_ALT
WAYPOINT
WAYPOINT
RTL

-------------
LOITER_TO_ALT
WAYPOINT
WAYPOINT
WAYPOINT
WAYPOINT
LAND
CONTINUE_AND_CHANGE_ALT
WAYPOINT
WAYPOINT
RTL
--]]

local MAV_SEVERITY_EMERGENCY=0 -- System is unusable. This is a "panic" condition.
local MAV_SEVERITY_ALERT=1     -- Action should be taken immediately. Indicates error in non-critical systems.
local MAV_SEVERITY_CRITICAL=2  -- Action must be taken immediately. Indicates failure in a primary system.
local MAV_SEVERITY_ERROR=3     -- Indicates an error in secondary/redundant systems.
local MAV_SEVERITY_WARNING=4   -- Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. 
local MAV_SEVERITY_NOTICE=5    -- An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
local MAV_SEVERITY_INFO=6      -- Normal operational messages. Useful for logging. No action is required for these messages.
local MAV_SEVERITY_DEBUG=7     -- Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
local MAV_SEVERITY_ENUM_END=8

local MAV_SEVERITY_foo = MAV_SEVERITY_NOTICE


local MAV_CMD_NAV_WAYPOINT = 16
local MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
local MAV_CMD_NAV_LAND = 21
local MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30
local MAV_CMD_NAV_LOITER_TO_ALT = 31
local MAV_CMD_DO_LAND_START = 189
local MAV_CMD_DO_AUX_FUNCTION = 218
local MAV_CMD_JUMP_TAG = 600
local MAV_CMD_DO_JUMP_TAG = 601

local ROTATION_PITCH_270 = 25



local wp_lidar_start = 0
local wp_lidar_stop = 0
local wp_decide_landing_direction_index = 0
local wp_land1_sequence_start = 0
local wp_land1_index = 0
local wp_land2_sequence_start = 0
local lidar_sample_count = 0
local lidar_samples_sum = 0

local landing_direction_has_been_chosen = false
local baro_has_been_updated = false

local mission_count = 0
local mission_type_matches_this_script = false

local wp_land1_x = 0
local wp_land1_y = 0


function reset()
    -- gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: RESET"))

    landing_direction_has_been_chosen = false
    wp_land1_sequence_start = 0
    wp_land2_sequence_start = 0
    
    wp_lidar_start = 0
    wp_lidar_stop = 0
    lidar_sample_count = 0
    lidar_samples_sum = 0
    baro_has_been_updated = false

    wp_land1_index = 0
    wp_land1_x = 0
    wp_land1_y = 0
    wp_decide_landing_direction_index= 0
end

function detect_if_mission_changed()
    -- try and detect if the mission has changed
    local has_changed = false
    local mission_count_new = mission:num_commands()-1
    if (mission_count ~= mission_count_new) then
        mission_count = mission_count_new
        has_changed = true
    end

    if (has_changed) then
        reset()
        detect_mission_type(false)
        if (mission_type_matches_this_script) then
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Detected Mission Change"))
        end
    end
end

function determine_waypoint_indexes()

    if (wp_lidar_start > 0) and (wp_lidar_stop > 0) and (wp_decide_landing_direction_index > 0) and (wp_land1_sequence_start > 0) and (wp_land1_index > 2) and (wp_land2_sequence_start > 0) then
        return
    end

    local most_recent_loiter_to_alt_index = 0
    for index = 1, mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if not mitem then
            return
        end

        if (mitem:command() == MAV_CMD_NAV_LOITER_TO_ALT) then
            most_recent_loiter_to_alt_index = index

        elseif (mitem:command() == MAV_CMD_NAV_LAND) and (wp_land1_sequence_start == 0) then
            wp_land1_index = index
            wp_land1_x = mitem:x()
            wp_land1_y = mitem:y()
            wp_land1_sequence_start = most_recent_loiter_to_alt_index
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Found wp_land1_sequence_start = %d", wp_land1_sequence_start))

            wp_decide_landing_direction_index = wp_land1_sequence_start - 1
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Found wp_decide_landing_direction_index = %d", wp_decide_landing_direction_index))

        elseif (mitem:command() == MAV_CMD_NAV_RETURN_TO_LAUNCH) and (wp_land1_sequence_start > 0) and (wp_land2_sequence_start == 0) then
            wp_land2_sequence_start = index + 1
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Found wp_land2_sequence_start = %d", wp_land2_sequence_start))
        elseif (mitem:command() == MAV_CMD_NAV_WAYPOINT) and (wp_lidar_start == 0) and (mitem:x() == wp_land1_x) and (mitem:y() ==  wp_land1_y) then
            wp_lidar_start = index
            wp_lidar_stop = index + 2
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Found wp_lidar_start,_stop = %d, %d", wp_lidar_start, wp_lidar_stop))
        end

        if (wp_land1_index > 2) and (wp_land1_sequence_start > 0) and (wp_land2_sequence_start > 0) and (wp_lidar_start > 0) then
            -- quit early if we know all we need to know
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: found everything we need!"))
            return
        end
    end
end


function check_wind_and_jump_to_INTO_wind_landing()

    if (landing_direction_has_been_chosen) or (wp_land1_sequence_start == 0) or (wp_land2_sequence_start == 0) or (wp_decide_landing_direction_index == 0) or (wp_land1_index <= 2) then
        -- we're not ready to calculate this
        return
    end

    if (mission:get_current_nav_index() ~= wp_decide_landing_direction_index) then
        -- we're not at the decision point yet
        return
    end

    landing_direction_has_been_chosen = true;

    local mitem1 = mission:get_item(wp_land1_index - 1)
    local mitem2 = mission:get_item(wp_land1_index)
    local wp1 = Location()
    local wp2 = Location()

    wp1:lat(mitem1:x())
    wp1:lng(mitem1:y())

    wp2:lat(mitem2:x())
    wp2:lng(mitem2:y())

    local bearing = wp1:get_bearing(wp2)

    -- wind is in NED, convert for readability
    local wind = ahrs:wind_estimate()
    local tail_wind = (math.sin(bearing) * wind:y()) + (math.cos(bearing) * wind:x())

    -- we need at least 10 cm/s of tailwind. With very little wind (or a noisy 0 value) we don't want to flip around.
    local tail_wind_threshold = 0.1
    if (tail_wind > tail_wind_threshold) then
        -- jump mission to other into-wind landing direction
        gcs:send_text(MAV_SEVERITY_foo, "LUA: jump mission to other into-wind landing direction")
        mission:set_current_cmd(wp_land2_sequence_start)
    else
        mission:set_current_cmd(wp_land1_sequence_start)
        gcs:send_text(MAV_SEVERITY_foo, "LUA: continuing with normal landing direction")
    end
end

function use_lidar_to_update_baro_if_necessary()

    if (baro_has_been_updated) or (wp_lidar_start == 0) or (wp_lidar_stop == 0) then
        -- we're not ready to calculate this or we already have done the job
        return
    end

    local current_index = mission:get_current_nav_index()
    if (current_index < wp_lidar_start) or (current_index > wp_lidar_stop) then
        -- local reset
        lidar_samples_sum = 0
        lidar_sample_count = 0
        return
    end

    if (not rangefinder:has_data_orient(ROTATION_PITCH_270)) then
        -- rangefinder not ready
        return
    end

    -- we're actively sampling rangefinder distance to ground
    local lidar_raw = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01

    -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
    local ahrs_get_rotation_body_to_ned_c_z = math.cos(ahrs:get_roll())*math.cos(ahrs:get_pitch())
    local lidar_corrected_for_attitude = lidar_raw * ahrs_get_rotation_body_to_ned_c_z

    lidar_samples_sum = lidar_samples_sum + lidar_corrected_for_attitude
    lidar_sample_count = lidar_sample_count + 1

    if (lidar_sample_count == 0) then
        -- divide-by-zero sanity check
        return
    elseif (lidar_sample_count == 1) then
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurements started"))
    end

    local lidar_average = lidar_samples_sum / lidar_sample_count
    
    gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurement: %.2fm, avg: %.2f", lidar_corrected_for_attitude, lidar_average))

    if (current_index == wp_lidar_stop) then
        baro_has_been_updated = true

        -- finished sampling, use the result to offset baro
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurements stopped: samples = %d, avg = %.2fm", lidar_sample_count, lidar_average))

        local ekf_agl_m = ahrs:get_hagl()
        if (not ekf_agl_m) then
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: unable to get current AGL so we can not correct baro"))
            return
        end

        local alt_error_m = ekf_agl_m - lidar_average
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: current_alt - alt_error is: %.2f - %.2f = %.2f", ekf_agl_m, lidar_average, alt_error_m))

        local baro_alt_offset = param:get('BARO_ALT_OFFSET')
        local baro_alt_offset_new_value = baro_alt_offset + alt_error_m
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: BARO_ALT_OFFSET changed from %.2f to %.2f", baro_alt_offset, baro_alt_offset_new_value))
        param:set('BARO_ALT_OFFSET', baro_alt_offset_new_value)
    end
end

function update()

    detect_if_mission_changed()
    if (not mission_type_matches_this_script) then
        return update, 5000
    end

    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        reset()
        return update, 5000
    end


    determine_waypoint_indexes()

    use_lidar_to_update_baro_if_necessary()

    check_wind_and_jump_to_INTO_wind_landing()

    return update, 1000
end

function detect_mission_type(force_gcs_msg)

    for index = 1, mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if not mitem then
            return
        end

        if (mitem:command() == MAV_CMD_DO_AUX_FUNCTION) or (mitem:command() == MAV_CMD_JUMP_TAG) or (mitem:command() == MAV_CMD_DO_JUMP_TAG) then
            -- not supported
            if (mission_type_matches_this_script or force_gcs_msg) then
                mission_type_matches_this_script = false
                gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: mission check STANDARD is snoozing"))
            end
            return
        end
    end

    if (not mission_type_matches_this_script or force_gcs_msg) then
        mission_type_matches_this_script = true
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: mission check STANDARD is running"))
    end
end

gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: SCRIPT START: mission check STANDARD"))
detect_mission_type(true)
return update() -- run immediately before starting to reschedule

