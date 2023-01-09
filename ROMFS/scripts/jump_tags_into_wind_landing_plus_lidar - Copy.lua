

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
local MAV_CMD_JUMP_TAG = 600
local MAV_CMD_DO_JUMP_TAG = 601

local ROTATION_PITCH_270 = 25


local MISSION_TAG_LIDAR = 100
local MISSION_TAG_DETERMINE_LAND_DIRECTION = 200
local MISSION_TAG_LAND1_START = 300
local MISSION_TAG_LAND1_START_REVERSED = 301

local lidar_sample_count = 0
local lidar_samples_sum = 0

local mission_timestamp_ms = 0
local mission_type_matches_this_script = false


function reset()
    -- gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: RESET"))
    lidar_sample_count = 0
    lidar_samples_sum = 0
end

function detect_if_mission_changed()

    local current_timestamp = mission:last_change_time_ms()
    if (mission_timestamp_ms == current_timestamp) then
        return
    end
    mission_timestamp_ms = current_timestamp

    reset()
    detect_mission_type(false)
    if (mission_type_matches_this_script) then
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Detected Mission Change"))
    end
end


function get_index_of_next_land()
    for index = mission:get_current_nav_index(), mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if (mitem) and (mitem:command() == MAV_CMD_NAV_LAND) then
            return index
        end
    end
    return 0
end


function check_wind_and_jump_to_INTO_wind_landing()
    local current_tag = mission:get_current_tag()
    if (not current_tag) or (current_tag ~= MISSION_TAG_DETERMINE_LAND_DIRECTION) then
        -- we're not at the decision point yet
        return
    end
    mission:invalidate_current_tag()

    local index_land = get_index_of_next_land()

    if (index_land == 0) then
        return
    end

    local mitem1 = mission:get_item(index_land - 1)
    local mitem2 = mission:get_item(index_land)
    if (not mitem1) or (not mitem2) then
        return
    end

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
        gcs:send_text(MAV_SEVERITY_foo, "LUA: jump mission to reverse direction")
        if (not mission:jump_to_tag(MISSION_TAG_LAND1_START_REVERSED)) then
            gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: jump_to_tag %u failed", jump_to_tag))
        end
    else 
        gcs:send_text(MAV_SEVERITY_foo, "LUA: continuing with normal landing direction")
    end
end

function sample_lidar_to_get_AGL()

    local current_tag = mission:get_current_tag()
    if (not current_tag) or (current_tag ~= MISSION_TAG_LIDAR) then
        -- we're not at the decision point
        if (lidar_sample_count > 0) then
            -- except we've recently been sampling so we must be done!
            local lidar_average_final = lidar_samples_sum / lidar_sample_count
            lidar_sample_count = 0
            lidar_samples_sum = 0
            update_baro(lidar_average_final)
        end
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

    if (lidar_sample_count <= 1) then
        lidar_sample_count = 1 -- quick divide-by-zero sanity check
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurements started"))
    end
    local lidar_average = lidar_samples_sum / lidar_sample_count
    gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurement %u: %.2fm, avg: %.2f", lidar_sample_count, lidar_corrected_for_attitude, lidar_average))
end

function update_baro(lidar_average)

    -- finished sampling, use the result to offset baro
    gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: Lidar measurements stopped: samples = %d, avg = %.2fm", lidar_sample_count, lidar_average))

    local ekf_agl_m = ahrs:get_hagl()
    if (not ekf_agl_m) then
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: unable to get current AGL so we can not correct baro"))
        return
    end

    local alt_error_m = ekf_agl_m - lidar_average
    gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: alt_error is: %.2f - %.2f = %.2f", ekf_agl_m, lidar_average, alt_error_m))

    local baro_alt_offset = param:get('BARO_ALT_OFFSET')
    local baro_alt_offset_new_value = baro_alt_offset + alt_error_m
    gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: BARO_ALT_OFFSET changed from %.2f to %.2f", baro_alt_offset, baro_alt_offset_new_value))
    param:set('BARO_ALT_OFFSET', baro_alt_offset_new_value)
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

    sample_lidar_to_get_AGL()
    check_wind_and_jump_to_INTO_wind_landing()

    return update, 1000
end

function detect_mission_type(force_gcs_msg)

    for index = 1, mission:num_commands()-1 do
        mitem = mission:get_item(index)
        if not mitem then
            return
        end

        if (mitem:command() == MAV_CMD_JUMP_TAG) then
            if (not mission_type_matches_this_script or force_gcs_msg) then
                mission_type_matches_this_script = true
                gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: mission check JUMP_TAG is running"))
            end
            return
        end
    end

    -- not supported
    if (mission_type_matches_this_script or force_gcs_msg) then
        mission_type_matches_this_script = false
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: mission check JUMP_TAG is snoozing"))
    end
end

gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: SCRIPT START: mission check JUMP_TAG"))
detect_mission_type(true)
return update() -- run immediately before starting to reschedule

