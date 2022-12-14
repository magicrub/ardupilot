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


local MAV_CMD_NAV_WAYPOINT = 16
local MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
local MAV_CMD_NAV_LAND = 21
local MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30
local MAV_CMD_NAV_LOITER_TO_ALT = 31
local MAV_CMD_DO_LAND_START = 189

local ROTATION_PITCH_270 = 25

local landing_direction_has_been_chosen = false

local wp_land1_sequence_start = 0
local wp_land2_sequence_start = 0
local wp_land1_index = 0

local wp_lidar_start = 0
local wp_lidar_stop = 0
local lidar_sample_count = 0
local lidar_samples_sum = 0
local baro_has_been_updated = false

local mission_count = 0

function reset()
    landing_direction_has_been_chosen = false
    wp_land1_sequence_start = 0
    wp_land2_sequence_start = 0
    wp_land1_index = 0
    
    wp_lidar_start = 0
    wp_lidar_stop = 0
    lidar_sample_count = 0
    lidar_samples_sum = 0
    baro_has_been_updated = false
end

function detect_if_mission_changed()
    -- try and detect if the mission has changed
    local mission_count_new = mission:num_commands()-1
    if (mission_count ~= mission_count_new) then
        reset()
    end
    mission_count = mission_count_new
end

function determine_waypoint_indexes()

    if (wp_land1_index > 0 and wp_land1_sequence_start > 0 and wp_land2_sequence_start > 0 and wp_lidar_start > 0 and wp_lidar_stop > 0) then
        return
    end

    -- start fresh
    reset();

    local most_recent_loiter_to_alt_index = 0
    local wp_land1_x = 0
    local wp_land1_y = 0

    for x = 1, mission:num_commands()-1 do
        mitem = mission:get_item(x)
        if not mitem then
            return
        end

        if (mitem:command() == MAV_CMD_NAV_LOITER_TO_ALT) then
            most_recent_loiter_to_alt_index = x

        elseif (mitem:command() == MAV_CMD_NAV_LAND) and (wp_land1_sequence_start == 0) then
            wp_land1_index = x
            wp_land1_x = mitem:x()
            wp_land1_y = mitem:y()
            wp_land1_sequence_start = most_recent_loiter_to_alt_index
            gcs:send_text(0, string.format("Found wp_land1_sequence_start = %d", wp_land1_sequence_start))

        elseif (mitem:command() == MAV_CMD_NAV_RETURN_TO_LAUNCH) and (wp_land1_sequence_start > 0) and (wp_land2_sequence_start == 0) then
            wp_land2_sequence_start = x + 1
            gcs:send_text(0, string.format("Found wp_land2_sequence_start = %d", wp_land2_sequence_start))

        elseif (mitem:command() == MAV_CMD_NAV_WAYPOINT) and (wp_land1_index > 0) and (mitem:x() == wp_land1_x) and (mitem:y() == wp_land1_y) then
            wp_lidar_start = x - 1
            wp_lidar_stop = x + 2
            gcs:send_text(0, string.format("Found wp_lidar_start and _end indexes = %d, %d", wp_lidar_start, wp_lidar_stop))
        end
    end
end


function check_wind_and_jump_to_INTO_wind_landing()

    if (wp_land1_index == 0) or (wp_land2_sequence_start > 0) or (landing_direction_has_been_chosen) then
        -- we're nto ready to calculate this
        return
    end

    local current_index = mission:get_current_nav_index()

    if (current_index ~= (wp_land1_sequence_start - 1)) then
        -- we're not at the decision point yet
        return
    end

    landing_direction_has_been_chosen = true;
    
    local wp1 = mission:get_item(wp_land1_index - 1)
    local wp2 = mission:get_item(wp_land1_index)
    if (not wp1 or not wp2) then
        -- there's something wrong with the mission items
        return
    end

    local bearing = wp1:get_bearing(wp2)

    -- wind is in NED, convert for readability
    local wind = ahrs:wind_estimate()
    local wind_north = wind:x()
    local wind_east = wind:y()

    local tail_wind = (math.sin(bearing) * wind_east) + (math.cos(bearing) * wind_north)

    if (tail_wind > 0) then
        -- jump mission to other into-wind landing direction
        gcs:send_text(0, " jump mission to other into-wind landing direction")
        mission:set_current_cmd(wp_land2_sequence_start)
    else
        gcs:send_text(0, "continuing with normal landing direction")
    end
end

function use_lidar_to_update_baro_if_necessary()

    if baro_has_been_updated or (wp_lidar_start == 0) or (wp_lidar_stop == 0) then
        -- we're not ready to calculate this or we already have done the job
        return
    end

    local current_index = mission:get_current_nav_index()

    if (current_index <= wp_lidar_start) or (current_index > wp_lidar_stop) then
        -- local reset
        lidar_samples_sum = 0
        lidar_sample_count = 0
        return
    end


    if (rangefinder:has_data_orient(ROTATION_PITCH_270)) then
        -- we're actively sampling rangefinder distance to ground
        local lidar_raw = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01

        -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
        local ahrs_get_rotation_body_to_ned_c_z = math.cos(ahrs.get_roll)*math.cos(ahrs.get_pitch)
        local lidar_corrected_for_attitude = lidar_raw * ahrs_get_rotation_body_to_ned_c_z

        lidar_samples_sum = lidar_samples_sim + lidar_corrected_for_attitude
        lidar_sample_count = lidar_sample_count + 1
    end

    if (current_index == wp_lidar_stop) and (lidar_sample_count > 0) then
        -- finished sampling, use the result to offset baro
        local baro_alt_offset = param:get('BARO_ALT_OFFSET')
        local lidar_average = lidar_samples_sum / lidar_sample_count

        local baro_alt_offset_new_value = baro_alt_offset + lidar_average
        gcs:send_text(0, string.format("BARO_ALT_OFFSET changed from %.2f to %.2f", baro_alt_offset, baro_alt_offset_new_value))
        param:set('BARO_ALT_OFFSET', baro_alt_offset_new_value)

        baro_has_been_updated = true
    end
end

function update()

    if mission:state() ~= mission.MISSION_RUNNING or not arming:is_armed() or not vehicle:get_likely_flying() then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        reset()
        return update, 5000
    end

    if (mission:get_current_nav_id() == MAV_CMD_NAV_WAYPOINT) then
        -- all the logic and triggers only happen while we're doing a NAV_WAYPOINT
        detect_if_mission_changed()

        determine_waypoint_indexes()
    
        use_lidar_to_update_baro_if_necessary()
    
        check_wind_and_jump_to_INTO_wind_landing()
    end

    return update, 1000
end


return update() -- run immediately before starting to reschedule

