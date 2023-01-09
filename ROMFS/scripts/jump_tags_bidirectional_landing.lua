
local MAV_SEVERITY_NOTICE=5    -- An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
local MAV_SEVERITY_INFO=6      -- Normal operational messages. Useful for logging. No action is required for these messages.


local MAV_CMD_NAV_LAND = 21


local MISSION_TAG_DETERMINE_LAND_DIRECTION = 200
local MISSION_TAG_LAND1_START = 300
local MISSION_TAG_LAND1_START_REVERSED = 301

local tag_check_complete_wind_dir = false

local run_once = true


local function get_index_of_next_land()
    local index_start = mission:get_index_of_jump_tag(MISSION_TAG_LAND1_START)
    if (not index_start) or (index_start == 0) then
        index_start = mission:get_current_nav_index()
    end

    for index = index_start, mission:num_commands()-1 do
        local mitem = mission:get_item(index)
        if (mitem) and (mitem:command() == MAV_CMD_NAV_LAND) then
            return index
        end
    end
    return 0
end


local function check_wind_and_jump_to_INTO_wind_landing()
    local index_land = get_index_of_next_land()

    if (index_land <= 1) then
        -- failed to get landing sequence start
        return
    end

    local mitem1 = mission:get_item(index_land - 1)
    local mitem2 = mission:get_item(index_land)
    if (not mitem1) or (not mitem2) then
        -- failed to get the two waypoints of the approach to calc the direction
        return
    end

    local wp1 = Location()
    wp1:lat(mitem1:x())
    wp1:lng(mitem1:y())

    local wp2 = Location()
    wp2:lat(mitem2:x())
    wp2:lng(mitem2:y())

    local bearing = wp1:get_bearing(wp2)
    local wind = ahrs:wind_estimate() -- wind vector3f in m/s
    local tail_wind = (math.sin(bearing) * wind:y()) + (math.cos(bearing) * wind:x())

    -- we need at least 10 cm/s of tailwind. With very little wind (or a noisy 0 value) we don't want to flip around.
    local tail_wind_threshold = 0.1
    if (tail_wind > tail_wind_threshold) then
        -- jump mission to other into-wind landing direction
        gcs:send_text(MAV_SEVERITY_INFO, "LUA: jump mission to reverse direction")
        if (not mission:jump_to_tag(MISSION_TAG_LAND1_START_REVERSED)) then
            gcs:send_text(MAV_SEVERITY_NOTICE, string.format("LUA: jump_to_tag %u failed", jump_to_tag))
        end
    else
        gcs:send_text(MAV_SEVERITY_INFO, "LUA: continuing with normal landing direction")
        if (not mission:jump_to_tag(MISSION_TAG_LAND1_START)) then
        -- fail quietly, this tag is optional. If not found, behavior is to continue on with current waypoint
        -- gcs:send_text(MAV_SEVERITY_NOTICE, string.format("LUA: jump_to_tag %u failed", jump_to_tag))
        end
    end
end


local function update()
    if (run_once) then
        gcs:send_text(MAV_SEVERITY_INFO, "LUA: SCRIPT START: Check Wind for Reverse Landing")
        run_once = false
    end

    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    local current_tag = mission:get_current_tag()
    if (not current_tag) or (current_tag ~= MISSION_TAG_DETERMINE_LAND_DIRECTION) then
        tag_check_complete_wind_dir = false
    elseif (tag_check_complete_wind_dir == false) then
        -- we're executing the tag, run once the check once
        tag_check_complete_wind_dir = true
        check_wind_and_jump_to_INTO_wind_landing()
    end

    return update, 1000
end


return update(), 1000

