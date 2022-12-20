

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



local MISSION_TAG_DETERMINE_LAND_DIRECTION = 200
local MISSION_TAG_LAND1_START = 300
local MISSION_TAG_LAND1_START_REVERSED = 301

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

    local index_land = get_index_of_next_land()

    if (index_land <= 1) then
        return
    end

    local mitem1 = mission:get_item(index_land - 1)
    local mitem2 = mission:get_item(index_land)
    if (not mitem1) or (not mitem2) then
        return
    end

    local wp1 = Location()
    wp1:lat(mitem1:x())
    wp1:lng(mitem1:y())

    local wp2 = Location()
    wp2:lat(mitem2:x())
    wp2:lng(mitem2:y())

    local bearing = wp1:get_bearing(wp2)
    local wind = ahrs:wind_estimate()
    local tail_wind = (math.sin(bearing) * wind:y()) + (math.cos(bearing) * wind:x())

    -- we need at least 10 cm/s of tailwind. With very little wind (or a noisy 0 value) we don't want to flip around.
    local tail_wind_threshold = 0.1
    local jump_to_tag = 0
    if (tail_wind > tail_wind_threshold) then
        -- jump mission to other into-wind landing direction
        gcs:send_text(MAV_SEVERITY_foo, "LUA: jump mission to other into-wind landing direction")
        jump_to_tag = MISSION_TAG_LAND1_START_REVERSED
    else
        gcs:send_text(MAV_SEVERITY_foo, "LUA: continuing with normal landing direction")
        jump_to_tag = MISSION_TAG_LAND1_START
    end
    if (not mission:jump_to_tag(jump_to_tag)) then
        gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: jump_to_tag %u failed", jump_to_tag))
    end
end

function update()
    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    check_wind_and_jump_to_INTO_wind_landing()

    return update, 1000
end

gcs:send_text(MAV_SEVERITY_foo, string.format("LUA: SCRIPT START: mission check JUMP_TAG"))
return update() -- run immediately before starting to reschedule

