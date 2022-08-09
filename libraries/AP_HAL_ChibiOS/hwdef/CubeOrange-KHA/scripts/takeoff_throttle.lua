-- This script is designed to fade in throttle command during takeoff
-- to prevent prop stall at low airspeed_estimate
-- Ryan Beall 9AUG22

local rotate_speed = assert(param:get("TKOFF_ROTATE_SPD"),"Lua: Could not read TKOFF_ROTATE_SPD")
local takeoff_throttle_max = assert(param:get("TKOFF_THR_MAX"),"Lua: Could not read TKOFF_THR_MAX")
local throttle_max_pwm = assert(param:get("SERVO1_MAX"),"Lua: Could not read SERVO1_MAX")
local throttle_min_pwm = assert(param:get("SERVO1_MIN"),"Lua: Could not read SERVO1_MIN")

local throttle_scale = throttle_max_pwm - throttle_min_pwm

local throttle_channel = SRV_Channels:find_channel(70) -- find throttle channel
local RC_Throttle = rc:get_channel(throttle_channel)

if takeoff_throttle_max == 0 then
        takeoff_throttle_max = param:get("THR_MAX")
end

takeoff_throttle_max = takeoff_throttle_max/100
local takeoff_throttle_min = 0.2*takeoff_throttle_max

function update()

    local nav_command = mission:get_current_nav_id()
    local mode = vehicle:get_mode()

    if (nav_command == 22) and (mode == 10) and arming:is_armed() then
        local airspeed = ahrs:airspeed_estimate()
        local throttle_max = takeoff_throttle_max*math.sqrt((airspeed/rotate_speed)*(1-takeoff_throttle_min)*(1-takeoff_throttle_min))+takeoff_throttle_min
        throttle_max = throttle_max

        throttle_max = math.max(throttle_max, takeoff_throttle_min)
        throttle_max = math.min(throttle_max, takeoff_throttle_max)

        throttle_max = throttle_scale*throttle_max+throttle_min_pwm
        throttle_max = math.floor(throttle_max + 0.5) -- turn into an integer value

        SRV_Channels:set_output_pwm_chan_timeout(1-1,throttle_max,15)

        return update, 10
    else
        return update, 500
    end

end

return update()
