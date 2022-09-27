-- This script is designed to fade in throttle command during takeoff
-- to prevent prop stall at low airspeed_estimate
-- Ryan Beall and Tom Pittenger 9AUG22

local FLIGHT_MODE_PLANE_AUTO = 10
local MAV_CMD_NAV_TAKEOFF = 22
local K_THROTTLE = 70

local rotate_speed = assert(param:get("TKOFF_ROTATE_SPD"),"Lua: Could not read TKOFF_ROTATE_SPD")
local takeoff_throttle_max = assert(param:get("TKOFF_THR_MAX"),"Lua: Could not read TKOFF_THR_MAX")
local throttle_srv_chan = assert(SRV_Channels:find_channel(K_THROTTLE), "Could not find K_THROTTLE" .. K_THROTTLE)
local throttle_max_pwm = assert(param:get("SERVO" .. throttle_srv_chan + 1 .. "_MAX"),"Lua: Could not read SERVO" .. throttle_srv_chan + 1 .. "_MAX")
local throttle_min_pwm = assert(param:get("SERVO" .. throttle_srv_chan + 1 .. "_MIN"),"Lua: Could not read SERVO" .. throttle_srv_chan + 1 .. "_MIN")

local throttle_scale = throttle_max_pwm - throttle_min_pwm

if takeoff_throttle_max == 0 then
        takeoff_throttle_max = param:get("THR_MAX")
end

takeoff_throttle_max = takeoff_throttle_max/100
-- set lower bound to 20% of max throttle when airspeed is zero
local takeoff_throttle_min = 0.2*takeoff_throttle_max
-- max throttle is unrestricted when airspeed is 90% of rotate speed
rotate_speed = rotate_speed*0.9

gcs:send_text(4, "K1000: Propeller Stall Limiter Enabled")

local j = 0

function update()

    local nav_command = mission:get_current_nav_id()
    local mode = vehicle:get_mode()

    if (nav_command == MAV_CMD_NAV_TAKEOFF) and (mode == FLIGHT_MODE_PLANE_AUTO) and arming:is_armed() then
        local airspeed = ahrs:airspeed_estimate()
        if airspeed == nil or rotate_speed == 0 then
            gcs:send_text(5,"K1000: takeoff_throttle.lua failing to limit throttle")
            return update, 10
        end

        local throttle_max = 1*math.sqrt((airspeed/rotate_speed)*(1-takeoff_throttle_min)*(1-takeoff_throttle_min))+takeoff_throttle_min

        throttle_max = math.max(throttle_max, takeoff_throttle_min) -- saturate to max of 1.0
        throttle_max = math.min(throttle_max, takeoff_throttle_max) -- saturate to a min of 0.0

        throttle_max = throttle_scale*throttle_max+throttle_min_pwm -- scale back to pwm values
        throttle_max = math.floor(throttle_max + 0.5) -- turn into an integer value

        --SRV_Channels:set_output_pwm_chan_timeout(throttle_srv_chan,throttle_max,15) -- override throttle channel
        logger:write('SCR1','throttle_max,airspeed','ff')

        return update, 10
    else
        return update, 500
    end

end

return update()
