-- This script is designed to fade in throttle command during takeoff
-- to prevent prop stall at low airspeed_estimate
-- Ryan Beall and Tom Pittenger 9AUG22

local FLIGHT_MODE_PLANE_AUTO = 10
local MAV_CMD_NAV_TAKEOFF = 22
local K_THROTTLE = 70

local rotate_speed = assert(param:get("TKOFF_ROTATE_SPD"), "Could not read param TKOFF_ROTATE_SPD")
local takeoff_throttle_max = assert(param:get("TKOFF_THR_MAX"), "Could not read param TKOFF_THR_MAX")
local throttle_srv_chan = assert(SRV_Channels:find_channel(K_THROTTLE), "Could not find Servo channel K_THROTTLE " .. K_THROTTLE)
local throttle_max_pwm = assert(param:get("SERVO" .. throttle_srv_chan + 1 .. "_MIN"), "Could not read param SERVO" .. throttle_srv_chan + 1 .. "_MIN")
local throttle_min_pwm = assert(param:get("SERVO" .. throttle_srv_chan + 1 .. "_MAX"), "Could not read param SERVO" .. throttle_srv_chan + 1 .. "_MAX")

local throttle_scale = throttle_max_pwm - throttle_min_pwm

if takeoff_throttle_max == 0 then
    takeoff_throttle_max = assert(param:get("THR_MAX"), "Could not read THR_MAX")
end

takeoff_throttle_max = takeoff_throttle_max*0.01
-- set lower bound to 20% of max throttle when airspeed is zero
local takeoff_throttle_min = 0.2*takeoff_throttle_max
-- max throttle is unrestricted when airspeed is 90% of rotate speed
rotate_speed = rotate_speed*0.9

gcs:send_text(0,"K1000: Takeoff Thrust Limiter Enabled")

function update()
    if (vehicle:get_mode() == FLIGHT_MODE_PLANE_AUTO) and (mission:get_current_nav_id() == MAV_CMD_NAV_TAKEOFF) and arming:is_armed() then
        -- sanity check airspeed sensor
        local airspeed = ahrs:airspeed_estimate()
        if (airspeed == nil) then
            gcs:send_text(0, "Takeoff Thrust Limiting failure: no Airspeed")
            return update, 10
        end
        -- calculate throttle limiter based on airspeed as a function of a square root law
        local throttle_max = takeoff_throttle_max*math.sqrt((airspeed/rotate_speed)*(1-takeoff_throttle_min)*(1-takeoff_throttle_min))+takeoff_throttle_min

        throttle_max = math.max(throttle_max, takeoff_throttle_min) -- saturate to max of 1.0
        throttle_max = math.min(throttle_max, takeoff_throttle_max) -- saturate to a min of 0.0

        throttle_max = (throttle_scale*throttle_max) + throttle_min_pwm -- scale back to pwm values
        throttle_max = math.floor(throttle_max + 0.5) -- turn into an integer value

        -- override throttle channel, if lua doesn't return faster
        -- than 15us then normal throttle will take back over
        SRV_Channels:set_output_pwm_chan_timeout(throttle_srv_chan,throttle_max,15)
        return update, 10

    else
        return update,100
    end
end

return update()
