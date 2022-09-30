-- Forward Motor Loss of Thrust Detector
-- Detects if throttle is above given threshold with
-- with lower than nominal vibrations
-- Ryan Beall 20AUG2022

local K_THROTTLE = 70
local motor_out_time = 0
local case = 0
local DISARMED = 0
local MOTOR_OPER = 1
local MOTOR_INOP_WAITING = 2
local MOTOR_INOP_ALERT = 3
local THROTTLE_MAX = assert(param:get("THR_MAX"),"Lua: Could not read THR_MAX")

-- consider motor stopped when vibe is low and RPM low for more than 4s
local MOTOR_STOPPED_MS = 4000
-- vibration threshold below which motor may be stopped
local VIBE_LOW_THRESH = 1.25
-- Throttle Threshold % above throttle max which motor should be considered valid to check vibes
local THROTTLE_ON_THRESH = 90 * THROTTLE_MAX * 0.01

gcs:send_text(4, "K1000: Deadstick Monitoring Enabled")

function update()
    if case == DISARMED then
        if arming:is_armed() then
            case = MOTOR_OPER
        end
        return update, 1000
    end
    if case == MOTOR_OPER then
        if not arming:is_armed() then
            case = DISARMED
            return update, 1000
        end

        local throttle = SRV_Channels:get_output_scaled(K_THROTTLE)
        local vibe = ahrs:get_vibration():length()

        if (vibe < VIBE_LOW_THRESH) and (throttle > THROTTLE_ON_THRESH) then
            motor_out_time = millis()
            case = MOTOR_INOP_WAITING
        end
        return update, 200
    end
    if case == MOTOR_INOP_WAITING then
        if not arming:is_armed() then
            case = DISARMED
            return update, 1000
        end

        local throttle = SRV_Channels:get_output_scaled(K_THROTTLE)
        local vibe = ahrs:get_vibration():length()

        if (vibe > VIBE_LOW_THRESH) or (throttle < THROTTLE_ON_THRESH) then
            case = MOTOR_OPER
            gcs:send_text(0, "K1000: Motor STARTED")
            return update, 200
        end
        if millis() > (motor_out_time + MOTOR_STOPPED_MS) then
            case = MOTOR_INOP_ALERT
        end
        return update, 200
    end
    if case == MOTOR_INOP_ALERT then
        if not arming:is_armed() then
            case = DISARMED
            return update, 1000
        end
        gcs:send_text(0, "WARNING: Loss of Thrust Possible")
        motor_out_time = millis()
        case = RESET_ESC
        return update, 200
    end
    if case == RESET_ESC then
        if not arming:is_armed() then
            case = DISARMED
            return update, 1000
        end
        gcs:send_text(0, "WARNING: Resetting ESC 1290us")
        param:set('SERVO1_MAX', 1290)
        case = MOTOR_INOP_RESETTING
        return update, 2000
    end
    if case == MOTOR_INOP_RESETTING
        if not arming:is_armed() then
            case = DISARMED
            return update, 1000
        end
        gcs:send_text(0, "WARNING: Resetting ESC 1800us")
        param:set('SERVO1_MAX', 1800)
        case = MOTOR_INOP_WAITING
        return update, 200
    end


end
-- start running update loop
return update()
