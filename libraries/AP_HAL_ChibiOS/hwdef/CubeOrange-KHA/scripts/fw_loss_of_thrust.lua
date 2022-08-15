-- This script is designed to detect loss of thrust
-- on forward motor

local motor_stop_ms = -1
local motor_stopped = false
gcs:send_text(4, "K1000: Deadstick Monitoring Enabled")

-- update motor running status
function check_motor()
  if not arming:is_armed() then
     motor_stopped = false
     motor_stop_ms = -1
     return true
  end
  local throttle = SRV_Channels:get_output_scaled(70)
  local vibe = ahrs:get_vibration():length()


  -- consider motor stopped when vibe is low and RPM low for more than 4s
  local MOTOR_STOPPED_MS = param:get('SCR_MTR_STOP_MS')
  -- vibration threshold below which motor may be stopped
  local VIBE_LOW_THRESH = param:get('SCR_VIBE_LOW')
  -- Throttle Threshold above which motor should be considered valid to check vibes
  local THROTTLE_ON_THRESH = param:get('SCR_THR_THRESH')
  -- time when motor stopped
  
  -- if Throttle is above 90% and vibe is high then assume motor is running
  if  (vibe > VIBE_LOW_THRESH) then
     -- motor is definately running
     motor_stop_ms = -1
     if motor_stopped then
        -- notify user motor has started
        gcs:send_text(0, "motor: STARTED")
        motor_stopped = false
     end
     return true
  end

  if (throttle > THROTTLE_ON_THRESH) and (vibe < VIBE_LOW_THRESH) then
   local now = millis()
   if motor_stop_ms == -1 then
       -- start timeout period
       motor_stop_ms = now
       return true
    end
    if now - motor_stop_ms < MOTOR_STOPPED_MS then
       return false
    end
    -- motor has been stopped for MOTOR_STOPPED_MS milliseconds, notify user
    if not motor_stopped then
       motor_stopped = true
       gcs:send_text(0, "WARNING: Loss of Thrust Possible")
    end
  end
  return motor_stopped
end
function update()
  -- check motor status
  check_motor()
  -- run at 5Hz
  return update, 200
end
-- start running update loop
return update()
