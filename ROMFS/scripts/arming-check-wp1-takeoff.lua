-- This script runs a custom arming check to ensure you're running a takeoff missionn item

local auth_id = arming:get_aux_auth_id()

local MAV_CMD_NAV_TAKEOFF = 22
local MAV_CMD_NAV_VTOL_TAKEOFF = 84

function update() -- this is the loop which periodically runs
  if auth_id then
    local cmd_id = mission:get_current_nav_id()

    if cmd_id == 0 then
      -- invalid waypoint index
      if mission:get_current_nav_index() == 0 and mission:set_current_cmd(1) then
        -- at boot, we're executing wp index 0, which is invalid. When switching to AUTO
        -- it will automatically jump to wp 1. So, for the sake of automagic arming tests,
        -- lets jump to wp1 to allow proper mission cmd sanity checks.
        arming:set_aux_auth_failed(auth_id, "Mission not available")
      else
        arming:set_aux_auth_failed(auth_id, "Could not retrieve mission")
      end
    elseif ((cmd_id ~= MAV_CMD_NAV_TAKEOFF) and (cmd_id ~= MAV_CMD_NAV_VTOL_TAKEOFF)) then
      arming:set_aux_auth_failed(auth_id, "Mission is not ready to takeoff")
    else
      arming:set_aux_auth_passed(auth_id)
    end
  end
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule


