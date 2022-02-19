-- This script runs a custom arming check to ensure you're running a takeoff missionn item

local auth_id = arming:get_aux_auth_id()

local MAV_CMD_NAV_TAKEOFF = 22
local MAV_CMD_NAV_VTOL_TAKEOFF = 84

function update() -- this is the loop which periodically runs
  if auth_id then
    local cmd_id = mission:get_current_nav_id()

    if not cmd_id then
      arming:set_aux_auth_failed(auth_id, "Could not retrieve mission")
    elseif ((cmd_id ~= MAV_CMD_NAV_TAKEOFF) and (cmd_id ~= MAV_CMD_NAV_VTOL_TAKEOFF)) then
      arming:set_aux_auth_failed(auth_id, "Mission is not ready to takeoff")
    else
      arming:set_aux_auth_passed(auth_id)
    end
  end
  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule


