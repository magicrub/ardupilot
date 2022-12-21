--This script is designed to set the K1000P preflight params prior to takeoff
local FLIGHT_MODE_PLANE_AUTO = 10
local MAV_CMD_NAV_LAND = 21
local TAKEOFF_PARAMS_SET = false
local LANDING_PARAMS_SET = false

if param:get("Q_ENABLE") == 0 then
    if not arming:is_armed() then
        assert(param:set('CAM_TRIGG_DIST',0), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('FS_GCS_ENABL',1), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('FS_LONG_ACTN',1), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('FS_LONG_TIMEOUT',600), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_CLMB_MAX',3), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_CLMB_OPER',1.5), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_LAND_ARSPD',18), "K1000: Fixed Wing Landing Parameter Failed to Write")
        assert(param:set('TECS_SINK_MAX',3), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_SINK_OPER',0), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_SPDWEIGHT',1.5), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TECS_TCONST_STE',0.0), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('THR_MAX',70.0), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TKOFF_TDRAG_ELEV',-10), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('TKOFF_THR_MAX',63), "K1000: Fixed Wing Preflight Parameter Failed to Write")
        assert(param:set('WP_LOITER_RAD',150), "K1000: Fixed Wing Preflight Parameter Failed to Write")
    end
    gcs:send_text(4, "K1000: Fixed Wing Preflight Params Set")
end

function update()
    local nav_command = mission:get_current_nav_id()
    local mode = vehicle:get_mode()

    if (nav_command == MAV_CMD_NAV_TAKEOFF) and (mode == FLIGHT_MODE_PLANE_AUTO) and arming:is_armed() and TAKEOFF_PARAMS_SET == false then
        assert(param:set('THR_MAX',70.0), "K1000: Fixed Wing Takeoff Parameter Failed to Write")
        TAKEOFF_PARAMS_SET = true
        gcs:send_text(4, "K1000: Fixed Wing Takeoff Params Set")
    elseif not nav_command == MAV_CMD_NAV_TAKEOFF then
        TAKEOFF_PARAMS_SET = false
    end

    if (nav_command == MAV_CMD_NAV_LAND) and (mode == FLIGHT_MODE_PLANE_AUTO) and arming:is_armed() and LANDING_PARAMS_SET == false then
        assert(param:set('TECS_CLMB_OPER',0), "K1000: Fixed Wing Landing Parameter Failed to Write")
        assert(param:set('TECS_SINK_OPER',0), "K1000: Fixed Wing Landing Parameter Failed to Write")
        assert(param:set('THR_MAX',80.0), "K1000: Fixed Wing Landing Parameter Failed to Write")
        LANDING_PARAMS_SET = true
        gcs:send_text(4, "K1000: Fixed Wing Landing Params Set")
    elseif not nav_command == MAV_CMD_NAV_LAND then
        LANDING_PARAMS_SET = false
    end

    return update, 5000
end
-- start running update loop
return update()
