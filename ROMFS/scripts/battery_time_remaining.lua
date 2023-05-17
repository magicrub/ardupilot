
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PLANE_MODE_MANUAL=0
local PLANE_MODE_CIRCLE=1
local PLANE_MODE_STABILIZE=2
local PLANE_MODE_TRAINING=3
local PLANE_MODE_ACRO=4
local PLANE_MODE_FLY_BY_WIRE_A=5
local PLANE_MODE_FLY_BY_WIRE_B=6
local PLANE_MODE_CRUISE=7
local PLANE_MODE_AUTOTUNE=8
local PLANE_MODE_AUTO=10
local PLANE_MODE_RTL=11
local PLANE_MODE_LOITER=12
local PLANE_MODE_TAKEOFF=13
local PLANE_MODE_AVOID_ADSB=14
local PLANE_MODE_GUIDED=15
local PLANE_MODE_INITIALIZING=16
local PLANE_MODE_QSTABILIZE=17
local PLANE_MODE_QHOVER=18
local PLANE_MODE_QLOITER=19
local PLANE_MODE_QLAND=20
local PLANE_MODE_QRTL=21
local PLANE_MODE_QAUTOTUNE=22
local PLANE_MODE_QACRO=23
local PLANE_MODE_THERMAL=24

local MAV_CMD_NAV_LAND=21
local MAV_CMD_NAV_TAKEOFF=22
local MAV_CMD_NAV_TAKEOFF_LOCAL=24
local MAV_CMD_NAV_VTOL_TAKEOFF=84
local MAV_CMD_NAV_VTOL_LAND=85

local lpf_coef_default = 0.002
local current_amps_very_filtered = { }
local time_remaining_s = { }
local SECONDS_MAX = 2*24*3600 -- 2 day
local SECONDS_MIN = 1

local FAST_MODE_DURATION_MS = 2*60*1000
local SLOW_MODE_DURATION_MS = 2*60*1000
local INIT_DELAY_DURATION_MS = 30*1000
local fast_mode_ms = 0
local slow_mode_ms = 0
local init_delay_ms = 0


-- constrain a value between limits
function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
 end
 
function init()
    gcs:send_text(MAV_SEVERITY.INFO, "LUA: START: Battery Time Remaning")
       
    for i=0, battery:num_instances() do
        current_amps_very_filtered[i] = 0.0
        time_remaining_s[i] = 0
    end
    return update, 1000 -- 1Hz
end


function update_battery_instance_1Hz(instance)
    if instance >= battery:num_instances() or battery:healthy(instance) == false then
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: bail[%d] %d %d', instance, battery:num_instances(), battery:healthy(instance)))
        return
    end

    local consumed_mah = battery:consumed_mah(instance)
    local current_amps = battery:current_amps(instance)
    local capacity = battery:pack_capacity_mah(instance)
    if not consumed_mah or not current_amps or capacity <= 10 then
        -- sanity check
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: sanity check'))
        return
    end


    if stop_filtering() then
        battery:set_time_remaining_external(0, instance)
        time_remaining_s[instance] = 0
        return
    elseif time_remaining_s[instance] == 0 then
        -- first time we are flying, init the LPF
        current_amps_very_filtered[instance] = current_amps
        -- fast_mode_ms = millis() -- start running in fast mode siliently
    end
    
    local coef = get_coef(instance)

    -- apply simple 1st order FIR filter to current_amps
    -- This coef is expected to be very very small (like 0.001), to
    -- create a time-constant very very long (like a few minutes)
    current_amps_very_filtered[instance] = (current_amps_very_filtered[instance] * (1.0-coef)) + (current_amps * coef)
    if current_amps_very_filtered[instance] == nil or current_amps_very_filtered[instance] == 0.0 then
        -- divide-by-zero check. Best to just not update it and keep old value
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: DBZ'))
        return
    end

    local mAh_remaining = constrain((capacity - consumed_mah), 0, capacity)
    local AmpSec_remaining = mAh_remaining * 3.6 -- == 3600 * 0.001 == AP_SEC_PER_HOUR * mAh_TO_Ah
    time_remaining_s[instance] = math.floor((AmpSec_remaining / current_amps_very_filtered[instance]) + 0.5)

    local time_remaining_s_constrained = constrain(time_remaining_s[instance], SECONDS_MIN, SECONDS_MAX)

    battery:set_time_remaining_external(time_remaining_s_constrained, instance)

    -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: batt[%d] = %d, %s', instance+1, time_remaining_s[instance], disp_time(time_remaining_s[instance])))
end

function disp_time(time)
    -- stolen from https://stackoverflow.com/questions/45364628/lua-4-script-to-convert-seconds-elapsed-to-days-hours-minutes-seconds
    local days = math.floor(time/86400)
    local hours = math.floor(math.fmod(time, 86400)/3600)
    local minutes = math.floor(math.fmod(time,3600)/60)
    local seconds = math.floor(math.fmod(time,60))
    -- result in D:HH:MM:SS format.
    return string.format("%d:%02d:%02d:%02d",days,hours,minutes,seconds)
end

function stop_filtering()
    
    local result = false

    if mission:state() == mission.MISSION_RUNNING then
        -- landing or taking off
        local mission_id = mission:get_current_nav_id()
        result = result or (mission_id == MAV_CMD_NAV_TAKEOFF)
        result = result or (mission_id == MAV_CMD_NAV_VTOL_TAKEOFF)
        result = result or (mission_id == MAV_CMD_NAV_TAKEOFF_LOCAL)

        result = result or (mission_id == MAV_CMD_NAV_LAND)
        result = result or (mission_id == MAV_CMD_NAV_VTOL_LAND)
    end
    
    -- other flight modes that we should not be in while flying but might be on the ground
    local flight_mode = vehicle:get_mode()
    result = result or (flight_mode == PLANE_MODE_INITIALIZING)
    result = result or (flight_mode == PLANE_MODE_TAKEOFF)
    result = result or (flight_mode == PLANE_MODE_MANUAL)
    -- result = result or (flight_mode == PLANE_MODE_FLY_BY_WIRE_A)
    -- result = result or (flight_mode == PLANE_MODE_FLY_BY_WIRE_B)

    -- not flying
    result = result or (not vehicle:get_likely_flying())

    local now_ms = millis()
    if result then
        init_delay_ms = now_ms
    elseif (init_delay_ms > 0) then
        if (now_ms - init_delay_ms < INIT_DELAY_DURATION_MS) then
            -- we're in startup delay, continue to inhibit filtering
            result = true
        else
            -- time has expired
            init_delay_ms = 0
        end
    end
    return result
end

function get_coef(instance)

    local now_ms = millis()
    local use_this_coef = lpf_coef_default


    -- if we suspect the estimate is way off so lets speed up the LPF for a bit
    local speed_things_up = false
    speed_things_up = speed_things_up or (time_remaining_s[instance] >= (SECONDS_MAX/4))
    speed_things_up = speed_things_up or (time_remaining_s[instance] == 0) -- we just initialized
    -- speed_things_up = speed_things_up or (other non-linear events)
    
    
    local slow_things_down = false
    -- slow_things_down = slow_things_down or (other non-linear events)


    if (speed_things_up or fast_mode_ms > 0) then
        if speed_things_up then
            if (fast_mode_ms == 0) then
                -- starting fast mode
                gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: BATT%d Estimate speeding up', instance+1))
            end
            fast_mode_ms = now_ms
        end

        if (now_ms - fast_mode_ms < FAST_MODE_DURATION_MS) then
            use_this_coef = use_this_coef * 10
        else
            -- ending fast mode
            fast_mode_ms = 0
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: BATT%d Estimate resuming normal speed', instance+1))
        end
    elseif (slow_things_down or slow_mode_ms > 0) then
        if slow_things_down then
            if (slow_mode_ms == 0) then
                -- starting slow mode
                gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: BATT%d Estimate slowing down', instance+1))
            end
            slow_mode_ms = now_ms
        end

        if (now_ms - slow_mode_ms < SLOW_MODE_DURATION_MS) then
            use_this_coef = use_this_coef / 10
        else
            -- ending slow mode
            slow_mode_ms = 0
            gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: BATT%d Estimate resuming normal speed', instance+1))
        end
    end
    return constrain(use_this_coef, 0.0001, 0.1)
end

function update()
    -- for instance = 0, battery:num_instances() do
    --     update_battery_instance_1Hz(instance)
    -- end

    update_battery_instance_1Hz(0)
    
    return update, 1000 -- 1Hz
end

return init, 2000


