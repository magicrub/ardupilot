
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local is_armed_last = false;
local param_AIRCRAFT_TOW = -1.0
local param_AIRCRAFT_TOW_min = 10.0     -- kg
local param_AIRCRAFT_TOW_max = 25.0     -- kg
local param_AIRCRAFT_SPAN_min = 5.0     -- m    (unused)
local param_AIRCRAFT_SPAN_max = 6.0     -- m    (unused)

-- param reference values for 13.8kg aircraft
local ref_AIRCRAFT_TOW = 13.8           -- kg   - takeoff weight
local ref_TKOFF_ROTATE_SPD = 12.5       -- m/s  - takeoff rotation speed
local ref_ARSPD_FBW_MIN = 14.0          -- m/s  - minimum airspeed
local ref_TRIM_ARSPD_CM = 1500          -- cm/s - cruise airspeed

function update()

    local do_param_adjustment = false
    param_AIRCRAFT_TOW = param:get('AIRCRAFT_TOW')

    if (not param_AIRCRAFT_TOW) then
        gcs:send_text(MAV_SEVERITY.ERROR, "LUA: param AIRCRAFT_TOW not found, check FW")
    elseif (param_AIRCRAFT_TOW ~= 0) and ((param_AIRCRAFT_TOW < param_AIRCRAFT_TOW_min) or (param_AIRCRAFT_TOW > param_AIRCRAFT_TOW_max)) then
        gcs:send_text(MAV_SEVERITY.ERROR, string.format('LUA: param AIRCRAFT_TOW = %.2f out of range. Valid: %.2f to %.2f', param_AIRCRAFT_TOW, param_AIRCRAFT_TOW_min, param_AIRCRAFT_TOW_max))
    else
        local is_armed = arming:is_armed()
        if (is_armed ~= is_armed_last) then
            is_armed_last = is_armed
            if (is_armed) then
                -- we just armed
                do_param_adjustment = true
            end
        end
    end

    if (do_param_adjustment == true) then
        adjust_params()
    end

    if (vehicle:get_time_flying_ms() < 30000) then
        -- repeat the checks until we've been airborne for 1 minte
        return update, 1000 -- 1Hz
    else
        gcs:send_text(MAV_SEVERITY.INFO, "LUA: Mass Param Adjuster Stopping")
    end
end

function adjust_params()

    if param_AIRCRAFT_TOW <= 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "LUA: Skipped weight adjustments, no params changed")
        return
    end

    local scaler = math.sqrt(param_AIRCRAFT_TOW / ref_AIRCRAFT_TOW)

    local param_TKOFF_ROTATE_SPD    = scaler * ref_TKOFF_ROTATE_SPD
    local param_ARSPD_FBW_MIN       = scaler * ref_ARSPD_FBW_MIN
    local param_TRIM_ARSPD_CM       = scaler * ref_TRIM_ARSPD_CM
    
    set_param('TKOFF_ROTATE_SPD',   param_TKOFF_ROTATE_SPD)
    set_param('ARSPD_FBW_MIN',      param_ARSPD_FBW_MIN)
    set_param('TRIM_ARSPD_CM',      param_TRIM_ARSPD_CM)
end


-- function for setting and saving parameter
function set_param(param_name, value)
    if param:set(param_name, value) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format('LUA: set param %s = %.2f', param_name, value))
    else
        gcs:send_text(MAV_SEVERITY.ERROR, string.format('LUA: unable to set param %s', param_name))
    end
end

function init()
    gcs:send_text(MAV_SEVERITY.INFO, "LUA: START: Mass Param Adjuster")
    return update, 1000 -- 1Hz
end


return init, 2000


