
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local lpf_coef = 0.001
local current_amps_very_filtered = { }
local time_remaining_s = { }
local SECONDS_MAX = 1*24*3600 -- 1 day
local SECONDS_MIN = 1

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

    if not vehicle:get_likely_flying() then
        -- when not flying, disable the LPF so it doesn't go crazy
        battery:set_time_remaining_external(0, instance)
        time_remaining_s[instance] = 0
        return
    elseif time_remaining_s[instance] == 0 then
        -- first time we are flying, init the LPF
        current_amps_very_filtered[instance] = current_amps
    end
    

    local coef = lpf_coef
    if time_remaining_s[instance] >= SECONDS_MAX then
        -- if the estimate is already very large, speed things up
        coef = lpf_coef * 10
    end

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
    -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: batt[%d] = %d, %s', instance, time_remaining_s[instance], disp_time(time_remaining_s[instance])))
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

function update()
    -- for instance = 0, battery:num_instances() do
    --     update_battery_instance_1Hz(instance)
    -- end

    update_battery_instance_1Hz(0)
    
    return update, 1000 -- 1Hz
end

return init, 2000


