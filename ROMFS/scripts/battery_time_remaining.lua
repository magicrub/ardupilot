
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local lpf_coef = 0.001
local one_minus_lpf_coef = 1.0-lpf_coef
local current_amps_very_filtered = { }

function init()
    gcs:send_text(MAV_SEVERITY.INFO, "LUA: START: Battery Time Remaning")
       
    for i=0, battery:num_instances() do
        current_amps_very_filtered[i] = 0.0
    end
end

function update()

    -- for instance = 0, battery:num_instances() do
    --     update_battery_1Hz(instance)
    -- end

    update_battery_1Hz(0)
    
    return update, 1000 -- 1Hz
end

function update_battery_1Hz(instance)
    if instance >= battery:num_instances() or battery:healthy(instance) == false then
        return
    end

    local consumed_mah = battery:consumed_mah(instance)
    local current_amps = battery:current_amps(instance)
    local capacity = battery:pack_capacity_mah(instance)
    if not consumed_mah or not current_amps or capacity <= 10 then
        -- sanity check
        return
    end

    -- apply simple 1st order FIR filter to current_amps
    -- This coef is expected to be very very small (like 0.001), to
    -- create a time-constant very very long (like a few minutes)
    current_amps_very_filtered[instance] = (current_amps_very_filtered[instance] * one_minus_lpf_coef) + (current_amps * lpf_coef)
    if current_amps_very_filtered[instance] == 0.0 then
        -- divide-by-zero check
        return
    end

    local mAh_remaining = math.max(0, capacity - consumed_mah)
    local AmpSec_remaining = mAh_remaining * 3.6 -- == 3600 * 0.001 == AP_SEC_PER_HOUR * mAh_TO_Ah
    local time_remaining = AmpSec_remaining / current_amps_very_filtered[instance]
    battery:set_time_remaining_external(instance, time_remaining)
end

return init(), 3000


