
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local lpf_coef = 0.001
local current_amps_very_filtered = { }
local init_needed = true

function init()
    gcs:send_text(MAV_SEVERITY.INFO, "LUA: START: Battery Time Remaning")
       
    for i=0, battery:num_instances() do
        current_amps_very_filtered[i] = 0.0
    end
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

    local coef = lpf_coef
    if not vehicle:get_likely_flying() then
        -- when not flying, speed up the filter to get to the new current_amps value faster
        coef = lpf_coef * 10
    end

    -- apply simple 1st order FIR filter to current_amps
    -- This coef is expected to be very very small (like 0.001), to
    -- create a time-constant very very long (like a few minutes)
    current_amps_very_filtered[instance] = (current_amps_very_filtered[instance] * (1.0-coef)) + (current_amps * coef)
    if current_amps_very_filtered[instance] == nil or current_amps_very_filtered[instance] == 0.0 then
        -- divide-by-zero check
        -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: DBZ'))
        return
    end

    local mAh_remaining = math.max(0, capacity - consumed_mah)
    local AmpSec_remaining = mAh_remaining * 3.6 -- == 3600 * 0.001 == AP_SEC_PER_HOUR * mAh_TO_Ah
    local time_remaining_s = AmpSec_remaining / current_amps_very_filtered[instance]

    time_remaining_s = math.max(1, time_remaining_s) -- don't go to zero, show 1s as lowest time so it stays valid

    battery:set_time_remaining_external(time_remaining_s, instance)
    local minutes = (time_remaining_s/60) % 60
    local hours = time_remaining_s/3600
    local days = hours / 24
    -- gcs:send_text(MAV_SEVERITY.DEBUG, string.format('LUA: time_remaining_s[%d] = %.0fd %.0fh %2.0fm', instance, days, hours, minutes))
end


function update()
    if init_needed then
        init_needed = false
        init()
    end

    -- for instance = 0, battery:num_instances() do
    --     update_battery_instance_1Hz(instance)
    -- end

    update_battery_instance_1Hz(0)
    
    return update, 1000 -- 1Hz
end

return update, 2000


