

local MAV_SEVERITY_NOTICE=5    -- An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
local MAV_SEVERITY_INFO=6      -- Normal operational messages. Useful for logging. No action is required for these messages.

local ROTATION_PITCH_270 = 25

local MISSION_TAG_MEASURE_AGL = 100

local agl_samples_count = 0
local agl_samples_sum = 0

local run_once = true


function sample_rangefinder_to_get_AGL()
    if (not rangefinder:has_data_orient(ROTATION_PITCH_270)) then
        -- rangefinder not ready
        return
    end

    -- we're actively sampling rangefinder distance to ground
    local distance_raw_m = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01

    -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
    local ahrs_get_rotation_body_to_ned_c_z = math.cos(ahrs:get_roll())*math.cos(ahrs:get_pitch())
    local agl_corrected_for_attitude_m = distance_raw_m * ahrs_get_rotation_body_to_ned_c_z

    if (agl_samples_count <= 0) then
        agl_samples_count = 0 -- divide-by-zero sanity check in case it somehow wrapped or initialized wrong
        agl_samples_sum = 0
        gcs:send_text(MAV_SEVERITY_INFO, string.format("LUA: AGL measurements started"))
    end

    agl_samples_sum = agl_samples_sum + agl_corrected_for_attitude_m
    agl_samples_count = agl_samples_count + 1

    local agl_average = agl_samples_sum / agl_samples_count
    gcs:send_text(MAV_SEVERITY_INFO, string.format("LUA: AGL measurement %u: %.2fm, avg: %.2f", agl_samples_count, agl_corrected_for_attitude_m, agl_average))
end


function update_baro(new_agl_m)
    local ekf_agl_m = ahrs:get_hagl()
    if (not ekf_agl_m) then
        gcs:send_text(MAV_SEVERITY_NOTICE, string.format("LUA: unable to get current AGL so we can not correct baro"))
        return
    end

    local alt_error_m = ekf_agl_m - new_agl_m
    gcs:send_text(MAV_SEVERITY_INFO, string.format("LUA: AGL alt_error is: %.2f - %.2f = %.2f", ekf_agl_m, new_agl_m, alt_error_m))

    local baro_alt_offset = param:get('BARO_ALT_OFFSET')
    local baro_alt_offset_new_value = baro_alt_offset + alt_error_m
    gcs:send_text(MAV_SEVERITY_INFO, string.format("LUA: BARO_ALT_OFFSET changed from %.2f to %.2f", baro_alt_offset, baro_alt_offset_new_value))
    param:set('BARO_ALT_OFFSET', baro_alt_offset_new_value)
end


function update()
    if (run_once) then
        gcs:send_text(MAV_SEVERITY_INFO, "LUA: SCRIPT START: Check AGL Adjust Baro")
        run_once = false
    end

    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    local current_tag = mission:get_current_tag()
    if (current_tag) and  (current_tag == MISSION_TAG_MEASURE_AGL) then
        sample_rangefinder_to_get_AGL()
    elseif (agl_samples_count > 0) then
        -- we're not at the decision point but  we've recently been sampling so we must be done!
        local agl_average_final_m = agl_samples_sum / agl_samples_count
        -- finished sampling, use the result to offset baro
        gcs:send_text(MAV_SEVERITY_INFO, string.format("LUA: AGL measurements stopped: samples = %d, avg = %.2fm", agl_samples_count, agl_average_final_m))
        update_baro(agl_average_final_m)
        agl_samples_count = 0
        agl_samples_sum = 0
    end

    return update, 1000
end


return update(), 1000


