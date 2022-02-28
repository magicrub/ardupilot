-----------------------------------------------------------------------------
-- HTTPSERVER sample
-----------------------------------------------------------------------------
local count = 0

gcs:send_text(0, "Lua Start")

function update() -- this is the loop which periodically runs
    count = count + 1
    gcs:send_text(0, "Lua Running" .. count)
    return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
