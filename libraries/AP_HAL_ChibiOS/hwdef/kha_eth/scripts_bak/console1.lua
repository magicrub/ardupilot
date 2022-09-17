-- UDP Socket send
local socket = require("socket")

local stream_id = 1
local ip
local ip_last
local port
local port_last
local name = "TestConsole"

local udp = assert(socket.udp())
assert(udp:settimeout(1))

function update_ip_port()
    ip = "0.0.0.0"
    port = 13137
    assert(ip, port)
    if (not ip_last) or (not port_last) or (ip ~= ip_last) or (port ~= port_last) then
        ip_last = ip
        port_last = port
        gcs:send_text(0, "UDP Stream " ..name.. " sending to " .. ip .. ":" .. port .. "")
        periph:can_printf("UDP Stream " ..name.. " sending to " .. ip .. ":" .. port .. "")
    end
end

local boot_delay = 1
function update() -- this is the loop which periodically runs
    if boot_delay > 0 then
        boot_delay = 0
        return update, 1000
    end

    local data = mod_payload:get_udp_out_data_str(stream_id)
    if data then
        update_ip_port()
        udp:sendto(data, ip, port)
    end
    local interval_ms = 10
    return update, interval_ms -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

