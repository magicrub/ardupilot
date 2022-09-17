-- UDP Socket send
local socket = require("socket")

local stream_id = 1
local ip
local ip_last
local port
local port_last
local name = "Serial2UDP_" ..stream_id.. ""

local udp = assert(socket.udp())
assert(udp:settimeout(1))

function update_ip_port()
    -- ip = networking:serial2udp_get_ip(stream_id)
    -- port = networking:serial2udp_get_port(stream_id)
    ip = "0.0.0.0"
    -- ip = "172.20.13.13"
    -- ip = "172.20.13.14"
    port = 1313
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
    update_ip_port()

    if boot_delay > 0 then
        boot_delay = 0
        return update, 100
    end

    local data
    local data_len = networking:serial2udp_get_udp_outbound_buffer(stream_id, data, 999)
    if data_len > 0 and data then
        udp:sendto(data, ip, port)
    end
    local interval_ms = 10
    return update, interval_ms -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

