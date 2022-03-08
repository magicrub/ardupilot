-- UDP Socket send
local socket = require("socket")

local stream_id = 1

local ip
local port
local name = kha:get_udp_out_name(stream_id)

local udp = assert(socket.udp())
assert(udp:settimeout(1))

function update_ip_port()
    ip = kha:get_udp_out_ip(stream_id)
    port = kha:get_udp_out_port(stream_id)
    assert(ip, port)
end

function update() -- this is the loop which periodically runs
    local data = kha:get_udp_out_data_str(stream_id)
    if data then
        update_ip_port()
        udp:sendto(data, ip, port)
    end
    local interval_ms = kha:get_udp_out_interval_ms(stream_id)
    return update, interval_ms -- reschedules the loop
end

update_ip_port()
gcs:send_text(0, "UDP Stream " ..name.. " sending to " .. ip .. ":" .. port .. "  is running...")
return update() -- run immediately before starting to reschedule

