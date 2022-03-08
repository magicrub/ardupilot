-- UDP Socket send
local socket = require("socket")

local stream_id = 0

local ip
local ip_last
local port
local port_last
local name = kha:get_udp_out_name(stream_id)

local udp = assert(socket.udp())
assert(udp:settimeout(1))

function update_ip_port()
    ip = kha:get_udp_out_ip(stream_id)
    port = kha:get_udp_out_port(stream_id)
    assert(ip, port)
    --assert(udp:setoption("ip-add-membership", {multiaddr = ip, interface = "*"}))
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

    local data = kha:get_udp_out_data_str(stream_id)
    if data then
        update_ip_port()
        udp:sendto(data, ip, port)
    end
    local interval_ms = kha:get_udp_out_interval_ms(stream_id)
    return update, interval_ms -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

