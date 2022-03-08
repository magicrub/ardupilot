-- UDP Socket send
local socket = require("socket")

local json_ip = kha:get_json_ip()
local json_port = kha:get_json_port()

gcs:send_text(0, "AHRS JSON binding to ip '" ..json_ip.. "' and port " ..json_port.. "...")

json_udp = assert(socket.udp())
assert(json_udp:setsockname(json_ip, json_port))
assert(json_udp:settimeout(5))
json_ip, json_port = json_udp:getsockname()
assert(json_ip, json_port)

gcs:send_text(0, "Waiting packets to " .. json_ip .. ":" .. json_port .. "...")

function update() -- this is the loop which periodically runs
    local json_dgram = kha:get_json_str()
    if json_dgram then
        json_udp:sendto(json_dgram, json_ip, json_port)
    end
    return update, 10 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

