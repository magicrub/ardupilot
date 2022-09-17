-----------------------------------------------------------------------------
-- UDP sample: echo protocol server
-- LuaSocket sample files
-- Author: Diego Nehab
-----------------------------------------------------------------------------
local socket = require("socket")
host = host or "192.168.1.50"
port = port or 8888
if arg then
    host = arg[1] or host
    port = arg[2] or port
end

gcs:send_text(0, "Binding to host '" ..host.. "' and port " ..port.. "...")
udp = assert(socket.udp())
assert(udp:setsockname(host, port))
assert(udp:settimeout(5))
ip, port = udp:getsockname()
assert(ip, port)
gcs:send_text(0, "Waiting packets on " .. ip .. ":" .. port .. "...")
function update() -- this is the loop which periodically runs
	dgram, ip, port = udp:receivefrom()
	if dgram then
		gcs:send_text(0, "Echoing '" .. dgram .. "' to " .. ip .. ":" .. port)
		udp:sendto(dgram, ip, port)
	else
        gcs:send_text(0, ip)
    end
    return update, 1 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

