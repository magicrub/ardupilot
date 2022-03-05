local socket = require("socket")

local ip_0 = param:get('KHA_JSON_IP0')
local ip_1 = param:get('KHA_JSON_IP1')
local ip_2 = param:get('KHA_JSON_IP2')
local ip_3 = param:get('KHA_JSON_IP3')
local dest_port = param:get('KHA_JSON_PORT')
local dest_ip = string.format("%d.%d.%d.%d", ip_0, ip_1, ip_2, ip_3)

local loop_count = 0

local udp = socket.udp()
udp:settimeout(0)
udp:setoption('reuseaddr',true)
udp:setsockname('*', 0)
udp:setpeername(dest_ip, dest_port) -- Echo IP and Port number

--gcs:send_text(0, "Starting JSON stream to " .. string.format("%s port %d", dest_ip, dest_port))

function update() -- this is the loop which periodically runs

    -- loop_count = loop_count + 1
    -- local str_data = string.format("loop count %d", loop_count);
    -- local dgram = "This is A test " .. str_data

    -- local str = kha:string()
    local str = kha:get_json_str()

    -- This works
    -- local str = FWVersion:string()


 --   local dgram = "JSON data " .. string.format("%s", str)
    local dgram = "JSON data " .. str
    


    if dest_port > 0 then
        udp:send(dgram)


    return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

