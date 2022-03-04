-----------------------------------------------------------------------------
-- UDP sample: echo protocol server
-- LuaSocket sample files
-- Author: Diego Nehab
-----------------------------------------------------------------------------
-- local socket = require("socket")

-- local ip_0 = param:get('KHA_JSON_IP0')
-- local ip_1 = param:get('KHA_JSON_IP1')
-- local ip_2 = param:get('KHA_JSON_IP2')
-- local ip_3 = param:get('KHA_JSON_IP3')
-- local port = param:get('KHA_JSON_PORT')

--local host = string.format("%d.%d.%d.%d", ip_0, ip_1, ip_2, ip_3)
--local host = "127.0.0.1"
--local host = "255.255.255.255"

--local dest_ip = '172.20.13.94'
--local host = host or "172.20.13.94"
--local host = host or "255.255.255.255"
--local port = port or 13138
--local dest_port = 13138

-- local loop_count = 0;

--if arg then
--    host = arg[1] or host
--    port = arg[2] or port
--end
--host = host or "127.0.0.1"
--port = port or 8888
--if arg then
--    host = arg[1] or host
--    port = arg[2] or port
--end

--gcs:send_text(0, "Binding to host '" ..host.. "' and port " ..port.. " for JSON output...")
-- gcs:send_text(0, "Binding to host '" ..host.. "' and port " ..port.. "...")
--local udp = assert(socket.udp())
--assert(udp:setsockname(host, port))
--assert(udp:settimeout(5))
--ip, port = udp:getsockname()
--assert(ip, port)
--udp:settimeout(0)
--udp:setpeername("255.255.255.255", 13138)
--udp:setpeername("172.20.13.94", 13138)

--assert(udp:setsockname("*",0))
--assert(udp:setpeername("255.255.255.255",1234))
--assert(udp:setoption(“broadcast”, true))



-- local udp = socket.udp()
-- udp:settimeout(0)
-- udp:setoption('reuseaddr',true)
-- udp:setsockname('*', 1900)
-- udp:setpeername('172.20.13.94', 50000) -- Echo IP and Port number

function update() -- this is the loop which periodically runs

    --gcs:send_text(0, "script tick")

    --local str_data = string.format("loop count %d", loop_count);
    --local dgram = "This is A test '" .. str_data
    --local dgram = 'This is A test'

    --gcs:send_text(0, "Echoing '" .. dgram .. "' to " .. ip .. ":" .. port)

    --udp:sendto(dgram, dest_ip, dest_port)
    --udp:send("Testing")
    -- udp:close()


    -- local strData1 = 'This is a test'
    -- udp:send(strData1)

    gcs:send_text(0, "ping")

    return update, 100 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

