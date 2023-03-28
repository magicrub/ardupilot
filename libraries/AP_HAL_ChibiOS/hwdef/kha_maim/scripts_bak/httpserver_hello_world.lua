-----------------------------------------------------------------------------
-- HTTPSERVER sample
-----------------------------------------------------------------------------
local socket = require("socket")
host = "*"
port = 80
gcs:send_text(0, "Binding to host '" ..host.. "' and port " ..port.. "...")
s = assert(socket.bind(host, port))
i, p = s:getsockname()
assert(i, p)
gcs:send_text(0, "Waiting connection from talker on " .. i .. ":" .. p .. "...")
gcs:send_text(0, "Connected. Here is the stuff:")
function update() -- this is the loop which periodically runs
    c = s:accept()
    l, e = c:receive()
    while not e do
        gcs:send_text(0, l)
        -- check if we received GET line
        if l:find("GET") then
            -- send response
            c:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
            c:send("<html><body>Hello World!</body></html>\r\n")
            break
        end
        l, e = c:receive()
    end
    c:close()
    return update, 1 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule

-- httpserver:init(80)
-- function update() -- this is the loop which periodically runs
-- 	request = httpserver:recv()
-- 	if request then
-- 		gcs:send_text(0, "Received " .. request)
--         -- send http ok response
--         httpserver:send("HTTP/1.0 200 OK\r\n\r\n")
--         httpserver:response_finish()
--     else
--         gcs:send_text(0, "No request")
--     end
--     return update, 1000 -- reschedules the loop
-- end

-- return update() -- run immediately before starting to reschedule
