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
        -- gcs:send_text(0, l)
        -- check if we received GET line
        if l:find("GET") then
            request_uri = l:match("GET /(.*) HTTP")
            if not request_uri or request_uri == "" then
                request_uri = "index.html"
            end
            -- gcs:send_text(0, "file_name: " .. request_uri)
            -- send response
            resp_file = io.open("@ROMFS/scripts/fs/"..request_uri, "r")
            -- send html file if exists
            if resp_file and request_uri:match(".html") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end
            elseif resp_file and request_uri:match(".png") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: image/png\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end
            elseif resp_file and request_uri:match(".js") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/javascript\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end
            elseif resp_file and request_uri:match(".css") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: text/css\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end
            elseif resp_file and request_uri:match(".ajax") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end
            -- handle get system time ajax commands
            elseif request_uri == "read/time" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send("{\"time\":\""..tostring(millis()).."\"}")
            elseif request_uri == "write/0" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send("{\"result\":\"ok\"}")
                gpio:write(60, 1)
                gpio:write(20, 1)
                gpio:write(3, 1)
                gpio:write(4, 1)
            elseif request_uri == "write/1" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send("{\"result\":\"ok\"}")
                gpio:write(60, 0)
                gpio:write(20, 0)
                gpio:write(3, 0)
                gpio:write(4, 0)
            else
                -- send 404 if file not found
                c:send("HTTP/1.0 404 Not Found\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                c:send("<html><body><h1>404 Not Found</h1></body></html>")
            end
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
