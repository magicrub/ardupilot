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
state = {
    system = {
        ip = "127.0.0.1",
        netmask = "255.0.0.0",
        gateway = "1.1.1.1",
        maintenancePort = "none"
    },
    payload1 = {
        enabled = true,
        consoleForward = {
            ip = "127.0.0.1",
            port = 1234
        }
    },
    payload2 = {
        enabled = true,
        consoleForward = {
            ip = "127.0.0.1",
            port = 1234
        }
    }
}

function update() -- this is the loop which periodically runs
    c = s:accept()
    l, e = c:receive()
    while not e do
        gcs:send_text(0, l)
        -- check if we received GET line
        if l:find("GET") then
            request_uri = l:match("GET /(.*) HTTP")
            if not request_uri or request_uri == "" then
                request_uri = "index.html"
            end
            -- send response
            resp_file = io.open("./fs/"..request_uri, "r")

            -- send html file if exists
            if resp_file and request_uri:match(".html") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end

            elseif request_uri == "state" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send([[{
                    "system": {
                        "voltage": 19,
                        "current": 1.2,
                        "ip": "]] state.system.ip [[",
                        "netmask": "]] state.system.netmask [[",
                        "gateway": "]] state.system.gateway [[",
                        "maintenancePort": "]] state.system.maintenancePort [["
                    },
                    "payload1": {
                        "enabled": ]] state.payload1.enabled [[,
                        "voltage": 12,
                        "current": 1.0,
                        "consoleForward": {
                            "ip": "]] state.payload1.consoleForward.ip [[",
                            "port": ]] state.payload1.consoleForward.port [[
                        }
                    },
                    "payload2": {
                        "enabled": ]] state.payload2.enabled [[,
                        "voltage": 12,
                        "current": 1.0,
                        "consoleForward": {
                            "ip": "]] state.payload2.consoleForward.ip [[",
                            "port": ]] state.payload2.consoleForward.port [[
                        }
                    }
                }]])

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

