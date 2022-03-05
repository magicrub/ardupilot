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

configuration = {
    system = {
        consoleForward = {
            ip = "127.0.0.1",
            port = 1234
        },
        maintenancePort = "none"
    },
    payload1 = {
        enabled = true,
        ip = "127.0.0.1",
        netmask = "255.0.0.0",
        gateway = "1.1.1.1",
    },
    payload2 = {
        enabled = true,
        ip = "127.0.0.1",
        netmask = "255.0.0.0",
        gateway = "1.1.1.1",
    }
}

function update() -- this is the loop which periodically runs
    c = s:accept()
    while true do
        l, e = c:receive()
        if e then break end

        -- check if we received GET line
        if l:find("GET") then
            request_uri = l:match("GET (/.*) HTTP")
            if request_uri == "/" then
                request_uri = "/index.html"
            end

        
            path, query = request_uri:match("(.*)(?.*)")
            if not path then path = request_uri end

            -- send response
            resp_file = io.open("./scripts/fs"..path, "r")

            -- send html file if exists
            if resp_file and path:match(".html") then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                while true do
                    l = resp_file:read(1024)
                    if not l then break end
                    c:send(l)
                end

            elseif path == "/state" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send([[{
                    "system": {
                        "voltage": ]] .. math.random() .. [[,
                        "current": ]] .. math.random() .. [[
                    },
                    "payload1": {
                        "voltage": ]] .. math.random() .. [[,
                        "current": ]] .. math.random() .. [[
                    },
                    "payload2": {
                        "voltage": ]] .. math.random() .. [[,
                        "current": ]] .. math.random() .. [[
                    }
                }]])

            elseif path == "/configuration" then
                c:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                c:send([[{
                    "system": {
                        "consoleForward": {
                            "ip": "]] .. configuration.system.consoleForward.ip .. [[",
                            "port": ]] .. configuration.system.consoleForward.port .. [[
                        },
                        "maintenancePort": "]] .. configuration.system.maintenancePort .. [["
                    },
                    "payload1": {
                        "enabled": ]] .. tostring(configuration.payload1.enabled) .. [[,
                        "ip": "]] .. configuration.payload1.ip .. [[",
                        "netmask": "]] .. configuration.payload1.netmask .. [[",
                        "gateway": "]] .. configuration.payload1.gateway .. [["
                    },
                    "payload2": {
                        "enabled": ]] .. tostring(configuration.payload2.enabled) .. [[,
                        "ip": "]] .. configuration.payload2.ip .. [[",
                        "netmask": "]] .. configuration.payload2.netmask .. [[",
                        "gateway": "]] .. configuration.payload2.gateway .. [["
                    }
                }]])

            else 
                -- send 404 if file not found
                c:send("HTTP/1.0 404 Not Found\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                c:send("<html><body><h1>404 Not Found</h1></body></html>")

            end

            c:close()
        end
    end

    return update, 1 -- reschedules the loop
end
