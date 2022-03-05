local socket = require "socket"
local lunajson = require "lunajson"
local math = require "math"

local host = "*"
local port = 80

gcs:send_text(0, "Binding to host '" .. host .. "' and port " .. port .. "...")

local socket = assert(socket.bind(host, port))
local ip, port = s:getsockname()
assert(ip, port)

gcs:send_text(0, "Waiting connection from talker on " .. ip .. ":" .. port .. "...")
gcs:send_text(0, "Connected. Here is the stuff:")

local configuration = {
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

local function update() -- this is the loop which periodically runs
    local connection = socket:accept()
    while true do
        local line, error = connection:receive()
        if error then break end

        -- check if we received GET line
        if line:find("GET") then
            local request_uri = line:match("GET (/.*) HTTP")
            if request_uri == "/" then
                request_uri = "/index.html"
            end
        
            local path, query = request_uri:match("(.*)(?.*)")
            if not path then path = request_uri end

            -- send response
            local file = io.open("./scripts/fs"..path, "r")
            if not file then file = io.open("@ROMFS/scripts/fs"..path, "r") end

            -- send html file if exists
            if file and path:match(".html") then
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                while true do
                    local data = file:read(1024)
                    if not data then break end
                    connection:send(data)
                end

            elseif path == "/state" then
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send(lunajson.encode({
                    system = {
                        voltage = math.random(),
                        current = math.random()
                    },
                    payload1 = {
                        voltage = math.random(),
                        current = math.random()
                    },
                    payload2 = {
                        voltage = math.random(),
                        current = math.random()
                    }}))

            elseif path == "/configuration" then
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send(lunajson.encode(configuration))

            else 
                -- send 404 if file not found
                connection:send("HTTP/1.0 404 Not Found\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                connection:send("<html><body><h1>404 Not Found</h1></body></html>")

            end

            connection:close()
        end
    end

    return update, 1 -- reschedules the loop
end

return update()