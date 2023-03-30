local socket = require("socket")

local host = "*"
local port = 80

local batt_instance_system = 0

gcs:send_text(0, "Binding to host '" .. host .. "' and port " .. port .. "...")

local socket = assert(socket.bind(host, port))
local ip, port = socket:getsockname()
assert(ip, port)

gcs:send_text(0, "Waiting connection from talker on " .. ip .. ":" .. port .. "...")
gcs:send_text(0, "Connected. Here is the stuff:")

local function urldecode(str)
    str = string.gsub(str, '+', ' ')
    str = string.gsub(str, '%%(%x%x)', function(h)
        return string.char(tonumber(h, 16))
    end)
    str = string.gsub(str, '\r\n', '\n')
    return str
end

-- parse querystring
local function parse(str)
    local vars = {}
    for pair in string.gmatch(tostring(str), '[^&]+') do
        if not string.find(pair, '=') then
            vars[urldecode(pair)] = ''
        else
            local key, value = string.match(pair, '([^=]*)=(.*)')
            if key then
                key = urldecode(key)
                value = urldecode(value)
                local type = type(vars[key])
                if type == 'nil' then
                    vars[key] = value
                elseif type == 'table' then
                    table.insert(vars[key], value)
                else
                    vars[key] = {vars[key], value}
                end
            end
        end
    end
    return vars
end

local configuration = {
    system = {
        consoleForward = {
            enabled = false,
            ip = "127.0.0.1",
            port = 1234
        },
        maintenancePort = "none"
    }
}

local function update() -- this is the loop which periodically runs
    local connection = socket:accept()
    while true do
        local line, error = connection:receive()
        if error then
            break
        end

        -- check if we received GET line
        if line:find("GET") then
            local request_uri = line:match("GET (/.*) HTTP")
            if request_uri == "/" then
                request_uri = "/index.html"
            end

            local path, query = request_uri:match("(.*)(?.*)")
            gcs:send_text(0, "Request: " .. request_uri)
            if not path then
                path = request_uri
            end

            -- send response
            local file = io.open("/APM/scripts/fs" .. path, "r")
            if not file then
                gcs:send_text(0, "Opening file: " .. "@ROMFS/scripts/fs" .. path)
                file = io.open("@ROMFS/scripts/fs" .. path, "r")
            end
            -- send html file if exists
            if file and path:match(".html") then
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n")
                while true do
                    local data = file:read(1024)
                    if not data then
                        break
                    end
                    connection:send(data)
                end
            elseif file and path:match(".png") then
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: image/png\r\nConnection: close\r\n\r\n")
                local count = 0
                while true do
                    count = count + 1
                    local data = file:read(1024)
                    if not data then break end
                    connection:send(data)
                end
                gcs:send_text(6, "Sent " .. count .. "KB")
            elseif path == "/state" then
                local state = {
                    system = {
                        voltage = battery:voltage(batt_instance_system),
                        current = battery:current_amps(batt_instance_system),
                        time = millis():tofloat() * 0.001
                    }
                }
                if state.system.voltage == nil then
                    state.system.voltage = 25.23
                end
                if state.system.current == nil then
                    state.system.current = 0.1
                end
                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send('{ "system": { "voltage": ' .. state.system.voltage .. ', "current": ' ..
                                    state.system.current .. ', "time": ' .. state.system.time .. ' } }')

            elseif path == "/configuration" then
                local system = configuration.system
                if query then
                    local updates = parse(string.sub(query, 2))
                    if updates["system.consoleForward.enabled"] == "true" then
                        system.consoleForward.enabled = true
                    elseif updates["system.consoleForward.enabled"] == "false" then
                        system.consoleForward.enabled = false;
                    end
                    system.consoleForward.ip = updates["system.consoleForward.ip"] or system.consoleForward.ip
                    system.consoleForward.port = tostring(updates["system.consoleForward.port"]) or
                                                     system.consoleForward.port
                    system.maintenancePort = updates["system.maintenancePort"] or system.maintenancePort
                    -- TODO: Tom, configuration has been updated. Use it
                end

                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send('{ "system": { "consoleForward": { "enabled": ' ..
                                    (system.consoleForward.enabled and 'true' or 'false') .. ', "ip": "' ..
                                    system.consoleForward.ip .. '", "port": ' .. system.consoleForward.port ..
                                    ' }, "maintenancePort": "' .. system.maintenancePort ..
                                    '" } }')

            elseif path == "/calibrate" then
                -- TODO
                gcs:send_text(0, "Calibrate INS") 

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
