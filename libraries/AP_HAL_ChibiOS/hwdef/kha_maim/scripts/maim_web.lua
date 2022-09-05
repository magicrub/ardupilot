local socket = require("socket")

local host = "*"
local port = 80

local batt_instance_system = 0
local batt_instance_payload1 = 1
local batt_instance_payload2 = 2

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
    },
    payload1 = {
        enabled = mod_payload:get_enable(1),
        ip = "127.0.0.1", -- tostring(mod_payload:get_udp_out_ip(1)),
        netmask = "255.0.0.0",
        gateway = "1.1.1.1"
    },
    payload2 = {
        enabled = mod_payload:get_enable(2),
        ip = "127.0.0.1",
        netmask = "255.0.0.0",
        gateway = "2.2.2.2"
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
            if not path then
                path = request_uri
            end

            -- send response
            local file = io.open("./scripts/fs" .. path, "r")
            if not file then
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

            elseif path == "/state" then
                local state = {
                    system = {
                        voltage = mod_payload:get_voltage(batt_instance_system),
                        current = mod_payload:get_current(batt_instance_system)
                    },
                    payload1 = {
                        voltage = mod_payload:get_voltage(batt_instance_payload1),
                        current = mod_payload:get_current(batt_instance_payload1)
                    },
                    payload2 = {
                        voltage = mod_payload:get_voltage(batt_instance_payload2),
                        current = mod_payload:get_current(batt_instance_payload2)
                    }
                }

                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send('{ "system": { "voltage": ' .. state.system.voltage .. ', "current": ' ..
                                    state.system.current .. ' }, "payload1": { "voltage": ' .. state.payload1.voltage ..
                                    ', "current": ' .. state.payload1.current .. ' }, "payload2": { "voltage": ' ..
                                    state.payload2.voltage .. ', "current": ' .. state.payload2.current .. ' } }')

            elseif path == "/configuration" then
                local system = configuration.system
                local payload1 = configuration.payload1
                local payload2 = configuration.payload2

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
                    if updates["payload1.enabled"] == "true" then
                        payload1.enabled = true
                    elseif updates["payload1.enabled"] == "false" then
                        payload1.enabled = false;
                    end
                    payload1.ip = updates["payload1.ip"] or payload1.ip
                    payload1.netmask = updates["payload1.netmask"] or payload1.netmask
                    payload1.gateway = updates["payload1.gateway"] or payload1.gateway
                    if updates["payload2.enabled"] == "true" then
                        payload2.enabled = true
                    elseif updates["payload2.enabled"] == "false" then
                        payload2.enabled = false;
                    end
                    payload2.ip = updates["payload2.ip"] or payload2.ip
                    payload2.netmask = updates["payload2.netmask"] or payload2.netmask
                    payload2.gateway = updates["payload2.gateway"] or payload2.gateway

                    mod_payload:set_enable(1, payload1.enabled)
                    mod_payload:set_enable(2, payload2.enabled)

                    -- TODO: Tom, configuration has been updated. Use it
                end

                connection:send("HTTP/1.0 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n")
                connection:send('{ "system": { "consoleForward": { "enabled": ' ..
                                    (system.consoleForward.enabled and 'true' or 'false') .. ', "ip": "' ..
                                    system.consoleForward.ip .. '", "port": ' .. system.consoleForward.port ..
                                    ' }, "maintenancePort": "' .. system.maintenancePort ..
                                    '" }, "payload1": { "enabled": ' .. (payload1.enabled and 'true' or 'false') ..
                                    ', "ip": "' .. payload1.ip .. '", "netmask": "' .. payload1.netmask ..
                                    '", "gateway": "' .. payload1.gateway .. '" }, "payload2": { "enabled": ' ..
                                    (payload2.enabled and 'true' or 'false') .. ', "ip": "' .. payload2.ip ..
                                    '", "netmask": "' .. payload2.netmask .. '", "gateway": "' .. payload2.gateway ..
                                    '" } }')

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
