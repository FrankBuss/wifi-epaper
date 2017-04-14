collectgarbage("setmemlimit", 64)

-- initialize all display pins
pins = { 0, 1, 2, 5, 6, 7, 8 }
for k,pin in pairs(pins) do
	gpio.mode(pin, gpio.OUTPUT)
	gpio.write(pin, gpio.LOW)
end

function executeString(s)
    local fun = loadstring(s)
    fun()
end

function stripHeader(s)
    local pos = string.find(s, "\r\n\r\n")
    return s:sub(pos + 4)
end

function runScript(host, url)
    receivedPage = ""
    conn = net.createConnection(net.TCP, 0) 
    conn:on("receive", function(conn, data) receivedPage = receivedPage .. data end)
    conn:on("disconnection", function(conn, data) pcall(function() executeString(stripHeader(receivedPage)) end) end)
    conn:connect(80, host)
    conn:send("GET /" .. url .. " HTTP/1.1\r\nHost: " .. host .. "\r\nConnection: close\r\n\r\n")
end

wifi.setphymode(wifi.PHYMODE_N)
wifi.setmode(wifi.STATION)
wifi.sta.config("SSID","PASSWORD")
wifi.sta.eventMonReg(wifi.STA_IDLE, function() print("IDLE") end)
wifi.sta.eventMonReg(wifi.STA_CONNECTING, function() print("CONNECTING...") end)
wifi.sta.eventMonReg(wifi.STA_WRONGPWD, function() print("WRONG PASSWORD!!!") end)
wifi.sta.eventMonReg(wifi.STA_APNOTFOUND, function() print("NO SUCH SSID FOUND") end)
wifi.sta.eventMonReg(wifi.STA_FAIL, function() print("FAILED TO CONNECT") end)
wifi.sta.eventMonReg(wifi.STA_GOTIP, function() runScript("192.168.11.27","init2.lua") end)
wifi.sta.eventMonStart()
