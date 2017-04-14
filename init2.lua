-- increase default limit of 4 kB to 64 kB for strings
collectgarbage("setmemlimit", 64)

-- callback function to show an image
function showImage(image)
    epd.image(image)
end

-- web load functions

function trim(s)
  return s:match "^%s*(.-)%s*$"
end

function stripHeader(s)
    local pos = string.find(s, "\r\n\r\n")
    return s:sub(pos + 4)
end

function getImage(host, filename)
    receivedPage = ""
    conn = net.createConnection(net.TCP, 0) 
    conn:on("receive", function(conn, data) receivedPage = receivedPage .. data end)
    conn:on("disconnection", function(conn, data) showImage(trim(stripHeader(receivedPage))) end)
    conn:connect(80, host)
    conn:send("GET /" .. filename .. " HTTP/1.1\r\nHost: " .. host .. "\r\nConnection: close\r\n\r\n")
end

-- local host, connected over WiFi
host = "192.168.11.27"

-- clear display
getImage(host, "white.bin")

-- init mqtt client without logins, keepalive timer 120s
m = mqtt.Client("clientid", 120)

-- on publish message receive event
m:on("message", function(client, topic, data) 
  if data ~= nil then
    print(topic .. ": " .. data) 
    getImage(host, data)
  end
end)

m:connect(host, 1883, 0, function(client)
	print("connected")
	-- subscribe "display" with qos = 0
	m:subscribe("display",0, function(client) print("subscribe success") end)
end, function(client, reason) print("failed reason: "..reason) end)

print("init2.lua loaded")
