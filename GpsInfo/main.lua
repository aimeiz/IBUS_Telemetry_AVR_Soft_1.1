--[[
GpsInfo widget for EdgeTX (text-only, half screen)
Works with sensors: Sats, Dist, Hdg, GSpd, GPS, GAlt, ASpd (optional)
Safely handles GPS as a table (lat/lon) or nil.
]]

local LABEL_X = 5
local VALUE_X = 100
local LINE_H  = 15
local TITLE_Y = 5
local START_Y = 30

-- Safe numeric read: returns number (or 0 if table/nil)
local function num(sensorName)
  local v = getValue(sensorName)
  return (type(v) == "number") and v or 0
end

-- Safe GPS read: returns table {lat=..., lon=..., fix=...} or nil
local function readGpsTable()
  local v = getValue("GPS")
  if type(v) == "table" then
    -- Common field names used in EdgeTX/OpenTX
    local lat = v.lat or v.latitude
    local lon = v.lon or v.longitude
    local fix = v.fix or v.gpsFix or nil
    return { lat = lat, lon = lon, fix = fix }
  end
  return nil
end

-- Fix text from fix code or sats fallback
local function fixTextFrom(gpsFix, sats)
  if type(gpsFix) == "number" then
    if gpsFix == 0 then return "NO FIX"
    elseif gpsFix == 2 then return "2D FIX"
    elseif gpsFix == 3 then return "3D FIX"
    else return tostring(gpsFix) end
  end
  -- Fallback by satellites count when fix code is unavailable
  if sats >= 6 then return "3D FIX"
  elseif sats >= 4 then return "2D FIX"
  else return "NO FIX" end
end

local function create(zone, options)
  return { zone = zone, options = options }
end

local function update(widget, options)
  widget.options = options
end

local function background(widget) end

local function drawLabelValue(y, label, value)
  lcd.drawText(LABEL_X, y, label)
  lcd.drawText(VALUE_X, y, value)
end

local function refresh(widget)
  lcd.clear()

  -- Read core sensors
  local sats    = num("Sats")
  local alt     = num("GAlt")
  local gspd    = num("GSpd")
  local vspeed  = num("VSpd")   -- may be 0 / missing
  local heading = num("Hdg")
  local dist    = num("Dist")
  local aspd    = num("ASpd")   -- optional (pitot); 0 if absent

  -- Read GPS table (lat/lon + optional fix)
  local gps = readGpsTable()
  local fixTxt = fixTextFrom(gps and gps.fix, sats)

  -- Title
  lcd.drawText(LABEL_X, TITLE_Y, "GPS Telemetry", INVERS)

  local y = START_Y
  drawLabelValue(y,   "Fix:",          fixTxt);                    y = y + LINE_H
  drawLabelValue(y,   "Satellites:",   string.format("%d", sats)); y = y + LINE_H

  -- Coordinates if available
  if gps and gps.lat and gps.lon then
    drawLabelValue(y, "Latitude:",     string.format("%.6f", gps.lat)); y = y + LINE_H
    drawLabelValue(y, "Longitude:",    string.format("%.6f", gps.lon)); y = y + LINE_H
  end

  drawLabelValue(y,   "Altitude:",     string.format("%.1f m",   alt));     y = y + LINE_H
  drawLabelValue(y,   "Speed:",        string.format("%.1f km/h", gspd));   y = y + LINE_H
  drawLabelValue(y,   "VSpeed:",       string.format("%.1f m/s", vspeed));  y = y + LINE_H
  drawLabelValue(y,   "Heading:",      string.format("%.0f°",    heading)); y = y + LINE_H
  drawLabelValue(y,   "Distance:",     string.format("%.0f m",   dist));    y = y + LINE_H

  if aspd > 0 then
    drawLabelValue(y, "AirSpeed:",     string.format("%.1f km/h", aspd));   y = y + LINE_H
  end
end

return {
  name = "GpsInfo",
  options = {},
  create = create,
  update = update,
  background = background,
  refresh = refresh
}
