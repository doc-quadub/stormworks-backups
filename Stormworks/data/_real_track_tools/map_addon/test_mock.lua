-- offline harness: fake Stormworks API, runs the addon, dumps the map lines
local out = io.open(arg[2], "w")
local calls = { add = 0, remove = 0, gettile = 0, ids = 0 }
matrix = {}
function matrix.translation(x, y, z)
    return { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1 }
end
local MOD = {}      -- fake placements of the reused tiles: centre -> {name, rot}
server = {}
function server.announce(t, m, p) print("[announce]", m) end
function server.getMapID() calls.ids = calls.ids + 1; return calls.ids end
function server.addMapLine(peer, id, a, b, w, r, g, bb, al)
    calls.add = calls.add + 1
    out:write(string.format("%.1f %.1f %.1f %.1f\n", a[13], a[15], b[13], b[15]))
end
function server.removeMapLine(peer, id) calls.remove = calls.remove + 1 end
function server.getTile(m)
    calls.gettile = calls.gettile + 1
    local x, z = m[13], m[15]
    if z > -13000 and z < 2000 and x > -9000 and x < 14000 then
        return { name = string.format("data/tiles/mega_island_%d_%d.xml", (x + 8000) // 1000, (z + 12000) // 1000) }, true
    end
    if z > -41000 and z < -24000 and x > -25000 and x < 7000 then
        return { name = string.format("data/tiles/arid_island_%d_%d.xml", (x + 24000) // 1000, (z + 40000) // 1000) }, true
    end
    local k = x .. "," .. z
    if MOD[k] then return { name = "data/tiles/" .. MOD[k][1] .. ".xml" }, true end
    return { name = "" }, true
end
function server.getTileTransform(m, name, r)
    local x, z = m[13], m[15]
    local k = x .. "," .. z
    local t = matrix.translation(x, 0, z)
    if MOD[k] and MOD[k][2] then     -- rotated 90 deg
        t[1], t[3], t[9], t[11] = 0, -1, 1, 0
    end
    return t, true
end
dofile(arg[1])
-- fake sea line: put a few reused tiles in a row south of Sawyer
local i = 0
for name in pairs(TILES_FOR_TEST or {}) do i = i + 1 end
local names = { "track_straight", "track_straight", "track_curve_left_diagonal", "track_bridge_straight" }
for j, n in ipairs(names) do MOD["0," .. (-14000 - 1000 * j)] = { n, j == 2 } end
onCreate(true)
for t = 1, 400 do onTick(1) end
print("after load: getTile", calls.gettile, "addMapLine", calls.add, "ids", calls.ids)
onCustomCommand("?railmap info", 0, true, true, "?railmap", "info")
onCustomCommand("?railmap ticks 500 60", 0, true, true, "?railmap", "ticks", "500", "60")
for t = 1, 100 do onTick(1) end
print("after ticks cmd: addMapLine", calls.add, "remove", calls.remove, "ids", calls.ids)
onCustomCommand("?railmap color 255 0 0 200", 0, true, true, "?railmap", "color", "255", "0", "0")
onCustomCommand("?railmap", 0, true, true, "?railmap")
onCustomCommand("?railmap off", 0, false, true, "?railmap", "off")
-- reload: saved data present, no scan
local saved = g_savedata
dofile(arg[1]); g_savedata = saved
local before = calls.gettile
onCreate(false)
for t = 1, 100 do onTick(1) end
print("reload: extra getTile", calls.gettile - before)
out:close()
