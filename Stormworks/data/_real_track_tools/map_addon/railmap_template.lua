-- Rail Map addon for Stormworks
-- Draws the railway network on the map screen (M): one line per route (double track and
-- station tracks are merged into one line) with short perpendicular ticks at equal intervals.
-- Track data is generated from the game's tile files by make_addon.py (decimetres).
--
-- Chat commands (admin): ?railmap            status and help
--                        ?railmap width W    line width
--                        ?railmap color R G B [A]
--                        ?railmap ticks SPACING LENGTH   (0 = no ticks)
--                        ?railmap on | off
--                        ?railmap rescan     find the sea line tiles again
--                        ?railmap info

local ADDON_NAME = "Rail Map"
local DATA_VERSION = "@@DATA_VERSION@@"

-- lines of the tiles with a known world position, world coordinates
local WORLD = @@WORLD@@

-- origin of tile <family>_island_X_Y is (1000*X - a, 1000*Y - b); used to check the positions
-- the game reports and to correct the lines above if they ever differ
local FAMILY_ORIGIN = { mega = { 8000, 12000 }, arid = { 24000, 40000 } }

-- lines of every other tile with tracks, tile local coordinates; placed where the scan finds them
local TILES = @@TILES@@

local DEFAULTS = {
    on = true,
    width = 1.0,
    r = 200, g = 25, b = 25, a = 255,
    tick_spacing = 300,     -- m along the line
    tick_length = 40,       -- m, whole tick
    scan_min = -80000,      -- scanned square, m (tile centres every 1000 m)
    scan_max = 80000,
}

local JOIN_DISTANCE = 15.0  -- line ends closer than this are one line
local SCAN_PER_TICK = 400
local DRAW_PER_TICK = 300

g_savedata = {}

local state = {
    scan = nil,         -- running tile scan
    lines = {},         -- {x1, z1, x2, z2} to draw
    ids = {},           -- map ui id per line
    jobs = {},          -- queued draw / remove jobs
    chains = 0,
    polylines = 0,
}

-- ---------------------------------------------------------------------------------------------
-- helpers

local function say(peer_id, text)
    server.announce(ADDON_NAME, text, peer_id)
end

local function cfg()
    return g_savedata.cfg
end

local function tile_name(raw)
    if type(raw) ~= "string" or raw == "" then
        return nil
    end
    local name = raw:match("([^/\\]+)$") or raw
    name = name:gsub("%.xml$", "")
    return name
end

-- "x,z,x,z,..." in decimetres -> {xs, zs}
local function decode(text)
    local xs, zs = {}, {}
    local n = 0
    for a, b in text:gmatch("(-?%d+),(-?%d+)") do
        n = n + 1
        xs[n] = tonumber(a) / 10
        zs[n] = tonumber(b) / 10
    end
    return { xs = xs, zs = zs }
end

local function family_of(name)
    local fam, x, y = name:match("^(%a+)_island_(%d+)_(%d+)$")
    if fam and FAMILY_ORIGIN[fam] then
        return fam, tonumber(x), tonumber(y)
    end
    return nil
end

-- ---------------------------------------------------------------------------------------------
-- tile scan: finds where the game placed each tile, including the reused sea line tiles

local function start_scan()
    local c = cfg()
    state.scan = {
        x = c.scan_min, z = c.scan_min,
        found = {}, seen = {},
        offsets = {}, names = {}, tiles = 0,
    }
    for fam in pairs(FAMILY_ORIGIN) do
        state.scan.offsets[fam] = { dx = 0, dz = 0, n = 0 }
    end
end

local function scan_cell(s, x, z)
    local m = matrix.translation(x, 0, z)
    local tile, ok = server.getTile(m)
    if ok == false or type(tile) ~= "table" then
        return
    end
    local name = tile_name(tile.name)
    if not name then
        return
    end
    s.tiles = s.tiles + 1
    local fam, tx, ty = family_of(name)
    if fam == nil and TILES[name] == nil then
        return
    end
    local tm, tok = server.getTileTransform(m, tile.name, 600)
    if not tok or type(tm) ~= "table" then
        tm = m
    end
    if fam then
        local o = FAMILY_ORIGIN[fam]
        local acc = s.offsets[fam]
        acc.dx = acc.dx + (tm[13] - (1000 * tx - o[1]))
        acc.dz = acc.dz + (tm[15] - (1000 * ty - o[2]))
        acc.n = acc.n + 1
        return
    end
    local key = math.floor(tm[13] + 0.5) .. "," .. math.floor(tm[15] + 0.5)
    if s.seen[key] then
        return
    end
    s.seen[key] = true
    s.found[#s.found + 1] = { name, tm[1], tm[3], tm[9], tm[11], tm[13], tm[15] }
    s.names[name] = (s.names[name] or 0) + 1
end

local rebuild

local function scan_step()
    local s = state.scan
    local c = cfg()
    for _ = 1, SCAN_PER_TICK do
        if s.z > c.scan_max then
            local offsets = {}
            for fam, acc in pairs(s.offsets) do
                if acc.n > 0 then
                    offsets[fam] = { acc.dx / acc.n, acc.dz / acc.n, acc.n }
                else
                    offsets[fam] = { 0, 0, 0 }
                end
            end
            g_savedata.instances = s.found
            g_savedata.offsets = offsets
            g_savedata.scan_tiles = s.tiles
            g_savedata.data_version = DATA_VERSION
            state.scan = nil
            rebuild()
            return
        end
        scan_cell(s, s.x, s.z)
        s.x = s.x + 1000
        if s.x > c.scan_max then
            s.x = c.scan_min
            s.z = s.z + 1000
        end
    end
end

-- ---------------------------------------------------------------------------------------------
-- geometry: polylines -> chains -> map lines

local function collect_polylines()
    local polys = {}
    local offsets = g_savedata.offsets or {}
    for fam, list in pairs(WORLD) do
        local off = offsets[fam]
        local dx, dz = 0, 0
        if off and off[3] > 0 and (math.abs(off[1]) > 1 or math.abs(off[2]) > 1) then
            dx, dz = off[1], off[2]
        end
        for _, text in ipairs(list) do
            local p = decode(text)
            for i = 1, #p.xs do
                p.xs[i] = p.xs[i] + dx
                p.zs[i] = p.zs[i] + dz
            end
            polys[#polys + 1] = p
        end
    end
    for _, inst in ipairs(g_savedata.instances or {}) do
        local list = TILES[inst[1]]
        if list then
            local m1, m3, m9, m11, m13, m15 = inst[2], inst[3], inst[4], inst[5], inst[6], inst[7]
            for _, text in ipairs(list) do
                local p = decode(text)
                for i = 1, #p.xs do
                    local x, z = p.xs[i], p.zs[i]
                    p.xs[i] = m1 * x + m9 * z + m13
                    p.zs[i] = m3 * x + m11 * z + m15
                end
                polys[#polys + 1] = p
            end
        end
    end
    return polys
end

-- end e of polyline p: point and outward unit direction (e = 1 start, 2 end)
local function end_of(p, e)
    local n = #p.xs
    local i, j
    if e == 1 then
        i, j = 1, 2
    else
        i, j = n, n - 1
    end
    local dx, dz = p.xs[i] - p.xs[j], p.zs[i] - p.zs[j]
    local l = math.sqrt(dx * dx + dz * dz)
    if l < 1e-6 then
        l = 1
    end
    return p.xs[i], p.zs[i], dx / l, dz / l
end

-- join polylines whose ends meet (tile borders, sea line tiles) into chains
local function make_chains(polys)
    local grid = {}
    local cell = JOIN_DISTANCE
    local function key(x, z)
        return math.floor(x / cell) .. ":" .. math.floor(z / cell)
    end
    local ends = {}
    for i, p in ipairs(polys) do
        if #p.xs >= 2 then
            for e = 1, 2 do
                local x, z, dx, dz = end_of(p, e)
                local item = { i = i, e = e, x = x, z = z, dx = dx, dz = dz }
                ends[#ends + 1] = item
                local k = key(x, z)
                grid[k] = grid[k] or {}
                table.insert(grid[k], item)
            end
        end
    end
    local function best_of(a)
        local best, bd = nil, JOIN_DISTANCE
        local cx, cz = math.floor(a.x / cell), math.floor(a.z / cell)
        for gx = cx - 1, cx + 1 do
            for gz = cz - 1, cz + 1 do
                for _, b in ipairs(grid[gx .. ":" .. gz] or {}) do
                    if b.i ~= a.i then
                        local d = math.sqrt((a.x - b.x) ^ 2 + (a.z - b.z) ^ 2)
                        local facing = a.dx * b.dx + a.dz * b.dz
                        if d < bd and facing < -0.5 then
                            best, bd = b, d
                        end
                    end
                end
            end
        end
        return best
    end
    local partner = {}      -- partner[i][e] = {j, f}
    for _, a in ipairs(ends) do
        local b = best_of(a)
        if b and best_of(b) == a then
            partner[a.i] = partner[a.i] or {}
            partner[a.i][a.e] = { b.i, b.e }
            -- meet in the middle
            local p, q = polys[a.i], polys[b.i]
            local ia = a.e == 1 and 1 or #p.xs
            local ib = b.e == 1 and 1 or #q.xs
            local mx, mz = (p.xs[ia] + q.xs[ib]) / 2, (p.zs[ia] + q.zs[ib]) / 2
            p.xs[ia], p.zs[ia], q.xs[ib], q.zs[ib] = mx, mz, mx, mz
        end
    end
    local used = {}
    local chains = {}
    local function walk(i, e_in)
        -- start at polyline i entering through end e_in, follow partners
        local xs, zs = {}, {}
        local cur, enter = i, e_in
        while cur and not used[cur] do
            used[cur] = true
            local p = polys[cur]
            local n = #p.xs
            local a, b, step = 1, n, 1
            if enter == 2 then
                a, b, step = n, 1, -1
            end
            for k = a, b, step do
                local m = #xs
                if m == 0 or xs[m] ~= p.xs[k] or zs[m] ~= p.zs[k] then
                    xs[m + 1] = p.xs[k]
                    zs[m + 1] = p.zs[k]
                end
            end
            local exit = enter == 1 and 2 or 1
            local nxt = partner[cur] and partner[cur][exit]
            if nxt then
                cur, enter = nxt[1], nxt[2]
            else
                cur = nil
            end
        end
        return { xs = xs, zs = zs }
    end
    for i, p in ipairs(polys) do
        if not used[i] and #p.xs >= 2 then
            local has1 = partner[i] and partner[i][1]
            local has2 = partner[i] and partner[i][2]
            if not has1 then
                chains[#chains + 1] = walk(i, 1)
            elseif not has2 then
                chains[#chains + 1] = walk(i, 2)
            end
        end
    end
    for i, p in ipairs(polys) do       -- closed rings
        if not used[i] and #p.xs >= 2 then
            chains[#chains + 1] = walk(i, 1)
        end
    end
    return chains
end

local function add_ticks(lines, ch, spacing, half)
    local xs, zs = ch.xs, ch.zs
    local n = #xs
    local total = 0
    local seg = {}
    for k = 1, n - 1 do
        local l = math.sqrt((xs[k + 1] - xs[k]) ^ 2 + (zs[k + 1] - zs[k]) ^ 2)
        seg[k] = l
        total = total + l
    end
    if total < spacing * 0.4 then
        return
    end
    local count = math.max(1, math.floor(total / spacing + 0.5))
    local step = total / count
    local k, base = 1, 0
    for t = 1, count do
        local s = (t - 0.5) * step
        while k < n - 1 and base + seg[k] < s do
            base = base + seg[k]
            k = k + 1
        end
        local l = seg[k]
        if l > 1e-6 then
            local f = (s - base) / l
            local tx, tz = (xs[k + 1] - xs[k]) / l, (zs[k + 1] - zs[k]) / l
            local px, pz = xs[k] + (xs[k + 1] - xs[k]) * f, zs[k] + (zs[k + 1] - zs[k]) * f
            lines[#lines + 1] = { px - tz * half, pz + tx * half, px + tz * half, pz - tx * half }
        end
    end
end

local function build_lines()
    local polys = collect_polylines()
    local chains = make_chains(polys)
    local c = cfg()
    local lines = {}
    for _, ch in ipairs(chains) do
        for k = 1, #ch.xs - 1 do
            lines[#lines + 1] = { ch.xs[k], ch.zs[k], ch.xs[k + 1], ch.zs[k + 1] }
        end
        if c.tick_spacing > 0 and c.tick_length > 0 then
            add_ticks(lines, ch, c.tick_spacing, c.tick_length / 2)
        end
    end
    state.polylines = #polys
    state.chains = #chains
    return lines
end

-- ---------------------------------------------------------------------------------------------
-- drawing

local function queue(kind, peer_id)
    state.jobs[#state.jobs + 1] = { kind = kind, peer = peer_id, i = 1 }
end

local function job_step()
    local job = state.jobs[1]
    if not job then
        return
    end
    local c = cfg()
    local done = 0
    if job.kind == "remove" then
        while job.i <= #state.ids and done < DRAW_PER_TICK * 4 do
            server.removeMapLine(job.peer, state.ids[job.i])
            job.i = job.i + 1
            done = done + 1
        end
        if job.i > #state.ids then
            table.remove(state.jobs, 1)
        end
        return
    end
    while job.i <= #state.lines and done < DRAW_PER_TICK do
        local l = state.lines[job.i]
        local id = state.ids[job.i]
        if not id then
            id = server.getMapID()
            state.ids[job.i] = id
        end
        server.addMapLine(job.peer, id, matrix.translation(l[1], 0, l[2]), matrix.translation(l[3], 0, l[4]),
            c.width, c.r, c.g, c.b, c.a)
        job.i = job.i + 1
        done = done + 1
    end
    if job.i > #state.lines then
        table.remove(state.jobs, 1)
    end
end

rebuild = function()
    state.jobs = {}
    if #state.ids > 0 then
        queue("remove", -1)
    end
    state.lines = build_lines()
    if cfg().on then
        queue("draw", -1)
    end
end

-- ---------------------------------------------------------------------------------------------
-- callbacks

function onCreate(is_world_create)
    g_savedata.cfg = g_savedata.cfg or {}
    for k, v in pairs(DEFAULTS) do
        if g_savedata.cfg[k] == nil then
            g_savedata.cfg[k] = v
        end
    end
    if g_savedata.data_version ~= DATA_VERSION or g_savedata.instances == nil then
        start_scan()
    else
        rebuild()
    end
end

function onTick(game_ticks)
    if state.scan then
        scan_step()
        return
    end
    job_step()
end

function onPlayerJoin(steam_id, name, peer_id, is_admin, is_auth)
    if cfg() and cfg().on and #state.lines > 0 and not state.scan then
        queue("draw", peer_id)
    end
end

local function status(peer_id)
    local c = cfg()
    local function n(v)
        local r = math.floor(v * 100 + 0.5)
        if r % 100 == 0 then
            return tostring(r // 100)
        end
        return tostring(r / 100)
    end
    say(peer_id, ADDON_NAME .. ": " .. (c.on and "on" or "off") .. ", width " .. n(c.width) ..
        ", colour " .. n(c.r) .. " " .. n(c.g) .. " " .. n(c.b) .. " " .. n(c.a) ..
        ", ticks every " .. n(c.tick_spacing) .. " m, " .. n(c.tick_length) .. " m long")
    say(peer_id, "?railmap width W | color R G B [A] | ticks SPACING LENGTH | on | off | rescan | info")
end

function onCustomCommand(full_message, user_peer_id, is_admin, is_auth, command, one, two, three, four)
    if command ~= "?railmap" then
        return
    end
    local c = cfg()
    if one == nil or one == "" or one == "help" then
        status(user_peer_id)
        return
    end
    if one == "info" then
        local found = g_savedata.instances and #g_savedata.instances or 0
        say(user_peer_id, string.format("%d track lines joined into %d, %d map lines",
            state.polylines, state.chains, #state.lines))
        say(user_peer_id, string.format("scan: %d tiles named, %d sea line tiles placed",
            g_savedata.scan_tiles or 0, found))
        for fam, off in pairs(g_savedata.offsets or {}) do
            say(user_peer_id, string.format("%s tiles seen %d, offset %.1f %.1f", fam, off[3], off[1], off[2]))
        end
        if state.scan then
            say(user_peer_id, "scan in progress")
        end
        return
    end
    if not is_admin then
        say(user_peer_id, "only an admin can change the rail map")
        return
    end
    if one == "on" then
        c.on = true
    elseif one == "off" then
        c.on = false
    elseif one == "width" and tonumber(two) then
        c.width = tonumber(two)
    elseif one == "color" and tonumber(two) and tonumber(three) and tonumber(four) then
        c.r, c.g, c.b = tonumber(two), tonumber(three), tonumber(four)
        local a = full_message:match("color%s+%S+%s+%S+%s+%S+%s+(%S+)")
        if tonumber(a) then
            c.a = tonumber(a)
        end
    elseif one == "ticks" and tonumber(two) and tonumber(three) then
        c.tick_spacing, c.tick_length = tonumber(two), tonumber(three)
    elseif one == "rescan" then
        start_scan()        -- the old lines stay until the scan is done and the map is redrawn
        say(user_peer_id, "scanning tiles")
        return
    else
        status(user_peer_id)
        return
    end
    rebuild()
    status(user_peer_id)
end
