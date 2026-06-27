c_cmd = "?widget"
c_ver = "v0.1.0"
c_spd_unit_tbl = {
    ["km/h"] = 216,
    ["kph"] = 216,
    ["kmph"] = 216,
    ["km/hr"] = 216,

    ["m/s"] = 60,
    ["mps"] = 60,

    ["mph"] = 216000.0/1609.344,
    ["MPH"] = 216000.0/1609.344,

    ["kt"] = 216000.0/1852.0,
    ["kn"] = 216000.0/1852.0,
}
c_alt_unit_tbl = {
    ["m"] = 1,
    ["ft"] = 1.0/0.3048,
}

function onCustomCommand(full_message, user_peer_id, is_admin, is_auth, cmd, ...)
    if cmd ~= c_cmd or user_peer_id < 0 then
        return
    end

    local args = {}
    for _, s in ipairs({...}) do
        if s ~= "" then
            table.insert(args, s)
        end
    end

    if #args <= 0 or args[1] == "help" then
        execHelp(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "version" then
        execVersion(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "on" then
        execOn(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "off" then
        execOff(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "spdofs" then
        execSpdOfs(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "altofs" then
        execAltOfs(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "spdunit" then
        execSpdUnit(user_peer_id, is_admin, is_auth, args)
    elseif args[1] == "altunit" then
        execAltUnit(user_peer_id, is_admin, is_auth, args)
    else
        server.announce(
            getAnnounceName(),
            string.format(
                (
                    'error: undefined subcommand "%s"\n' ..
                    'see "%s help" for list of subcommands'
                ),
                args[1],
                c_cmd
            ),
            user_peer_id
        )
    end

    saveAddon()
end

function execHelp(user_peer_id, is_admin, is_auth, args)
    server.announce(
        getAnnounceName(),
        (
            c_cmd .. " on\n" ..
            c_cmd .. " off\n" ..
            c_cmd .. " spdofs HOFS VOFS\n" ..
            c_cmd .. " altofs HOFS VOFS\n" ..
            c_cmd .. " spdunit UNIT\n" ..
            c_cmd .. " altunit UNIT\n" ..
            c_cmd .. " help\n" ..
            c_cmd .. " version"
        ),
        user_peer_id
    )
end

function execVersion(user_peer_id, is_admin, is_auth, args)
    if #args > 1 then
        server.announce(getAnnounceName(), "error: extra arguments", user_peer_id)
        return
    end

    local msg = getAddonName() .. " " .. c_ver
    server.announce(getAnnounceName(), msg, user_peer_id)
end

function execOn(user_peer_id, is_admin, is_auth, args)
    if #args > 1 then
        server.announce(getAnnounceName(), "error: extra arguments", user_peer_id)
        return
    end
    g_userdata[user_peer_id].enabled = true
    server.announce(getAnnounceName(), "widgets are now enabled", user_peer_id)
end

function execOff(user_peer_id, is_admin, is_auth, args)
    if #args > 1 then
        server.announce(getAnnounceName(), "error: extra arguments", user_peer_id)
        return
    end
    g_userdata[user_peer_id].enabled = false
    server.announce(getAnnounceName(), "widgets are now disabled", user_peer_id)
end

function execSpdOfs(user_peer_id, is_admin, is_auth, args)
    return execSetOfs(user_peer_id, is_admin, is_auth, args, "spdofs", "spd_hofs", "spd_vofs")
end

function execAltOfs(user_peer_id, is_admin, is_auth, args)
    return execSetOfs(user_peer_id, is_admin, is_auth, args, "altofs", "alt_hofs", "alt_vofs")
end

function execSetOfs(user_peer_id, is_admin, is_auth, args, param_name, param_key_hofs, param_key_vofs)
    if #args == 1 then
        server.announce(
            getAnnounceName(),
            string.format(
                (
                    "current %s is (%f, %f)\n" ..
                    'use "%s %s HOFS VOFS" to configure'
                ),
                param_name,
                g_userdata[user_peer_id][param_key_hofs],
                g_userdata[user_peer_id][param_key_vofs],
                c_cmd,
                args[1]
            ),
            user_peer_id
        )
        return
    elseif #args ~= 3 then
        server.announce(getAnnounceName(), "error: wrong number of arguments", user_peer_id)
        return
    end

    local hofs_txt = args[2]
    local vofs_txt = args[3]
    local hofs = tonumber(hofs_txt)
    local vofs = tonumber(vofs_txt)
    if not hofs then
        server.announce(
            getAnnounceName(),
            string.format('error: got invalid number "%s"', hofs_txt),
            user_peer_id
        )
        return
    elseif not vofs then
        server.announce(
            getAnnounceName(),
            string.format('error: got invalid number "%s"', vofs_txt),
            user_peer_id
        )
        return
    elseif hofs < -1 or 1 < hofs or vofs < -1 or 1 < vofs then
        server.announce(getAnnounceName(), "error: offset must be within the range -1 to 1", user_peer_id)
        return
    end

    g_userdata[user_peer_id][param_key_hofs] = hofs
    g_userdata[user_peer_id][param_key_vofs] = vofs
    server.announce(
        getAnnounceName(),
        string.format("%s is now set to (%f, %f)", param_name, hofs, vofs),
        user_peer_id
    )
end

function execSpdUnit(user_peer_id, is_admin, is_auth, args)
    return execSetUnit(
        user_peer_id,
        is_admin,
        is_auth,
        args,
        "spdunit",
        "spd_unit",
        c_spd_unit_tbl,
        'available units are "km/h", "m/s", "mph", "kt"'
    )
end

function execAltUnit(user_peer_id, is_admin, is_auth, args)
    return execSetUnit(
        user_peer_id,
        is_admin,
        is_auth,
        args,
        "altunit",
        "alt_unit",
        c_alt_unit_tbl,
        'available units are "m", "ft"'
    )
end

function execSetUnit(user_peer_id, is_admin, is_auth, args, param_name, param_key, param_tbl, param_choices)
    if #args < 2 then
        server.announce(
            getAnnounceName(),
            string.format(
                (
                    'current %s is "%s"\n' ..
                    'use "%s %s UNIT" to configure\n' ..
                    "%s"
                ),
                param_name,
                g_userdata[user_peer_id][param_key],
                c_cmd,
                args[1],
                param_choices
            ),
            user_peer_id
        )
        return
    end
    if #args > 2 then
        server.announce(getAnnounceName(), "error: too many arguments", user_peer_id)
        return
    end

    local unit = args[2]
    if param_tbl[unit] == nil then
        server.announce(
            getAnnounceName(),
            string.format('error: got undefined unit "%s"\n%s', unit, param_choices),
            user_peer_id
        )
        return
    end
    g_userdata[user_peer_id][param_key] = unit
    server.announce(
        getAnnounceName(),
        string.format('%s is now set to "%s"', param_name, unit),
        user_peer_id
    )
end

function onTick(game_ticks)
    syncPlayers()

    for peer_id, _ in pairs(g_userdata) do
        if not g_userdata[peer_id].enabled then
            goto continue
        end

        local spd, alt = g_tracker:getAstroSpdAlt(peer_id)
        g_uim:setPopupScreen(
            peer_id,
            g_spd_ui_id,
            "",
            true,
            formatSpd(spd, g_userdata[peer_id].spd_unit),
            g_userdata[peer_id].spd_hofs,
            g_userdata[peer_id].spd_vofs
        )
        g_uim:setPopupScreen(
            peer_id,
            g_alt_ui_id,
            "",
            true,
            formatAlt(alt, g_userdata[peer_id].alt_unit),
            g_userdata[peer_id].alt_hofs,
            g_userdata[peer_id].alt_vofs
        )
        ::continue::
    end

    g_tracker:tick()
    g_uim:flushPopup()
end

function onCreate(is_world_create)
    g_userdata = {[0] = newUserData()}
    g_spd_ui_id = nil
    g_alt_ui_id = nil
    g_tracker = buildTracker()
    g_uim = buildUIManager()

    loadAddon()

    if g_spd_ui_id == nil then
        g_spd_ui_id = server.getMapID()
    end
    if g_alt_ui_id == nil then
        g_alt_ui_id = server.getMapID()
    end
    saveAddon()

    server.removePopup(-1, g_spd_ui_id)
    server.removePopup(-1, g_alt_ui_id)
end

function onDestroy()
    server.removePopup(-1, g_spd_ui_id)
    server.removePopup(-1, g_alt_ui_id)
end

function onPlayerJoin(steam_id, name, peer_id, is_admin, is_auth)
    g_uim:onPlayerJoin(steam_id, name, peer_id, is_admin, is_auth)
end

function syncPlayers()
    local player_exists = { [0] = true }
    local player_list = server.getPlayers()
    for _, player in pairs(player_list) do
        player_exists[player.id] = true
    end

    local userdata_tbl = {}
    for peer_id, _ in pairs(player_exists) do
        local userdata = g_userdata[peer_id]
        if userdata == nil then
            userdata = newUserData()
        end
        userdata_tbl[peer_id] = userdata
    end
    g_userdata = userdata_tbl
end

function formatSpd(spd, spd_unit)
    if type(spd) ~= "number" or type(spd_unit) ~= "string" or c_spd_unit_tbl[spd_unit] == nil then
        return "SPD\n---\n---"
    end

    local speed_main = spd * c_spd_unit_tbl[spd_unit]
    local speed_ms = spd * c_spd_unit_tbl["m/s"]
    return string.format(
        "SPD\n%.2f%s\n%.2fm/s",
        speed_main, spd_unit, speed_ms
    )
end

function formatAlt(alt, alt_unit)
    if type(alt) ~= "number" or type(alt_unit) ~= "string" or c_alt_unit_tbl[alt_unit] == nil then
        return "ALT\n---"
    end

    return string.format(
        "ALT\n%.2f%s",
        alt*c_alt_unit_tbl[alt_unit],
        alt_unit
    )
end

function newUserData()
    return {
        enabled = true,
        spd_hofs = 0.8,
        spd_vofs = -0.9,
        spd_unit = "km/h",
        alt_hofs = 0.9,
        alt_vofs = -0.8,
        alt_unit = "m",
    }
end

function loadAddon()
    if type(g_savedata) == "table" and g_savedata.version == 1 then
        if type(g_savedata.spd_ui_id) == "number" and g_savedata.spd_ui_id%1 == 0 then
            g_spd_ui_id = g_savedata.spd_ui_id
        end
        if type(g_savedata.alt_ui_id) == "number" and g_savedata.alt_ui_id%1 == 0 then
            g_alt_ui_id = g_savedata.alt_ui_id
        end
        if type(g_savedata.hostdata) == "table" and g_userdata[0] ~= nil then
            if type(g_savedata.hostdata.enabled) == "boolean" then
                g_userdata[0].enabled = g_savedata.hostdata.enabled
            end
            if type(g_savedata.hostdata.spd_hofs) == "number" and -1 <= g_savedata.hostdata.spd_hofs and g_savedata.hostdata.spd_hofs <= 1 then
                g_userdata[0].spd_hofs = g_savedata.hostdata.spd_hofs
            end
            if type(g_savedata.hostdata.spd_vofs) == "number" and -1 <= g_savedata.hostdata.spd_vofs and g_savedata.hostdata.spd_vofs <= 1 then
                g_userdata[0].spd_vofs = g_savedata.hostdata.spd_vofs
            end
            if type(g_savedata.hostdata.spd_unit) == "string" and c_spd_unit_tbl[g_savedata.hostdata.spd_unit] ~= nil then
                g_userdata[0].spd_unit = g_savedata.hostdata.spd_unit
            end
            if type(g_savedata.hostdata.alt_hofs) == "number" and -1 <= g_savedata.hostdata.alt_hofs and g_savedata.hostdata.alt_hofs <= 1 then
                g_userdata[0].alt_hofs = g_savedata.hostdata.alt_hofs
            end
            if type(g_savedata.hostdata.alt_vofs) == "number" and -1 <= g_savedata.hostdata.alt_vofs and g_savedata.hostdata.alt_vofs <= 1 then
                g_userdata[0].alt_vofs = g_savedata.hostdata.alt_vofs
            end
            if type(g_savedata.hostdata.alt_unit) == "string" and c_alt_unit_tbl[g_savedata.hostdata.alt_unit] ~= nil then
                g_userdata[0].alt_unit = g_savedata.hostdata.alt_unit
            end
        end
    end
end

function saveAddon()
    local savedata = {
        version = 1,
        spd_ui_id = g_spd_ui_id,
        alt_ui_id = g_alt_ui_id,
        hostdata = nil,
    }

    if g_userdata[0] ~= nil then
        savedata.hostdata = {
            enabled = g_userdata[0].enabled,
            spd_hofs = g_userdata[0].spd_hofs,
            spd_vofs = g_userdata[0].spd_vofs,
            spd_unit = g_userdata[0].spd_unit,
            alt_hofs = g_userdata[0].alt_hofs,
            alt_vofs = g_userdata[0].alt_vofs,
            alt_unit = g_userdata[0].alt_unit,
        }
    end

    g_savedata = savedata
end

function buildTracker()
    local tracker = {
        _player_pos_old = {},
        _player_pos_new = {},
        _vehicle_pos_old = {},
        _vehicle_pos_new = {},
    }

    function tracker:getAstroSpdAlt(peer_id)
        local spd, pos = self:getAstroSpdPos(peer_id)

        local alt = nil
        if pos ~= nil then
            local _
            _, alt, _ = matrix.position(pos)
        end
        return spd, alt
    end

    function tracker:getAstroSpdPos(peer_id)
        local spd, world_pos = self:getWorldSpdPos(peer_id)

        local astro_pos = nil
        if world_pos ~= nil then
            astro_pos = server.getAstroPos(world_pos)
        end
        return spd, astro_pos
    end

    function tracker:getWorldSpdPos(peer_id)
        local vehicle_id, is_success = getPlayerVehicle(peer_id)
        if is_success then
            return self:getVehicleWorldSpdPos(vehicle_id)
        end
        return self:getPlayerWorldSpdPos(peer_id)
    end

    function tracker:getPlayerWorldSpdPos(peer_id)
        local player_pos_new = self._player_pos_new[peer_id]
        if player_pos_new == nil then
            local is_success
            player_pos_new, is_success = getPlayerPos(peer_id)
            if not is_success then
                return nil, nil
            end
            self._player_pos_new[peer_id] = player_pos_new
        end

        local spd = nil
        local pos = {table.unpack(player_pos_new)}
        local player_pos_old = self._player_pos_old[peer_id]
        if player_pos_old ~= nil then
            spd = matrix.distance(player_pos_old, player_pos_new)
        end
        return spd, pos
    end

    function tracker:tickPlayer()
        self._player_pos_old = self._player_pos_new
        self._player_pos_new = {}
    end

    function tracker:getVehicleWorldSpdPos(vehicle_id)
        local vehicle_pos_new = self._vehicle_pos_new[vehicle_id]
        if vehicle_pos_new == nil then
            local is_success
            vehicle_pos_new, is_success = server.getVehiclePos(vehicle_id)
            if not is_success then
                return nil, nil
            end
            self._vehicle_pos_new[vehicle_id] = vehicle_pos_new
        end

        local spd = nil
        local pos = {table.unpack(vehicle_pos_new)}
        local vehicle_pos_old = self._vehicle_pos_old[vehicle_id]
        if vehicle_pos_old ~= nil then
            spd = matrix.distance(vehicle_pos_old, vehicle_pos_new)
        end
        return spd, pos
    end

    function tracker:tickVehicle()
        self._vehicle_pos_old = self._vehicle_pos_new
        self._vehicle_pos_new = {}
    end

    function tracker:tick()
        self:tickPlayer()
        self:tickVehicle()
    end

    return tracker
end

function buildUIManager()
    local uim = {
        _popup_old = {},
        _popup_new = {},
    }

    function uim:setPopupScreen(peer_id, ui_id, name, is_show, text, horizontal_offset, vertical_offset)
        if peer_id < 0 then
            -- peer_id=-1 is not supported
            return
        end

        local key = string.pack("jj", peer_id, ui_id)
        self._popup_new[key] = {
            peer_id = peer_id,
            ui_id = ui_id,
            name = name,
            is_show = is_show,
            text = text,
            horizontal_offset = horizontal_offset,
            vertical_offset = vertical_offset,
        }
    end

    function uim:flushPopup()
        for key, popup in pairs(self._popup_old) do
            if self._popup_new[key] == nil then
                server.removePopup(popup.peer_id, popup.ui_id)
            end
        end

        for key, popup_new in pairs(self._popup_new) do
            local popup_old = self._popup_old[key]
            if popup_old == nil or
                popup_new.name ~= popup_old.name or
                popup_new.is_show ~= popup_old.is_show or
                popup_new.text ~= popup_old.text or
                popup_new.horizontal_offset ~= popup_old.horizontal_offset or
                popup_new.vertical_offset ~= popup_old.vertical_offset then
                server.setPopupScreen(
                    popup_new.peer_id,
                    popup_new.ui_id,
                    popup_new.name,
                    popup_new.is_show,
                    popup_new.text,
                    popup_new.horizontal_offset,
                    popup_new.vertical_offset
                )
            end
        end

        self._popup_old = self._popup_new
        self._popup_new = {}
    end

    function uim:onPlayerJoin(steam_id, name, peer_id, is_admin, is_auth)
        for key, popup in pairs(self._popup_old) do
            if popup.peer_id == peer_id then
                server.removePopup(popup.peer_id, popup.ui_id)
                self._popup_old[key] = nil
            end
        end
    end

    return uim
end

function getPlayerPos(peer_id)
    local object_id, is_success = server.getPlayerCharacterID(peer_id)
    if not is_success then
        return nil, false
    end

    local object_pos, is_success = server.getObjectPos(object_id)
    if not is_success then
        return nil, false
    end

    return object_pos, true
end

function getAddonName()
    local addon_index, is_success = server.getAddonIndex()
    if not is_success then
        return "???"
    end

    local addon_data = server.getAddonData(addon_index)
    if addon_data == nil then
        return "???"
    end

    return addon_data.name
end

function getAnnounceName()
    return string.format("[%s]", getAddonName())
end

function getPlayerVehicle(peer_id)
    local object_id, is_success = server.getPlayerCharacterID(peer_id)
    if not is_success then
        return 0, false
    end
    return server.getCharacterVehicle(object_id)
end
