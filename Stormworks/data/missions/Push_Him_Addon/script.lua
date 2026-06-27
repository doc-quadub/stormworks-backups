-- =============================================
-- Vehicle Push Addon
-- Command: ?push <vehicle_id>
-- =============================================

active_pushes = {}

local INITIAL_FORCE = 2.5
local DECAY = 0.85
local MIN_FORCE = 0.05

function onTick(game_ticks)
    for vid, data in pairs(active_pushes) do
        if not server.getVehicleSimulating(vid) then
            active_pushes[vid] = nil
        else
            local m, ok = server.getVehiclePos(vid)
            if ok then
                local x, y, z = matrix.position(m)

                local nx = x + data.dx * data.force
                local ny = y + data.dy * data.force
                local nz = z + data.dz * data.force

                server.moveVehicle(vid, matrix.translation(nx, ny, nz))

                data.force = data.force * DECAY

                if data.force < MIN_FORCE then
                    active_pushes[vid] = nil
                end
            else
                active_pushes[vid] = nil
            end
        end
    end
end

function onCustomCommand(full_message, peer_id, is_admin, is_auth, command, arg1)
    if command == "?push" then
        local vid = tonumber(arg1)

        if not vid then
            server.notify(peer_id, "[PUSH]", "Usage: ?push <vehicle_id>", 7)
            return
        end

        if not server.getVehicleSimulating(vid) then
            server.notify(peer_id, "[PUSH]", "Vehicle not found / not loaded", 7)
            return
        end

        local dx, dy, dz, ok = server.getPlayerLookDirection(peer_id)
        if not ok then return end

        local len = math.sqrt(dx*dx + dy*dy + dz*dz)
        if len == 0 then return end
        dx, dy, dz = dx/len, dy/len, dz/len

        active_pushes[vid] = {
            dx = dx,
            dy = dy,
            dz = dz,
            force = INITIAL_FORCE
        }

        server.notify(peer_id, "[PUSH]", "Pushing vehicle " .. vid, 7)
    end
end