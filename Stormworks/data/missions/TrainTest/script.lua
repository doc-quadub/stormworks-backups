-- =============================================
-- Improved Train Inertia Addon
-- =============================================

vehicles = {}

local DRAG = 1.01
local SMOOTH = 0.0001

function onVehicleLoad(vehicle_id)
    local data, ok = server.getVehicleData(vehicle_id)
    if ok and string.find(data.name, "TRAIN") then
        local m = server.getVehiclePos(vehicle_id)
        local x,y,z = matrix.position(m)

        vehicles[vehicle_id] = {
            last_x = x,
            last_y = y,
            last_z = z,
            vx = 0,
            vy = 0,
            vz = 0
        }
    end
end

function onVehicleUnload(vehicle_id)
    vehicles[vehicle_id] = nil
end

function onTick()
    for vid, v in pairs(vehicles) do
        if server.getVehicleSimulating(vid) then
            local m, ok = server.getVehiclePos(vid)
            if ok then
                local x,y,z = matrix.position(m)

                local dx = x - v.last_x
                local dy = y - v.last_y
                local dz = z - v.last_z

                v.vx = v.vx * (1 - SMOOTH) + dx * SMOOTH
                v.vy = v.vy * (1 - SMOOTH) + dy * SMOOTH
                v.vz = v.vz * (1 - SMOOTH) + dz * SMOOTH

                v.vx = v.vx * DRAG
                v.vy = v.vy * DRAG
                v.vz = v.vz * DRAG

                local nx = x + v.vx
                local ny = y + v.vy
                local nz = z + v.vz

                server.moveVehicle(vid, matrix.translation(nx, ny, nz))

                v.last_x = nx
                v.last_y = ny
                v.last_z = nz
            end
        end
    end
end