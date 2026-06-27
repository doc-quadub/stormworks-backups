-- =============================================
-- Vehicle ID Checker Addon
-- Command: ?idchk (look at the vehicle)
-- =============================================

tracked_vehicles = {}

local MAX_DISTANCE = 500
local MAX_ANGLE_DOT = 0.95

function onVehicleLoad(vehicle_id)
    tracked_vehicles[vehicle_id] = true
end

function onVehicleUnload(vehicle_id)
    tracked_vehicles[vehicle_id] = nil
end

function onVehicleDespawn(vehicle_id)
    tracked_vehicles[vehicle_id] = nil
end

local function normalize(x, y, z)
    local len = math.sqrt(x*x + y*y + z*z)
    if len == 0 then return 0,0,0 end
    return x/len, y/len, z/len
end

local function dot(ax, ay, az, bx, by, bz)
    return ax*bx + ay*by + az*bz
end

local function find_vehicle_in_view(peer_id)
    local player_matrix, ok = server.getPlayerPos(peer_id)
    if not ok then return nil end

    local px, py, pz = matrix.position(player_matrix)

    local lx, ly, lz, ok2 = server.getPlayerLookDirection(peer_id)
    if not ok2 then return nil end

    lx, ly, lz = normalize(lx, ly, lz)

    local best_vehicle = nil
    local best_distance = MAX_DISTANCE

    for vid, _ in pairs(tracked_vehicles) do
        if server.getVehicleSimulating(vid) then
            local vmatrix, vok = server.getVehiclePos(vid)
            if vok then
                local vx, vy, vz = matrix.position(vmatrix)
				
                local dx = vx - px
                local dy = vy - py
                local dz = vz - pz

                local dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if dist <= MAX_DISTANCE then
                    local ndx, ndy, ndz = normalize(dx, dy, dz)

                    local d = dot(lx, ly, lz, ndx, ndy, ndz)

                    if d > MAX_ANGLE_DOT then
                        if dist < best_distance then
                            best_distance = dist
                            best_vehicle = vid
                        end
                    end
                end
            end
        end
    end

    return best_vehicle
end

function onCustomCommand(full_message, peer_id, is_admin, is_auth, command, ...)
    if command == "?idchk" then
        local vid = find_vehicle_in_view(peer_id)

        if vid then
            server.notify(peer_id, "[ID CHECK]", "Vehicle ID: " .. vid, 7)
        else
            server.notify(peer_id, "[ID CHECK]", "No vehicle in sight", 7)
        end
    end
end