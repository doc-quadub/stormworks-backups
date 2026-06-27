local N = 10
local COMMAND = "?ECC"

function onCreate(is_world_create)
    g_savedata = g_savedata or {}
end

function onCustomCommand(full_message, user_peer_id, is_admin, is_auth, cmd, ...)
    if user_peer_id < 0 then return end
    if string.lower(cmd) ~= string.lower(COMMAND) then return end
    local args = {}
    for _, s in ipairs({...}) do
        if s:match("%S") then
            table.insert(args, s)
        end
    end
    if #args == 0 then
        server.notify(user_peer_id, "Error", "Use: ?ECC <consumption number>", 10)
        return
    end

    local input_val = tonumber(args[1])
    if input_val == nil or input_val <= 0 then
        server.notify(user_peer_id, "Error", "Specify a positive number.", 5)
        return
    end
    local cost = math.floor(input_val * N + 0.5)
    local current_money = server.getCurrency()

    if current_money >= cost then
        server.setCurrency(current_money - cost, current_rp)
        server.notify(user_peer_id, "Successfully", 
            string.format("$%d was debited. New balance: $%d", cost, current_money - cost), 5)
    else
        server.notify(user_peer_id, "Insufficient funds", 
            string.format("Requires $%d, you have $%d", cost, current_money), 3)
    end
end