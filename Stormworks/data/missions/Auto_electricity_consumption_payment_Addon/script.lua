-- =============================================================================
-- Electric Bill Addon for Stormworks
-- Command: ?bill_status, ?bill_check, ?bill_list, ?bill_limit, ?bill_price, ?bill_debug
-- =============================================================================

local BILL_DIAL_NAME = "energy_kwh"          -- Dial name that outputs total kWh consumed
local BLOCK_KEYPAD_NAME = "bill_block"       -- Keypad name to control "electricity" (0 - fine, 1 - locked)
local BILL_INTERVAL_TICKS = 600

local tracked = {}
local blocked = false
local tick_timer = 0
local init_welcome_done = false
local welcome_sent = false

function onCreate(is_world_create)
    g_savedata = g_savedata or {}
    if g_savedata.price == nil then g_savedata.price = 15.0 end
    if g_savedata.limit == nil then g_savedata.limit = 5000 end
    if g_savedata.debug == nil then g_savedata.debug = false end

    tracked = {}
    blocked = false
    tick_timer = 0
    init_welcome_done = false
	welcome_sent = false
end

local function apply_block_state()
    for vid, _ in pairs(tracked) do
        server.setVehicleKeypad(vid, BLOCK_KEYPAD_NAME, blocked and 1 or 0)
    end
end

local function try_add_vehicle(vehicle_id)
    if tracked[vehicle_id] then return end
    if not server.getVehicleSimulating(vehicle_id) then return end

    local dial_data, success = server.getVehicleDial(vehicle_id, BILL_DIAL_NAME)
    if success then
        local vehicle_data = server.getVehicleData(vehicle_id)
        local name = vehicle_data and vehicle_data.name or "Unnamed"
        tracked[vehicle_id] = {
            last_value = dial_data.value,
            unbilled = 0,
            name = name
        }
        server.setVehicleKeypad(vehicle_id, BLOCK_KEYPAD_NAME, blocked and 1 or 0)
    end
end

function onVehicleLoad(vehicle_id)
    try_add_vehicle(vehicle_id)
end

function onVehicleUnload(vehicle_id)
    tracked[vehicle_id] = nil
end

function onVehicleDespawn(vehicle_id, peer_id)
    tracked[vehicle_id] = nil
end

function bill_vehicles()
    for vid, data in pairs(tracked) do
        if server.getVehicleSimulating(vid) then
            local dial_data, success = server.getVehicleDial(vid, BILL_DIAL_NAME)
            if success then
                local current = dial_data.value
                local previous = data.last_value
                if previous then
                    local delta = current - previous
                    if delta < 0 then delta = 0 end
                    data.unbilled = data.unbilled + delta
                end
                data.last_value = current
            end
        end
    end

    local total_kwh = 0
    for _, data in pairs(tracked) do
        total_kwh = total_kwh + data.unbilled
    end

    if total_kwh > 0 then
        local raw_cost = total_kwh * g_savedata.price
        local cost = math.ceil(raw_cost)
        if cost < 1 then
            if g_savedata.debug then
                server.announce("[ElectricBill]", string.format(
                    "Cost %.2f (<1), skipping bill.", raw_cost))
            end
			
            for _, data in pairs(tracked) do data.unbilled = 0 end
            return
        end

        local money = server.getCurrency()
        local new_balance = money - cost

        if g_savedata.debug then
            server.announce("[ElectricBill]", string.format(
                "Billing: total_kwh=%.2f, price=%.2f, raw_cost=%.2f, final_cost=%d, balance=%d, new_balance=%d",
                total_kwh, g_savedata.price, raw_cost, cost, money, new_balance))
        end

        if new_balance >= g_savedata.limit then
            -- Correct way to set global balance: peer_id = -1, amount
            local research = server.getResearchPoints()
			server.setCurrency(new_balance, research)
            blocked = false
            if g_savedata.debug then
                server.announce("[ElectricBill]", string.format(
                    "Billed %d credits for %.2f kWh. New balance: %d",
                    cost, total_kwh, new_balance))
            end
        else
            blocked = true
            if g_savedata.debug then
                server.announce("[ElectricBill]", string.format(
                    "Insufficient funds! Need %d, balance %d < limit %d. Movement blocked.",
                    cost, money, g_savedata.limit))
            end
        end

        for _, data in pairs(tracked) do
            data.unbilled = 0
        end
    end

    apply_block_state()
end

function onTick(game_ticks)
    tick_timer = tick_timer + game_ticks

    if not init_welcome_done then
		if tick_timer >= 120 and not welcome_sent then
			server.announce("[ElectricBill]", "Addon active. Use ?bill_check for info.")
			welcome_sent = true

		elseif tick_timer >= 150 then
			local money = server.getCurrency()
			local price = g_savedata.price or 1.0
			local limit = g_savedata.limit or 5000
			local count = 0
			for _ in pairs(tracked) do count = count + 1 end

			server.announce("[ElectricBill]", string.format(
				"============Status============\nBalance - %d $,\nLimit - %d $,\nPrice - %d $/kWh,\nElectro Locomotives - %d,\nBlocked - %s",
				money, limit, price, count, blocked and "YES" or "NO"))

			init_welcome_done = true
		end
	end

    if tick_timer >= BILL_INTERVAL_TICKS then
        tick_timer = 0
        bill_vehicles()
    end
end

function onCustomCommand(full_message, user_peer_id, is_admin, is_auth, command, ...)
    local args = {...}

    if command == "?bill_debug" then
        g_savedata.debug = not g_savedata.debug
        server.announce("[ElectricBill]", "Debug mode " .. (g_savedata.debug and "ON" or "OFF"))

    elseif command == "?bill_check" then
        server.announce("[ElectricBill]", "Addon is working. Set limit with: ?bill_limit <value>")

    elseif command == "?bill_list" then
        if next(tracked) == nil then
            server.announce("[ElectricBill]", "No electric locomotives detected.")
        else
            server.announce("[ElectricBill]", "--- Tracked Locomotives ---")
            for vid, data in pairs(tracked) do
                local dial_val = "N/A"
                local dial_data, success = server.getVehicleDial(vid, BILL_DIAL_NAME)
                if success then dial_val = string.format("%.2f", dial_data.value) end
                server.announce("[ElectricBill]", string.format(
                    "ID %d | %s | Consumption: %s kWh",
                    vid, data.name, dial_val))
            end
        end

    elseif command == "?bill_limit" then
        local val = tonumber(args[1])
        if not val or val < 0 then
            server.announce("[ElectricBill]", "Usage: ?bill_limit <non-negative number>")
        else
            g_savedata.limit = val
            server.announce("[ElectricBill]", "Limit set to " .. val)
        end

    elseif command == "?bill_status" then
        local money = server.getCurrency()
        local price = g_savedata.price or 1.0
        local limit = g_savedata.limit or 5000
        local count = 0
        for _ in pairs(tracked) do count = count + 1 end
        server.announce("[ElectricBill]", string.format(
            "Balance: %d | Limit: %d | Price/kWh: %d | Locomotives: %d | Blocked: %s",
            money, limit, price, count, blocked and "YES" or "NO"))

    elseif command == "?bill_price" then
        if not is_admin then
            server.announce("[ElectricBill]", "Admin only.")
            return
        end
        local val = tonumber(args[1])
        if not val or val <= 0 then
            server.announce("[ElectricBill]", "Usage: ?bill_price <positive number>")
        else
            g_savedata.price = val
            server.announce("[ElectricBill]", "Price set to " .. val .. " credits/kWh")
        end

    elseif command == "?bill_block" then
        if not is_admin then
            server.announce("[ElectricBill]", "Admin only.")
            return
        end
        local state_str = args[1]
        if state_str == "true" then
            blocked = true
            server.announce("[ElectricBill]", "Movement manually BLOCKED.")
        elseif state_str == "false" then
            blocked = false
            server.announce("[ElectricBill]", "Movement manually UNBLOCKED.")
        else
            server.announce("[ElectricBill]", "Usage: ?bill_block true/false")
        end
        apply_block_state()
    end
end