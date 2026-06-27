function drawCircleSmooth(cx, cy, radius)
    local x = radius
    local y = 0
    local err = 0
    while x >= y do
        screen.drawRect(cx + x, cy + y, 1, 1)
        screen.drawRect(cx + y, cy + x, 1, 1)
        screen.drawRect(cx - y, cy + x, 1, 1)
        screen.drawRect(cx - x, cy + y, 1, 1)
        screen.drawRect(cx - x, cy - y, 1, 1)
        screen.drawRect(cx - y, cy - x, 1, 1)
        screen.drawRect(cx + y, cy - x, 1, 1)
        screen.drawRect(cx + x, cy - y, 1, 1)
        if err <= 0 then
            y = y + 1
            err = err + 2 * y + 1
        end
        if err > 0 then
            x = x - 1
            err = err - 2 * x + 1
        end
    end
end

function floor(n)
    if n >= 0 then
        return n - n % 1
    else
        local i = n - n % 1
        if n ~= i then
            i = i - 1
        end
        return i
    end
end

function abs(n)
    if n < 0 then
        return -n
    else
        return n
    end
end

function round(num, n)
    local mult = 10^n
    return floor(num * mult + 0.5) / mult
end

function onTick()
    cs = input.getNumber(1) -- Compass Sensor
    speed = input.getNumber(2) -- Direct Speed Forward
    alt = input.getNumber(3) -- My Altitude
    bottom_dist = input.getNumber(4) -- Distance to Bottom
    dh = input.getNumber(7) -- Depth Hold Data
	hs = input.getNumber(8) -- Horizontal Speed Forward
	vs = input.getNumber(9) -- Vertical Speed
	fhs = input.getNumber(10) -- Front Horizontal Speed
	rhs = input.getNumber(11) -- Right Horizontal Speed

-- == Compass ==
    compass_y = 4
    compass_x1 = 7
    compass_x2 = 93
    compass_w = compass_x2 - compass_x1
    normalized_cs = (cs + 0.5) % 1.0
	x = compass_x1 + (1 - normalized_cs) * compass_w
	x = math.max(compass_x1, math.min(compass_x2, x))

-- == Speed ==
    speed_max_fwd = 15
    speed_max_rev = -8

    speedY_risk = 0
    speedY_text = 0

    if speed >= 0 then
        if speed <= speed_max_fwd then
            speedY = 79 - (speed / speed_max_fwd) * 66
        else
            speedY = 16
        end
    else
        if speed >= speed_max_rev then
            speedY = 79 - (speed / speed_max_rev) * 66
        else
            speedY = 16
        end
    end

    speedY_risk = math.max(17, math.min(79, speedY))
    speedY_text = math.max(17, math.min(75, speedY))

	if speed < 0 then
		hs = hs * (-1)
	end

-- == Alt == 
    if alt >= 0 then
        depth = 0.1
        depthY = 16
    else
        depth = -alt
        bottom_depth = depth + bottom_dist
        depthY = 16 + (depth / bottom_depth) * 66
        if depthY > 82 - 2 then depthY = 82 - 2 end
        if depthY < 16 then depthY = 16 end
    end
end

function onDraw()
    local s = screen
    local sc = s.setColor
    local dl = s.drawLine
    local dt = s.drawText
	local dr = s.drawRect
	local drf = s.drawRectF

-- == Maket ==
	local s = screen
    local sc = s.setColor
    local dl = s.drawLine
    local dt = s.drawText
	local dr = s.drawRect
	local drf = s.drawRectF

    sc(32, 32, 32)
	dt(0, 24, "O")
    dt(0, 35, "1")
    dt(0, 46, "2")
    dt(0, 57, "3")
	dt(0, 68, "C")
	
	if touch0 then
		sc(24, 24, 24)
		drf(0, 23, 5, 7)
		sc(64, 64, 64)
		dt(0, 24, "O")
	elseif touch1 then
		sc(24, 24, 24)
		drf(0, 34, 5, 7)
		sc(64, 64, 64)
		dt(0, 35, "1")
	elseif touch2 then
		sc(24, 24, 24)
		drf(0, 45, 5, 7)
		sc(64, 64, 64)
		dt(0, 46, "2")
	elseif touch3 then
		sc(24, 24, 24)
		drf(0, 56, 5, 7)
		sc(64, 64, 64)
		dt(0, 57, "3")
	elseif touch4 then
		sc(24, 24, 24)
		drf(0, 67, 5, 7)
		sc(64, 64, 64)
		dt(0, 68, "C")
	end
		
	sc(16, 16, 16)
    drf(0, 0, 5, 23)
    drf(0, 74, 5, 24)

    sc(96, 96, 96)
    dr(5, 0, 90, 95)

-- == Compass ==
    sc(255, 69, 0)
    dl(7, compass_y, 93, compass_y)

    sc(255, 0, 0)
	local risk_x = math.max(compass_x1, math.min(compass_x2, x))
    dl(risk_x, compass_y - 2, risk_x, compass_y + 2)

    sc(96, 96, 96)
    local dir = floor((normalized_cs + 0.5) * 360) % 360
	if dir >= 100 then
    	dt(42, compass_y + 4, string.format("%d", floor(dir)))
	elseif dir >= 10 then
		dt(45, compass_y + 4, string.format("%d", floor(dir)))
	else
		dt(47, compass_y + 4, string.format("%d", floor(dir)))
	end

-- == Speed ==
    sc(255, 69, 0)
    dl(7, 16, 7, 80)
	dl(7, 16, 9, 16)
	dl(7, 80, 9, 80)
    sc(96, 96, 96)
    dt(7, 10, "SPD")
	if abs(vs) <= 10 then
		dt(7, 82, string.format("VERT:%.1f", vs))
	else
		dt(7, 82, string.format("VERT:%d", floor(vs)))
	end
	
	if abs(hs) <= 10 then
		dt(7, 88, string.format("HORZ:%.1f", hs))
	else
		dt(7, 88, string.format("HORZ:%d", floor(hs)))
	end

	if speed >= speed_max_fwd then
		sc(255, 34, 0)
	elseif speed <= speed_max_rev then
		sc(255, 34, 0)
	else
		sc(255, 197, 0)
	end
	
    dl(7, speedY_risk, 10, speedY_risk)
	
	if speed <= 9.9 and speed > -2 then
	    dt(11, speedY_text, string.format("%.1f", speed))	
	else
		dt(11, speedY_text, string.format("%d", floor(speed)))
	end

-- == Alt ==
    sc(255, 69, 0)
    dl(93, 16, 93, 80)
	dl(92, 16, 93, 16)
	dl(92, 80, 94, 80)
    sc(96, 96, 96)
    dt(81, 10, "ALT")

	if alt < 0 then
		sc(255, 0, 0)
        dl(91, depthY, 94, depthY)
		if depth >= 100 then
			dt(76, depthY, string.format("%d", floor(depth)))
		elseif depth >= 10 then
			dt(81, depthY, string.format("%d", floor(depth)))
		else
			dt(76, depthY, string.format("%.1f", depth))
		end
	end

-- == Bottom Text ==
    sc(96, 96, 96)
	if bottom_dist+depth <= 10 then
		dt(52, 82, string.format("BOTM:%.1f", bottom_dist+depth))
	else
		dt(52, 82, string.format("BOTM:%d", floor(bottom_dist+depth)))
	end
	if -dh <= 9.9 then
		dt(52, 88, string.format("DHLD:%.1f", -dh))
	else
		dt(52, 88, string.format("DHLD:%d", floor(-dh)))
	end
	
-- == Velocity Vector ==
	sc(96, 96, 96)
	drawCircleSmooth(48, 48, 22)
	
	local max_horizontal_speed = 4
	local circle_inner_radius = 20
	
	local vector_x = (rhs / max_horizontal_speed) * circle_inner_radius
	local vector_y = (fhs / max_horizontal_speed) * circle_inner_radius
	
	local vector_length = math.sqrt(vector_x * vector_x + vector_y * vector_y)
	if vector_length > circle_inner_radius then
		vector_x = (vector_x / vector_length) * circle_inner_radius
		vector_y = (vector_y / vector_length) * circle_inner_radius
	end
	
	local center_x, center_y = 48, 48
	local end_x = center_x + vector_x
	local end_y = center_y - vector_y
	
	sc(255, 69, 0)
	dl(center_x, center_y, end_x, end_y)
end