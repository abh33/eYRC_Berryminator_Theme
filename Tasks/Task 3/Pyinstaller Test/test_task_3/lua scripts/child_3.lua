--[[
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This Lua script is to evaluate Task 3 of Berryminator (BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MOE project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
]]--

--Teams are NOT allowed to modify this script.


function sysCall_init()
    -- do some initialization here
    -- print('Child script init run')

    flag_whether_signal_received = 0   -- 0 not received, 1 received
    x_outer_grid = {-1, 9}
    y_outer_grid = {-1, 12}

    handles()
    get_target_points()

    score = 0   -- current score, max = 40 for four points
    point = 1  -- target point number, 1 means first navigational co-ordinate
    path = {{0,0}}   -- path taken by the bot during whole simulation
    
end


function handles()
    -- Getting handles

    dummy_handle  = sim.getObjectHandle('BM_Dummy_3')
    BM_Bot_handle = sim.getObjectHandle('BM_Bot')
    vs_handle     = sim.getObjectHandle('vision_sensor_1')
    
end


function get_target_points()

    s = (sim.getStringSignal("points"))
    -- print(s)

    if s ~= nil then

        flag_whether_signal_received = 1    -- Signal received 
        s = mysplit(s, "%%")

        target_points = {}

        for i = 1,4,1 do 
            table.insert(target_points, get_points(s[i]))
        end

        -- print(target_points)
        
        -- target_point_1 = get_points(s[1])   -- ex: target_point_1 = {44, 0}
        -- target_point_2 = get_points(s[2])
        -- target_point_3 = get_points(s[3])
        -- target_point_4 = get_points(s[4])
        
        -- print("Target navigational co-ordinates : ")
        -- print("target_point_1 = ")
        -- print(target_point_1)
        
        -- print("target_point_2 = ")
        -- print(target_point_2)
        
        -- print("target_point_3 = ")
        -- print(target_point_3)
        
        -- print("target_point_4 = ")
        -- print(target_point_4)

    else

        flag_whether_signal_received = 0   -- Signal NOT received
        target_points = {{99, 99}, {99, 99}, {99, 99}, {99, 99}}    -- These points are not in the grid

    end

end


function get_points(t)
    q = mysplit(t,"%,")
    --print()
    --print(q[1])
    --print(q[2])
    p1 = mysplit(q[1],"%(")
    p2 = mysplit(q[2],"%)")
    -- print("")
    --print(tonumber(p1[1]))
    --print(tonumber(p2[1]))
    p = {tonumber(p1[1]), tonumber(p2[1])}
    -- table.insert(p,tonumber(p1[1]))
    -- table.insert(p,tonumber(p2[1]))
    return p
end


function mysplit (inputstr, sep)
        if sep == nil then
                sep = "%s"
        end
        local t={}
        for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
                table.insert(t, str)
                --print(str)
        end
        return t
end


function sysCall_sensing()
    -- Put some sensing code here.

    position = sim.getObjectPosition(BM_Bot_handle,-1)

    -- Offsetting position to get position wrt qr codes
    for i=1,2 do
        position[i] = math.floor((position[i]+ 0.2665)/0.5330) 
    end
    -- print("Bot co-ordinate detected:")
    -- print(position)

    -- Recording any new co-ordinate and checking it against target co-ordinate
    last_index = #path
    if path[last_index][1] ~= position[1] or path[last_index][2] ~= position[2] then

        point_to_add = {position[1], position[2]}
        table.insert(path, point_to_add)

        if point <=4 then
            -- Checks for the four target co-ordinates, increases score
            if (target_points[point][1] == point_to_add[1] and target_points[point][2] == point_to_add[2]) then

                print("Reached co-ordinate: ")
                print(target_points[point])
                score = score + 10
                point = point + 1
            
            end
        end

        -- If bot goes out of the arena, throw a warning
        for i=1,2 do
            if point_to_add[1] == x_outer_grid[i] then
                print("[Warning] BM_Bot is out of the arena.")
            end
        end

        for i=1,2 do
            if point_to_add[2] == y_outer_grid[i] then
                print("[Warning] BM_Bot is out of the arena.")
            end
        end

    end

    -- Path recorded
    -- print(path)

end


function get_required_data_child( inInts, inFloats, inStrings, inBuffer)
    -- This function sends back eval data to python script

    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''
    
    s= {tostring(score), tostring(flag_whether_signal_received)}
    for i= 1, #path, 1 do
    --   print(path[i])
        temp = table.concat(path[i],",")
        table.insert(s, temp)
    end
    
    data_to_be_sent = s

    return inInts,inFloats,data_to_be_sent,inBuffer

end   