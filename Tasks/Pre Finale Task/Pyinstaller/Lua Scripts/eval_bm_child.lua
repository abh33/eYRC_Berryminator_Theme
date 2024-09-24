--[[
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This Lua script is to evaluate Task 5 of Berryminator (BM) Theme (eYRC 2021-22).
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

    -- Collection 1 - BM_Bot + arm + gripper
    local robotBase=sim.getObjectHandle('BM_Bot')
    robotCollection=sim.createCollection(0)
    sim.addItemToCollection(robotCollection,sim.handle_tree,robotBase,0)

    -- Collection 2 - gripper
    local gripperBase=sim.getObjectHandle('gripper')
    gripperCollection=sim.createCollection(0)
    sim.addItemToCollection(gripperCollection,sim.handle_tree,gripperBase,0)

    -- Collection 3 - table, pots, collection box + walls
    collection_3()

    -- Berries present in the scene  R1-R2-R3-R4
    berries_list = { 
        'lemon_1', 'lemon_2', 'blueberry_1', 'blueberry_2',  'strawberry_1', 'strawberry_2',
        'lemon_1#0', 'lemon_2#0', 'blueberry_1#0', 'blueberry_2#0',  'strawberry_1#0', 'strawberry_2#0',
        'lemon_1#2', 'lemon_2#2', 'blueberry_1#2', 'blueberry_2#2',  'strawberry_1#2', 'strawberry_2#2',
        'lemon_1#1', 'lemon_2#1', 'blueberry_1#1', 'blueberry_2#1',  'strawberry_1#1', 'strawberry_2#1',
    }

    -- Corresponding force sensors present in the scene
    fs_list = {
        'force_sensor_l1'  , 'force_sensor_l2'  ,'force_sensor_b1'   , 'force_sensor_b2'  , 'force_sensor_s1'  , 'force_sensor_s2', 
        'force_sensor_l1#0', 'force_sensor_l2#0', 'force_sensor_b1#0', 'force_sensor_b2#0', 'force_sensor_s1#0', 'force_sensor_s2#0', 
        'force_sensor_l1#2', 'force_sensor_l2#2', 'force_sensor_b1#2', 'force_sensor_b2#2', 'force_sensor_s1#2', 'force_sensor_s2#2', 
        'force_sensor_l1#1', 'force_sensor_l2#1', 'force_sensor_b1#1', 'force_sensor_b2#1', 'force_sensor_s1#1', 'force_sensor_s2#1', 
    }

    
    ci_list = {'CI'}
    plucked_list = {'Plucked'}
    dropped_list = {'Dropped'}
    collisions_list = {'Collisions'}   -- Collision of collection 1 with collecion 3 list
    pluck_check_index = 0              -- used to check true plucking
    no_of_collisions = 0               -- variable to confirm true plukcing. >10 -> true plucking
    mass_calc=0                        -- Mass of the entire robotic arm
    mass_calc_list={'Mass'}            
    detected_berry_name=''

    x_outer_grid = {-1, 9}             -- out of arena co-ordinates
    y_outer_grid = {-1, 12}            -- out of arena co-ordinates
    path = {{"Path"}, {0,0}}                     -- path taken by the bot during whole simulation

    dyn_check_flag = 1                 -- flag check so that dynamics of arm is checked only once
    reveal_bot_flag = 1           -- 1 denotes revealing of bot is still going on
    sim.setStringSignal('start_student_run','0')    -- signal to indicate python that robot revealing is complete, now run theme_implementation_primary()
    sim_time_taken_to_reveal = 0       -- time took to reveal the bot

    handles()                          -- getting handles
    rotate_plants()                    -- Rotating plants in child script. As we want the scene to go back to original state after simulation is stopped
    

    original_berry_position_wrt_world = {}      -- actual berries position wrt world
    student_position_data_list = {}             -- student's detected berries position wrt world (after transformation)
    get_berries_position_wrt_world()            -- Gets actual position of all the berries wrt world and stores it in original_berry_position_wrt_world


    -- Setting target velocity of camera revolute joint to 15 deg/s
    sim.setJointTargetVelocity( camera_revolute_joint_handle, - 0.261799)

end


-- function to generate collection no. 3 - pots, table, collection box and walls
function collection_3()

    -- Can't add these wall names to collection_3_names variable
    -- Because while generating the walls, names of some walls get changed
    -- Therefore have to follow the other way
    -- 'wall_0', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6', 'wall_7', 'wall_8', 'wall_9', 'wall_10', 'wall_11', 'wall_12', 'wall_13', 'wall_14', 'wall_15', 'wall_16', 'wall_17', 'wall_18', 'wall_19',  


    -- object names to be present in collection 3 except walls
    collection_3_names = { 
         
         'collection_box_1', 'cb_qr_3', 'collection_box_2', 'cb_qr_4',

         'vertical_rack_1', 'lemon_tree_pot',
         'vertical_rod_1', 'base_2', 'base_3',
         'blueberry_tree_pot', 
         'strawberry_tree_pot',

         'vertical_rack_1#0', 'lemon_tree_pot#0', 
         'vertical_rod_1#0', 'base_2#0', 'base_3#0',
         'blueberry_tree_pot#0',
         'strawberry_tree_pot#0',

         'vertical_rack_1#1', 'lemon_tree_pot#1', 
         'vertical_rod_1#1', 'base_2#1', 'base_3#1',
         'blueberry_tree_pot#1', 
         'strawberry_tree_pot#1', 

         'vertical_rack_1#2', 'lemon_tree_pot#2', 
         'vertical_rod_1#2', 'base_2#2', 'base_3#2',
         'blueberry_tree_pot#2', 
         'strawberry_tree_pot#2', 
        }

    local qr_plane_handle = sim.getObjectHandle('QR_Plane')
    local qr_plane_tree = sim.getObjectsInTree(qr_plane_handle,sim.handle_all, 0)

    table.remove(qr_plane_tree,1)     -- removing handle of QR Plane
    local walls_handle = qr_plane_tree   -- table of handle of all the walls

    -- local vertical_rack_1 = sim.getObjectHandle('vertical_rack_1')
    -- local vertical_rod_1 = sim.getObjectHandle('vertical_rod_1')
    -- local base_2 = sim.getObjectHandle('base_2')
    -- local base_3 = sim.getObjectHandle('base_3')

    -- local lemon_tree_pot = sim.getObjectHandle('lemon_tree_pot')
    -- local blueberry_tree_pot = sim.getObjectHandle('blueberry_tree_pot')
    -- local strawberry_tree_pot = sim.getObjectHandle('strawberry_tree_pot')

    -- local collection_box_1 = sim.getObjectHandle('collection_box_1')
    -- local cb_qr_1 = sim.getObjectHandle('cb_qr_1')

    -- Creating the collection no. 3
    collection_3=sim.createCollection(0)

    for i=1, #collection_3_names, 1 do

        local handle = sim.getObjectHandle(collection_3_names[i])
        sim.addItemToCollection(collection_3,sim.handle_single, handle,0)

    end

    for i=1, #walls_handle, 1 do

        sim.addItemToCollection(collection_3,sim.handle_single, walls_handle[i],0)

    end

    -- sim.addItemToCollection(collection_3,sim.handle_single,vertical_rack_1,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,vertical_rod_1,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,base_2,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,base_3,0)

    -- sim.addItemToCollection(collection_3,sim.handle_single,lemon_tree_pot,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,blueberry_tree_pot,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,strawberry_tree_pot,0)

    -- sim.addItemToCollection(collection_3,sim.handle_single,collection_box_1,0)
    -- sim.addItemToCollection(collection_3,sim.handle_single,cb_qr_1,0)

end


-- Gets actual position of all the berries wrt world
function get_berries_position_wrt_world()

    for i=1, #berries_list, 1 do

        local handle = sim.getObjectHandle(berries_list[i])
        local berry_position = sim.getObjectPosition(handle, -1)

        -- original_berry_position_wrt_world[berries_list[i]] = berry_position
        table.insert(original_berry_position_wrt_world, berry_position)

    end
    
    -- print(original_berry_position_wrt_world)
end


function handles()
    -- Getting handles

    -- dummy_handle  = sim.getObjectHandle('eval_bm')
    BM_Bot_handle = sim.getObjectHandle('BM_Bot')
    camera_handle = sim.getObjectHandle('DefaultCamera')
    vs_2_handle     = sim.getObjectHandle('vision_sensor_2')
    force_sensor_br= sim.getObjectHandle('force_sensor_br')
    camera_revolute_joint_handle = sim.getObjectHandle("rj_for_camera")
    support_cuboid_handle = sim.getObjectHandle("cuboid_for_camera")
    -- vs_1_handle     = sim.getObjectHandle('vision_sensor_1')
    
end


-- Function to shuffle a list
function shuffle(list)
	
    for i = #list, 2, -1 do
		local j = math.random(i)
		list[i], list[j] = list[j], list[i]
	end

	return list
end


function rotate_plants()
    -- This will randomly change the orientation of plants in all four rooms
    -- 0, +- 30 deg can be the possible change

    -- Pots list
    local pots_list = { 
        'lemon_tree_pot', 'blueberry_tree_pot', 'strawberry_tree_pot',
        'lemon_tree_pot#0', 'blueberry_tree_pot#0', 'strawberry_tree_pot#0',
        'lemon_tree_pot#2', 'blueberry_tree_pot#2', 'strawberry_tree_pot#2',
        'lemon_tree_pot#1', 'blueberry_tree_pot#1', 'strawberry_tree_pot#1',
    }

    -- Possible change of angles
    angles_choice = {0, 0.523599, -0.523599}         -- 0.523599 rad = 30 deg

    for i=1,4,1 do   -- total 4 rooms

        -- Shuffling the list of angles
        local shuffled_angles_choice = shuffle(angles_choice)

        -- One by one applying the angle change to individual pots in a single room
        local start_index = (i-1)*3 + 1
        local end_index   = start_index + 2

        local shuffled_angles_choice_index = 1
        for i=start_index, end_index, 1 do
            local pot_handle = sim.getObjectHandle(pots_list[i])
            local pot_orientation = sim.getObjectOrientation(pot_handle, -1)

            pot_orientation[3] = pot_orientation[3] + shuffled_angles_choice[shuffled_angles_choice_index]

            sim.setObjectOrientation( pot_handle, -1, pot_orientation)

            shuffled_angles_choice_index = shuffled_angles_choice_index + 1
        end

    end

    -- sim.setObjectOrientation(lemon_tree_pot     ,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})
    -- sim.setObjectOrientation(blueberry_tree_pot ,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})
    
    -- -- Because of the position of the collection box, we can not rotate strawberry_tree_pot. 
    -- --sim.setObjectOrientation(strawberry_tree_pot,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})

    -- sim.setObjectOrientation(strawberry_tree_pot,-1,{0,0,0})

end


function sysCall_actuation()
    -- put your actuation code here

    -- Reveal the bot
    -- i.e. rotate around BM_Bot for a 360 deg turn then stop
    if reveal_bot_flag==1 then
        reveal_bot()
    end
    
    -- if atleast one berry is identified then go for pluck check for the latest detected berry
    if #ci_list > 1 then
        pluck_check()
    end

    -- check dynamics of arm when 1 sec of simulation time is exceeded -- only once
    if (dyn_check_flag==1) then
        check_simulation_time()
    end

end

function reveal_bot()

    local position = sim.getJointPosition( camera_revolute_joint_handle)

    if (position > 0) and (position < 0.523599) then           -- Between 0 and 30 deg
        
        -- Setting target velocity of camera revolute joint to 0 deg/s
        sim.setJointTargetVelocity( camera_revolute_joint_handle, 0)

        -- Reset DefaultCamera position  
        local start_pose = {0.9593454599, -1.217581272, -1.503504276, 0.1416777372, 0.3513730466, 0.6136090755, -0.6927829981}
        sim.setObjectPose(camera_handle, BM_Bot_handle, start_pose)

        sim.removeObject(support_cuboid_handle)
        sim.removeObject(camera_revolute_joint_handle)

        reveal_bot_flag = 0
        sim.setStringSignal('start_student_run','1')

        local simulation_time = (sim.getStringSignal( 'time'))

        if simulation_time == nil then
            sim_time_taken_to_reveal = '0'
        else
            sim_time_taken_to_reveal = simulation_time       -- keeping it in string only
        end

    end

end


function check_simulation_time()

    local simulation_time = (sim.getStringSignal( 'time'))

    if simulation_time == nil then
        simulation_time = 0
    else
        simulation_time = tonumber(simulation_time)
    end

    if simulation_time > 1 then

        -- Checking dynamics of robotic arm after 1 simulation sec, only once.
        dynamics_and_mass_check()

        dyn_check_flag = 0

    end

end


function sysCall_sensing()
    -- Put some sensing code here.

    -----------------------------------------------------------------------------------------------------
    -- Storing path taken by BM_Bot

    position = sim.getObjectPosition(BM_Bot_handle,-1)

    -- Offsetting position to get position wrt qr codes
    for i=1,2 do
        position[i] = math.floor((position[i]+ 0.2665)/0.5330) 
    end
    -- print("Bot co-ordinate detected:")
    -- print(position)

    -- Recording any new co-ordinate
    last_index = #path
    if path[last_index][1] ~= position[1] or path[last_index][2] ~= position[2] then

        point_to_add = {position[1], position[2]}
        table.insert(path, point_to_add)

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

    -----------------------------------------------------------------------------------------------------


    -----------------------------------------------------------------------------------------------------
    -- Storing any collisions between collection 1 and collection 3

    local result,pairHandles=sim.checkCollision(robotCollection,collection_3)
    if result>0 then
        -- print('Robot is colliding. Colliding pair is '..sim.getObjectName(pairHandles[1]).." "..sim.getObjectName(pairHandles[2]))
        local pairString = (sim.getObjectName(pairHandles[1]).." and "..sim.getObjectName(pairHandles[2]))
        if collisions_list[#collisions_list] ~= pairString then
            table.insert(collisions_list, pairString)
            print('Robot is colliding. Colliding pair is '..pairString)
        end
    end



end



-- Function which will be called by students. They will send x,y,z of detected berry wrt VS 2
-- We will transform this x,y,z wrt Vision Sensor (VS) 2 to x,y,z wrt world
-- We will keep appending this to student_position_data_list
-- ci_check will confirm whether a berry exists or not at that position
function detected_berry_by_team(inInts, inFloats, inStrings, inBuffer)

    detected_berry_name=tostring(inStrings[1])
    
    student_berry_position = {tonumber(inStrings[2]), tonumber(inStrings[3]), tonumber(inStrings[4])}
    eulerAngles = {0, 0, 0}

    berry_matrix_wrt_vs_2 = sim.buildMatrix( student_berry_position, eulerAngles)
    vs_2_matrix_wrt_world = sim.getObjectMatrix( vs_2_handle, -1)

    berry_matrix_wrt_world = sim.multiplyMatrices( vs_2_matrix_wrt_world, berry_matrix_wrt_vs_2)
    student_berry_position_wrt_world = {berry_matrix_wrt_world[4], berry_matrix_wrt_world[8], berry_matrix_wrt_world[12]}

    -- print("Student's Berry Position wrt world :")
    -- print(student_berry_position_wrt_world)

    table.insert(student_position_data_list, student_berry_position_wrt_world)

    ci_check()

    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''

    return inInts,inFloats,inStrings,inBuffer

end


-- If a berry exists at the position detected by the student then append that berry name to CI list
-- since it is a TRUE detection
-- If a berry doesn't exist, we don't append.
function ci_check()

    -- for s_index,s in pairs(student_position_data_list) do
    s_index = #student_position_data_list
    s = student_position_data_list[s_index]

        --print(s)
        for o_index,o in pairs(original_berry_position_wrt_world) do
            x_error = math.abs(o[1] - s[1])
            y_error = math.abs(o[2] - s[2])
            z_error = math.abs(o[3] - s[3])
            
            if x_error <=0.025 and y_error <=0.025 and z_error <=0.025 then
                
                no_of_collisions = 0
                
                detected_berry_name=string.lower(detected_berry_name)
                if(string.find(berries_list[o_index],detected_berry_name)~=nil) then
                    table.insert(ci_list, berries_list[o_index])
                    pluck_check_index = o_index
                    print('Berry Detected by Team- ',detected_berry_name)
                end

                -- temp = o_index / 4
                
                -- if temp <= 1 then
                --     table.insert(ci_list, 'Lemon')
                
                -- elseif temp <= 2 then
                --     table.insert(ci_list, 'Blueberry')

                -- elseif temp <=3 then
                --     table.insert(ci_list, 'Strawberry')
                
                -- end
                
            end
            
        end

    -- end


end


-- This function will check for plucking for the latest detected berry
-- First the force sensor should be broken 
-- Second, the collision between gripper and that latest detected berry should happen for more than 10 times
-- This will constitute a True Plucking and that berry will be appended in pluck list
function pluck_check()


    -- for i=1, #fs_list, 1 do

        fs_handle = sim.getObjectHandle(fs_list[pluck_check_index])
        result, forceVector, torqueVector = sim.readForceSensor(fs_handle)
        if result == 3 then

            local berry_handle = sim.getObjectHandle(berries_list[pluck_check_index])
            local result,pairHandles=sim.checkCollision(gripperCollection,berry_handle)
            if result>0 then
                no_of_collisions = no_of_collisions + 1
                -- print('Robot is colliding. Colliding pair is '..sim.getObjectName(pairHandles[1]).." "..sim.getObjectName(pairHandles[2]))
                if no_of_collisions > 10 then
                    
                    if plucked_list[#plucked_list] ~= berries_list[pluck_check_index] then
                        table.insert(plucked_list, berries_list[pluck_check_index])
                    end

                end
            end
            
            -- temp = i / 4

            -- if temp <= 1 then
            --     table.insert(plucked_list, 'Lemon')
            
            -- elseif temp <= 2 then
            --     table.insert(plucked_list, 'Blueberry')

            -- elseif temp <=3 then
            --     table.insert(plucked_list, 'Strawberry')
            
            -- end

        end

    -- end

end



-- This function will check the positions of all the berries
-- If found inside the collection box, will add to dropped list
function drop_check()

    -- First checking CB1
    table.insert(dropped_list, "CB1")

    for i=1, #berries_list, 1 do
        berry_handle   = sim.getObjectHandle(berries_list[i])
        berry_position = sim.getObjectPosition(berry_handle, -1)

        if ( 0 <= berry_position[3] and berry_position[3] <= 0.0535 ) then
            if ( 5.782 <= berry_position[2] and berry_position[2] <= 5.933 ) then
                if ( 0.407 <= berry_position[1] and berry_position[1] <= 0.67 ) then
                    -- if comes here that means berry is inside collection box 1
                    table.insert(dropped_list, berries_list[i])

                end    
            end

        end 
    
    end


    -- Now checking CB2
    table.insert(dropped_list, "CB2")

    for i=1, #berries_list, 1 do
        berry_handle   = sim.getObjectHandle(berries_list[i])
        berry_position = sim.getObjectPosition(berry_handle, -1)

        if ( 0 <= berry_position[3] and berry_position[3] <= 0.0535 ) then
            if ( 5.782 <= berry_position[2] and berry_position[2] <= 5.933 ) then
                if ( 3.609 <= berry_position[1] and berry_position[1] <= 3.873 ) then
                    -- if comes here that means berry is inside collection box 1
                    table.insert(dropped_list, berries_list[i])

                end    
            end

        end 
    
    end

end


-- Called by python after their theme_implementation_primary function has ended
function get_required_data_child( inInts, inFloats, inStrings, inBuffer)
    -- This function sends back eval data to python script

    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''
    
    -- ci_check()
    -- pluck_check()

    -- Drop check is done at the last
    drop_check()

    -- s= {tostring(score), tostring(flag_whether_signal_received)}
    s = {table.concat(ci_list,","), table.concat(plucked_list,","), table.concat(dropped_list,","), 
            table.concat(collisions_list,","), table.concat(eval_all_dynamics_list,","),
                table.concat(mass_calc_list,",")}


    for i= 1, #path, 1 do
    --   print(path[i])
        temp = table.concat(path[i],",")
        table.insert(s, temp)
    end

    temp = table.concat(dynamics_not_enabled_list, ",")
    table.insert(s, temp)


    -- Getting total simulation time
    local end_simulation_time = (sim.getStringSignal( 'time'))

    if end_simulation_time == nil then
        end_simulation_time = '0'
    end
    temp = {'EndSimTime', end_simulation_time}
    temp = table.concat( temp, ",")
    table.insert(s, temp)


    temp = {'SimTimeToReveal', sim_time_taken_to_reveal}
    temp = table.concat( temp, ",")
    table.insert(s, temp)

    
    data_to_be_sent = s
    -- print(data_to_be_sent)

    return inInts,inFloats,data_to_be_sent,inBuffer

end   


function sysCall_cleanup()
    -- do some clean-up here

    -- -- Checking dynamics of robotic arm before simulation ends
    -- dynamics_and_mass_check()


end


function dynamics_and_mass_check()
    -- This function will check the dynamics and mass

    arm_handle       = sim.getObjectHandle('robotic_arm')
    robot = arm_handle
    gripper_handle   = sim.getObjectHandle('gripper')

    -- Get all objects inside robotic arm ... which also includes gripper now
    objects = sim.getObjectsInTree(arm_handle,sim.handle_all, 0)
    -- print("Total Objects tree: ")
    -- print(objects)

    gripper_tree = sim.getObjectsInTree(gripper_handle,sim.handle_all, 0)


    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    -- Removing gripper objects to make a tree of only robotic arm

    objects_temp = objects
    objects = {}
    -- Iterating whole arm and deleting gripper tree objects
    for i, obj_handle in ipairs(objects_temp) do
        skip = false

        for _,v in pairs(gripper_tree) do
            if v == obj_handle then
              skip = true
            end
        end

        if not skip then
            table.insert(objects, obj_handle)
            -- print(sim.getObjectName(obj_handle))
        end

    end


    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --    
    -- CHECK: whether all shapes, joints and force sensor are dynamically enabled or not
    
    eval_all_dynamics_list = {'Eval_Dyn'}
    eval_all_dynamics = 1
    dynamics_not_enabled_list = {"Dyn"}    -- Storing object names who are dynamically not enabled 
    -- Iterating arm except gripper
    for i, obj_handle in ipairs(objects) do
        -- print(sim.getObjectType(objects[i]))
        current_object_type = sim.getObjectType(objects[i])
        
        -- If current object is joint or force sensor type or shape type
        if  current_object_type== sim.object_joint_type or current_object_type == sim.object_forcesensor_type or current_object_type ==  sim.object_shape_type then
            if sim.isDynamicallyEnabled(objects[i]) then
                -- Nothing to print, PASS
                -- print(' ')
            else
                -- check is it visible shape, if yes exclude
                if string.find(sim.getObjectName(objects[i]), "visible") then
                    -- print ("Avoided visible ")
                    -- local tempp = 9999
                else
                    print("")
                    print("[ERROR] "..sim.getObjectName(objects[i])..' is not dynamically enabled')
                    print("Wait till simulation ends. Make it dynamically enabled.")
                    eval_all_dynamics = 0
                    table.insert( dynamics_not_enabled_list, sim.getObjectName(objects[i]))
                end
            end
        end
    end
    -- sim.setIntegerSignal('eval_all_dynamics', eval_all_dynamics)
    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    -- print(eval_all_dynamics)
    if eval_all_dynamics == 1 then
        print("All shapes, joints and force sensors are dynamically enabled")
        
    end
    
     result,forceVector,torqueVector = sim.readForceSensor(force_sensor_br)
    -- r = sim.simxReadForceSensor(client_id, handle, sim.simx_opmode_blocking)
    mass_calc = -forceVector[3]/9.81
    print('Mass                                       : '..mass_calc.." kg")
    table.insert(mass_calc_list,tostring(mass_calc))
    table.insert(eval_all_dynamics_list, eval_all_dynamics)
    
end