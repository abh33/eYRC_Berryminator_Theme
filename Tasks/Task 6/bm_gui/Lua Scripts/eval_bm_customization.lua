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
    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''

    init_system_time=sim.getSystemTime()

    sim.setStringSignal('gfh36801nc','0')
    flag_error = 0

    if_added = false                      -- flag 

    if(pcall(handles))then
        
        if(pcall(organize_screen_init))then
            print("Evaluation started successfully.")
            print("")
            -- print('Getting handles complete.')
            r = all_objects_check()
            if r then

                if (pcall(set_all_objects_special_property)) then

                    if (pcall(set_respondable_property)) then
                    
                        r = arm_check()
                        -- rotate_plants()
                        set_camera_before_run()

                        if r then
                            sim.setStringSignal('gfh36801nc','1')    -- indicating to python that scene check is complete
                            -- After this python will replace the BM_Bot and start running their task_4_primary() function
                        else
                            flag_error = 1                                                                                                                                                                                                                                    
                        end

                    else
                        print("[ERROR] Couldn't set respondable property of objects.")
                        flag_error=1
                    end

                else
                    print("[ERROR] Couldn't set special property of objects.")
                    flag_error=1
                end

            else

                flag_error = 1

            end

        else
            print("[ERROR] Re-download exe file")
        end

    else
        print("[ERROR] Couldn't find expected objects in the scene")
        flag_error=1
    end


    if(flag_error == 1)then

        sim.setStringSignal('gfh36801nc','0')
        end_program()
        print('[ERROR] Evaluation can not proceed further.')

    end


end


function handles()
    -- Getting handles
    -- self handle
    dummy_handle  = sim.getObjectHandle('eval_bm')
    -- camera handle
    camera_handle    = sim.getObjectHandle('DefaultCamera')
    BM_Bot_handle    = sim.getObjectHandle('BM_Bot')
    arm_handle       = sim.getObjectHandle('robotic_arm')
    force_sensor_br_handle = sim.getObjectHandle('force_sensor_br')
    vs_1_handle      = sim.getObjectHandle('vision_sensor_1')
    vs_2_handle      = sim.getObjectHandle('vision_sensor_2')
    nameplate_handle = sim.getObjectHandle('name_plate')
    gripper_handle   = sim.getObjectHandle('gripper')

    local gripperBase=sim.getObjectHandle('gripper')

    -- 'wall_0', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6', 'wall_7', 'wall_8', 'wall_9', 'wall_10', 'wall_11', 'wall_12', 'wall_13', 'wall_14', 'wall_15', 'wall_16', 'wall_17', 'wall_18', 'wall_19',  

    -- Default object names
    names = {'DefaultCamera', 'floor', 'warning_strips', 'QR_Plane', 
          'DefaultLights', 'DefaultLightA', 'DefaultLightB', 'DefaultLightC', 'DefaultLightD',
         'XYZCameraProxy', 'DefaultNXViewCamera', 'DefaultNYViewCamera', 'DefaultNZViewCamera', 'DefaultXViewCamera', 'DefaultYViewCamera', 'DefaultZViewCamera',
         'depositing_bay_1', 'depositing_bay_2', 'greenhouse_walls', 'home_charging_station',
         'collection_box_1', 'cb_qr_3', 'collection_box_2', 'cb_qr_4',

         'vertical_rack_1', 'lemon_tree_pot', 'lemon_tree_branch', 'lemon_tree_leaves',
         'force_sensor_l1', 'force_sensor_l2', 'lemon_1', 'lemon_2',
         'vertical_rod_1', 'base_2', 'base_3',
         'blueberry_tree_pot', 'blueberry_tree_branch', 'blueberry_tree_leaves',
         'force_sensor_b1', 'force_sensor_b2', 'blueberry_1', 'blueberry_2', 
         'strawberry_tree_pot', 'strawberry_tree_branch', 'strawberry_tree_leaves',
         'force_sensor_s1', 'force_sensor_s2', 'strawberry_1', 'strawberry_2',

         'vertical_rack_1#0', 'lemon_tree_pot#0', 'lemon_tree_branch#0', 'lemon_tree_leaves#0',
         'force_sensor_l1#0', 'force_sensor_l2#0', 'lemon_1#0', 'lemon_2#0',
         'vertical_rod_1#0', 'base_2#0', 'base_3#0',
         'blueberry_tree_pot#0', 'blueberry_tree_branch#0', 'blueberry_tree_leaves#0',
         'force_sensor_b1#0', 'force_sensor_b2#0', 'blueberry_1#0', 'blueberry_2#0', 
         'strawberry_tree_pot#0', 'strawberry_tree_branch#0', 'strawberry_tree_leaves#0',
         'force_sensor_s1#0', 'force_sensor_s2#0', 'strawberry_1#0', 'strawberry_2#0',

         'vertical_rack_1#1', 'lemon_tree_pot#1', 'lemon_tree_branch#1', 'lemon_tree_leaves#1',
         'force_sensor_l1#1', 'force_sensor_l2#1', 'lemon_1#1', 'lemon_2#1',
         'vertical_rod_1#1', 'base_2#1', 'base_3#1',
         'blueberry_tree_pot#1', 'blueberry_tree_branch#1', 'blueberry_tree_leaves#1',
         'force_sensor_b1#1', 'force_sensor_b2#1', 'blueberry_1#1', 'blueberry_2#1', 
         'strawberry_tree_pot#1', 'strawberry_tree_branch#1', 'strawberry_tree_leaves#1',
         'force_sensor_s1#1', 'force_sensor_s2#1', 'strawberry_1#1', 'strawberry_2#1',

         'vertical_rack_1#2', 'lemon_tree_pot#2', 'lemon_tree_branch#2', 'lemon_tree_leaves#2',
         'force_sensor_l1#2', 'force_sensor_l2#2', 'lemon_1#2', 'lemon_2#2',
         'vertical_rod_1#2', 'base_2#2', 'base_3#2',
         'blueberry_tree_pot#2', 'blueberry_tree_branch#2', 'blueberry_tree_leaves#2',
         'force_sensor_b1#2', 'force_sensor_b2#2', 'blueberry_1#2', 'blueberry_2#2', 
         'strawberry_tree_pot#2', 'strawberry_tree_branch#2', 'strawberry_tree_leaves#2',
         'force_sensor_s1#2', 'force_sensor_s2#2', 'strawberry_1#2', 'strawberry_2#2',

         'eval_bm', 'target'}


    -- Adding object names of the walls generated to names variable
    local qr_plane_handle = sim.getObjectHandle('QR_Plane')
    local qr_plane_tree = sim.getObjectsInTree(qr_plane_handle,sim.handle_all, 0)

    table.remove(qr_plane_tree,1)     -- removing handle of QR Plane
    local walls_handle = qr_plane_tree   -- table of handle of all the walls

    for i=1, #walls_handle, 1 do

        local wall_name = sim.getObjectName( walls_handle[i])
        table.insert(names, wall_name)
    end


    -- Getting handles of all the objects in names variable
    for p,q in ipairs(names) do
        local handle = sim.getObjectHandle(q)
        if handle == -1 then
            print(q)
        end
    end

    -- local vertical_rack_1 = sim.getObjectHandle('vertical_rack_1')
    -- local vertical_rod_1 = sim.getObjectHandle('vertical_rod_1')
    -- local base_2 = sim.getObjectHandle('base_2')
    -- local base_3 = sim.getObjectHandle('base_3')

    -- lemon_tree_pot = sim.getObjectHandle('lemon_tree_pot')
    -- blueberry_tree_pot = sim.getObjectHandle('blueberry_tree_pot')
    -- strawberry_tree_pot = sim.getObjectHandle('strawberry_tree_pot')

    -- local collection_box_1 = sim.getObjectHandle('collection_box_1')
    -- local cb_qr_1 = sim.getObjectHandle('cb_qr_3')
    -- local collection_box_2 = sim.getObjectHandle('collection_box_2')
    -- local cb_qr_2 = sim.getObjectHandle('cb_qr_4')

    -- local qr_plane_handle = sim.getObjectHandle("QR_Plane")
    -- local floor_handle = sim.getObjectHandle('Floor')
    
end


function set_all_objects_special_property()

    local all_handles = sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0)
    local total_objects_count = #all_handles
    -- local index = 1

    for index=1, total_objects_count, 1
    do
        sim.setObjectSpecialProperty(all_handles[index],1)    -- turning collidable property ON for all objects in scene

    end

    -- Berries present in the scene  R1-R2-R3-R4
    local berries_list = { 
        'lemon_1', 'lemon_2', 'blueberry_1', 'blueberry_2',  'strawberry_1', 'strawberry_2',
        'lemon_1#0', 'lemon_2#0', 'blueberry_1#0', 'blueberry_2#0',  'strawberry_1#0', 'strawberry_2#0',
        'lemon_1#2', 'lemon_2#2', 'blueberry_1#2', 'blueberry_2#2',  'strawberry_1#2', 'strawberry_2#2',
        'lemon_1#1', 'lemon_2#1', 'blueberry_1#1', 'blueberry_2#1',  'strawberry_1#1', 'strawberry_2#1',
    }

    -- local walls_list = {
    --     'wall_0', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6', 'wall_7', 'wall_8', 'wall_9', 'wall_10', 'wall_11', 'wall_12', 'wall_13', 'wall_14', 'wall_15', 'wall_16', 'wall_17', 'wall_18', 'wall_19'
    -- }

    local qr_plane_handle = sim.getObjectHandle("QR_Plane")
    sim.setObjectSpecialProperty(qr_plane_handle, 512)       -- QR Plane - only renderable


    -- local index = 1
    for index=1, #berries_list, 1
    do
        local berry_handle = sim.getObjectHandle(berries_list[index])
        sim.setObjectSpecialProperty( berry_handle, 513)    -- All berries - collidable and renderable

    end

    local floor_handle = sim.getObjectHandle('floor')
    sim.setObjectSpecialProperty(floor_handle, 0)       -- Floor - nothing ON

    sim.setObjectSpecialProperty(dummy_handle, 0)       -- Dummy - nothing ON

end


function set_respondable_property()

    local collection_3_names = { 
         
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

    for index=1, #collection_3_names, 1
    do
        local handle = sim.getObjectHandle(collection_3_names[index])
        sim.setObjectInt32Parameter( handle, sim.shapeintparam_respondable, 1)    -- Making repondable

    end

end


function all_objects_check()
    -- This function will detect extra objects added in the scene other than the default, except in BM_Bot

    r = true

    all_objects_in_scene = sim.getObjectsInTree( sim.handle_scene, sim.handle_all, 0)
    -- print(all_objects_in_scene)
    -- Get all objects inside BM_Bot_handle
    objects = sim.getObjectsInTree(BM_Bot_handle,sim.handle_all, 0)
    -- print(objects)

    -- -- Default object names
    -- names = {'DefaultCamera', 'Floor', 'warning_strips', 'QR_Plane', 
    --      'wall_0', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6', 'wall_7', 'wall_8', 'wall_9', 'wall_10', 'wall_11', 'wall_12', 'wall_13', 'wall_14', 'wall_15', 'wall_16', 'wall_17', 'wall_18', 'wall_19',  
    --      'DefaultLights', 'DefaultLightA', 'DefaultLightB', 'DefaultLightC', 'DefaultLightD',
    --      'XYZCameraProxy', 'DefaultNXViewCamera', 'DefaultNYViewCamera', 'DefaultNZViewCamera', 'DefaultXViewCamera', 'DefaultYViewCamera', 'DefaultZViewCamera',
    --      'depositing_bay_1', 'depositing_bay_2', 'greenhouse_walls', 'home_charging_station',
    --      'collection_box_1', 'cb_qr_3', 'collection_box_2', 'cb_qr_4',

    --      'vertical_rack_1', 'lemon_tree_pot', 'lemon_tree_branch', 'lemon_tree_leaves',
    --      'force_sensor_l1', 'force_sensor_l2', 'lemon_1', 'lemon_2',
    --      'vertical_rod_1', 'base_2', 'base_3',
    --      'blueberry_tree_pot', 'blueberry_tree_branch', 'blueberry_tree_leaves',
    --      'force_sensor_b1', 'force_sensor_b2', 'blueberry_1', 'blueberry_2', 
    --      'strawberry_tree_pot', 'strawberry_tree_branch', 'strawberry_tree_leaves',
    --      'force_sensor_s1', 'force_sensor_s2', 'strawberry_1', 'strawberry_2',

    --      'vertical_rack_1#0', 'lemon_tree_pot#0', 'lemon_tree_branch#0', 'lemon_tree_leaves#0',
    --      'force_sensor_l1#0', 'force_sensor_l2#0', 'lemon_1#0', 'lemon_2#0',
    --      'vertical_rod_1#0', 'base_2#0', 'base_3#0',
    --      'blueberry_tree_pot#0', 'blueberry_tree_branch#0', 'blueberry_tree_leaves#0',
    --      'force_sensor_b1#0', 'force_sensor_b2#0', 'blueberry_1#0', 'blueberry_2#0', 
    --      'strawberry_tree_pot#0', 'strawberry_tree_branch#0', 'strawberry_tree_leaves#0',
    --      'force_sensor_s1#0', 'force_sensor_s2#0', 'strawberry_1#0', 'strawberry_2#0',

    --      'vertical_rack_1#1', 'lemon_tree_pot#1', 'lemon_tree_branch#1', 'lemon_tree_leaves#1',
    --      'force_sensor_l1#1', 'force_sensor_l2#1', 'lemon_1#1', 'lemon_2#1',
    --      'vertical_rod_1#1', 'base_2#1', 'base_3#1',
    --      'blueberry_tree_pot#1', 'blueberry_tree_branch#1', 'blueberry_tree_leaves#1',
    --      'force_sensor_b1#1', 'force_sensor_b2#1', 'blueberry_1#1', 'blueberry_2#1', 
    --      'strawberry_tree_pot#1', 'strawberry_tree_branch#1', 'strawberry_tree_leaves#1',
    --      'force_sensor_s1#1', 'force_sensor_s2#1', 'strawberry_1#1', 'strawberry_2#1',

    --      'vertical_rack_1#2', 'lemon_tree_pot#2', 'lemon_tree_branch#2', 'lemon_tree_leaves#2',
    --      'force_sensor_l1#2', 'force_sensor_l2#2', 'lemon_1#2', 'lemon_2#2',
    --      'vertical_rod_1#2', 'base_2#2', 'base_3#2',
    --      'blueberry_tree_pot#2', 'blueberry_tree_branch#2', 'blueberry_tree_leaves#2',
    --      'force_sensor_b1#2', 'force_sensor_b2#2', 'blueberry_1#2', 'blueberry_2#2', 
    --      'strawberry_tree_pot#2', 'strawberry_tree_branch#2', 'strawberry_tree_leaves#2',
    --      'force_sensor_s1#2', 'force_sensor_s2#2', 'strawberry_1#2', 'strawberry_2#2',

    --      'eval_bm', 'target'}



    for i, obj_handle in ipairs(all_objects_in_scene) do


        -- Noting where BM_Bot objects are present
        -- obj_name = sim.getObjectName(obj_handle)
        for j, bot_obj_handle in ipairs(objects) do
            -- print(obj_handle)
            -- print(arm_obj_handle)
            if obj_handle == bot_obj_handle then
                all_objects_in_scene[i] = "NA"
            end    
        end


    end
    -- print(all_objects_in_scene)


    -- Detecting extra objects in the scene, except ours
    for i, obj_handle in ipairs(all_objects_in_scene) do

        if obj_handle ~= "NA" then

            has_allowed_object = false
            obj_name = sim.getObjectName(obj_handle)

            for j, allowed_names in ipairs(names) do
                if obj_name ==  allowed_names then
                        has_allowed_object = true
                end
            end

            if has_allowed_object == false then
                print("")
                print("[ERROR] Found extra object in the scene: "..obj_name)
                print("Remove it and retry")
                r = false
                return r
            
            -- Since teams are going to use their own gripper and IK script, we can not
            -- disable scripts from now on.
            
            -- Code to disable student scripts if ANY.
            --[[
            else
                 if sim.getObjectName(all_objects_in_scene[i]) ~= 'eval_bm' then
                      Deactivating their scripts for each object if any
                     student_child_script_handle = sim.getScriptAssociatedWithObject(obj_handle)
                     student_cutomization_script_handle = sim.getCustomizationScriptAssociatedWithObject(obj_handle)
                     if (student_child_script_handle ~= -1) then
                         sim.setScriptAttribute( student_child_script_handle, sim.scriptattribute_enabled, false)
                         print("child done")
                     elseif (student_cutomization_script_handle ~= -1) then
                         sim.setScriptAttribute( student_cutomization_script_handle, sim.scriptattribute_enabled, false)
                         print("custom done")
                     end
                 end
            --]]
            end
        end

    end

    -- -- Setting DefaultCamera's position and orientation
    -- sim.setObjectPosition( camera_handle, -1, {2.81, 3.1, 14.678})
    -- sim.setObjectOrientation( camera_handle, -1,{ -3.14, 0, 3.14})

    -- Checking whether child of fs_br is robotic_arm or not
    child_handle = sim.getObjectChild(force_sensor_br_handle, 0)
    if child_handle ~= arm_handle then
        print("")
        print("[ERROR] robotic_arm is not a child of force_sensor_br.")
        -- print("Remove it and retry")
        r = false
        return r
    end

    return r

end


function arm_check()
    -- This function will check model definition, respondable, dynamic properties, set density
    -- check no of joints, calculate total torque and force required, individual values

    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''
    r = true

    robot = arm_handle
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
    -- CHECK: checking model definition, respondable, dynamic properties

    -- Iterating whole arm
    for i, obj_handle in ipairs(objects) do


        -- checking model definition properties
        property = sim.getModelProperty(objects[i])
        if sim.getObjectName(objects[i]) == "robotic_arm" then
            
            if property ~= 0 then  -- 0 corresponds to -> object is model base (ticked), all inside options (unticked)

                print("")
                print("[ERROR] Model definition is incorrect for robotic_arm")
                r = false
                return r    

            end

        else
            if property ~= 61440 then   -- 61440 corresponds to -> object is model base (unticked)   

                print("")
                print("[ERROR] Model definition is incorrect for "..sim.getObjectName(objects[i]))
                r = false
                return r    

            end
        end    



        -- checking respondable, dynamic properties of Shapes
        result, parameter_static = sim.getObjectInt32Parameter( objects[i], sim.shapeintparam_static)
        result, parameter_respondable = sim.getObjectInt32Parameter( objects[i], sim.shapeintparam_respondable)
        if (string.find(sim.getObjectName(objects[i]), "dyn") or sim.getObjectName(objects[i]) == "robotic_arm") and (sim.getObjectType(objects[i]) == sim.object_shape_type) then
            
            if parameter_static ~= 0 then    -- 1 corresponds to static shape
                print("")
                print("[ERROR] This dynamic body is currently static : "..sim.getObjectName(objects[i]))
                r = false
                return r
            elseif parameter_respondable ~= 1 then   -- 1 corresponds to respondable
                print("")
                print("[ERROR] This dynamic body is currently non respondable : "..sim.getObjectName(objects[i]))
                r = false
                return r
            end

            -- Setting uniform density to 700
            result = sim.computeMassAndInertia( objects[i], 700) 
            
            -- It gets difficult for the Physics Engine to simulate if the mass is too low.
            -- Hence we are increasing the mass.
            mass_obj=sim.getShapeMass(objects[i])
            if (mass_obj<0.8) then
                sim.setShapeMass(objects[i],0.8)
            end
            
            -- Again if the inertia/mass is < 2x10^-3, we will increase it.
            inertiaMatrix,transformationMatrix=sim.getShapeInertia(objects[i])
            
            for i, inertia in ipairs(inertiaMatrix) do
                if (i==1 or i==5 or i==9)then
                    if ((inertia/mass_obj)<0.02) then
                        inertiaMatrix[i]=0.02*mass_obj
                    end
                end
            end
            
            sim.setShapeInertia(objects[i],inertiaMatrix,transformationMatrix)
            
            -- If result is not 1 i.e. failed...throw error
            if result ~= 1 then
                print("")
                print("[ERROR] This dynamic body should be any of the following shapes only- Convex shape or Convex compound shape or  Pure simple shape or Pure compound shape  : "..sim.getObjectName(objects[i]))
                r = false
                return r
            end

        elseif string.find(sim.getObjectName(objects[i]), "visible") and (sim.getObjectType(objects[i]) == sim.object_shape_type) then
            
            if parameter_static ~= 1 then    -- 1 corresponds to static shape
                print("")
                print("[ERROR] This visible body is currently non-static : "..sim.getObjectName(objects[i]))
                r = false
                return r
            elseif parameter_respondable ~= 0 then   -- 1 corresponds to respondable
                print("")
                print("[ERROR] This visible body is currently respondable : "..sim.getObjectName(objects[i]))
                r = false
                return r
            end
    
        elseif sim.getObjectType(objects[i]) ~= sim.object_joint_type and sim.getObjectType(objects[i]) ~= sim.object_forcesensor_type then
                    
            print("")
            print("[ERROR] This shape's name is incorrect as per the required convention : "..sim.getObjectName(objects[i]))
            r = false
            return r

        end    


    end

    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --





    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    -- CHECK: no of joints, calculate total torque, force required
    
    no_of_joints = 0
    torque = 0
    force = 0
    individual_values = {}    -- storing individual joints torque/force
    -- Iterating whole arm
    for i, obj_handle in ipairs(objects) do

        


        current_object_type = sim.getObjectType(objects[i])
        -- If current object is a joint type or force sensor type
        if  current_object_type == sim.object_joint_type or current_object_type == sim.object_forcesensor_type then
            
            -- not taking force sensor reading as it is a rigid link
            if current_object_type == sim.object_joint_type then
                no_of_joints = no_of_joints + 1

                -- Capping Joint's Upper Velocity Limit to 30 deg/s i.e. 0.523599 rad/s
                local cap = 0.523599
                local upper_vel_in_rad = sim.getObjectFloatParam( objects[i], sim.jointfloatparam_upper_limit)
                if upper_vel_in_rad > cap then
                    sim.setObjectFloatParam( objects[i], sim.jointfloatparam_upper_limit, cap)
                end

                if string.find(sim.getObjectName(objects[i]), "rj") and sim.getJointType(objects[i]) == sim.joint_revolute_subtype then      -- Revolute joint then torque
                    max_torque = sim.getJointMaxForce(objects[i])
                    table.insert(individual_values, max_torque.."Nm")
                    torque = torque + max_torque
                    
                elseif string.find(sim.getObjectName(objects[i]), "pj") and sim.getJointType(objects[i]) == sim.joint_prismatic_subtype then  -- Primatic joint then force
                    max_force = sim.getJointMaxForce(objects[i])
                    table.insert(individual_values, max_force.."N")
                    force = force + max_force
                    
                else
                    if sim.getJointType(objects[i]) == sim.joint_spherical_subtype then
                    
                        print("")
                        print("[ERROR] Spherical joint is not allowed : "..sim.getObjectName(objects[i]))
                    
                    else
                    
                        print("")
                        print("[ERROR] Naming convention not followed for "..sim.getObjectName(objects[i]))
                    
                    end
                    r = false
                    return r

                end
            end
            
        end
        
    end
    folder_path = sim.getStringSignal('get_folder_name_customisation')
    log_file = io.open(folder_path .. "/bm_logfile.txt", "w")
    io.output(log_file)
    print('Total number of joints                     : '..no_of_joints)
    print('Total torque required                      : '..torque..' Nm')
    print('Total force required                       : ' ..force..'N')
    print("Recorded torque/force for joints in order  : ")
    io.write('Total number of joints:'..no_of_joints.."\n")
    io.write('Total torque required:'..torque.."\n")
    io.write('Total force required:'..force.."\n")
    print(individual_values)

    for val = 1, #individual_values do
        io.write("Individual Values:" .. individual_values[val] .. "\n")
    end
    -- Checking joints constraint
    if (no_of_joints < 1 or no_of_joints > 6) then
        eval_no_of_joints = 0
        print("")
        print("[ERROR] No. of joints in your model doesn\'t meet the given constraints")
        r = false
        return r
    else
        eval_no_of_joints = 1
        -- print('No. of joints constraint satisfied')
    end
    io.write('Eval Number of Joints:'..eval_no_of_joints.."\n")
    io.close(log_file)
    -- sim.setIntegerSignal('eval_no_of_joints', eval_no_of_joints)
    -- sim.setFloatSignal('torque',torque)
    -- sim.setFloatSignal('force', force)

    
    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    
    -- r = bb_vol()
    
    return r
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

function set_camera_before_run()
    -- Function to build hierarchy to reveal the robot

    -- local pose = {1.692300797, -0.9232017994, -1.257745504, 0.05531254411, -0.4251197577, -0.5650444627, 0.7049387097}
    -- local pose = {2.075736284, -1.156342268, -1.624120235, 0.05531253666, -0.4251212478, -0.5650471449, 0.7049356103}
    -- local start_pose = {0.9593454599, -1.217581272, -1.503504276, 0.1416777372, 0.3513730466, 0.6136090755, -0.6927829981}
    local start_pose = {0.6471632719, 1.369236946, -1.530833006, -0.282384932, -0.1902547479, 0.6810308695, -0.6482737064}
    -- sim.setObjectParent(camera_handle,BM_Bot_handle, true)
    sim.setObjectPose(camera_handle, BM_Bot_handle, start_pose)

    -- Creating revolute joint which will used to reveal the bot
    camera_revolute_joint_handle = sim.createJoint( sim_joint_revolute_subtype, sim.jointmode_force, 0, {0.05, 0.01})
    result = sim.setObjectName( camera_revolute_joint_handle, "rj_for_camera")
    -- Enabling motor
    sim.setObjectInt32Param( camera_revolute_joint_handle, sim.jointintparam_motor_enabled, 1)
    -- Setting its position to that BM_Bot frame
    sim.setObjectPosition( camera_revolute_joint_handle, BM_Bot_handle, {0,0,0})

    -- Adding a cuboid
    support_cuboid_handle = sim.createPureShape( 0, 0, {0.01, 0.01, 0.01}, 1, nil)
    sim.setObjectName( support_cuboid_handle, "cuboid_for_camera")
    -- Setting its position to that of Default Camera
    sim.setObjectPosition( support_cuboid_handle, camera_handle, {0,0,0})

    inertiaMatrix,transformationMatrix=sim.getShapeInertia(support_cuboid_handle)
            
    for i, inertia in ipairs(inertiaMatrix) do
        if (i==1 or i==5 or i==9) then
                inertiaMatrix[i] = 3.495e-03
        end
    end
    
    sim.setShapeInertia( support_cuboid_handle, inertiaMatrix, transformationMatrix)

    -- Setting BM_Bot as parent of revolute joint
    sim.setObjectParent( camera_revolute_joint_handle, BM_Bot_handle, true)

    -- Setting revolute joint as parent of support cuboid
    sim.setObjectParent( support_cuboid_handle, camera_revolute_joint_handle, true)

    -- Setting support cuboid as parent of DefaultCamera
    sim.setObjectParent( camera_handle, support_cuboid_handle, true)

    if_added = true


end


function end_program()
    -- This function would call organize_screen_end


    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''
    

    -- Removing Dummy, Enabling all toolbars
    organize_screen_end( inInts, inFloats, inStrings, inBuffer)

end

function organize_screen_init()
    -- This function disables all the menus and toolbars
    
    -- Clearing status bar
    sim.addStatusbarMessage(nil)

    -- real time ON
    sim.setBoolParam(sim.boolparam_realtime_simulation,true)

    -- disable undo/redo functionality and hide bounding boxes of selected objects
    sim.setInt32Param(sim.intparam_settings,2)
    
    -- sim.setNavigationMode(sim.navigation_passive)
    sim.setBoolParam(sim.boolparam_hierarchy_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_hierarchy_visible,false)
    sim.setBoolParam(sim.boolparam_objproperties_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_browser_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_browser_visible,false)
    sim.setBoolParam(sim.boolparam_pause_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_objectshift_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_objectrotate_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_calcmodules_toolbarbutton_enabled,false)
    sim.setBoolParam(sim.boolparam_statustext_open,true)
    sim.setBoolParam(sim.boolparam_infotext_visible,true)
    

    -- Highest priority for our scripts ( eval_bm)
    self_handle = dummy_handle
    child_script_handle = sim.getScriptAssociatedWithObject(self_handle)
    cutomization_script_handle = sim.getCustomizationScriptAssociatedWithObject(self_handle)
    sim.setScriptAttribute(cutomization_script_handle,sim.scriptattribute_executionorder, sim.scriptexecorder_first) 
    sim.setScriptAttribute(child_script_handle,sim.scriptattribute_executionorder, sim.scriptexecorder_first)

end


function get_camera_handle()
    camera_handle    = sim.getObjectHandle('DefaultCamera')
end


function organize_screen_end( inInts, inFloats, inStrings, inBuffer)
    -- This function remove the Dummy model and enable all the menus and toolbars

    if (pcall(get_camera_handle)) then                -- If Default Camera exists
        sim.setObjectParent(camera_handle,-1, true)
    end

    -- if if_added then
    --     sim.removeObject(support_cuboid_handle)
    --     sim.removeObject(camera_revolute_joint_handle)
    -- end
    
    sim.removeModel(dummy_handle)
    print("")
    print("Evaluation stopped.")
    
    -- enable undo/redo functionality
    sim.setInt32Param(sim.intparam_settings,22)
    
    -- sim.setNavigationMode(15873)
    sim.setBoolParam(sim.boolparam_hierarchy_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_hierarchy_visible,true)
    sim.setBoolParam(sim.boolparam_objproperties_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_browser_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_browser_visible,true)
    sim.setBoolParam(sim.boolparam_pause_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_objectshift_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_objectrotate_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_calcmodules_toolbarbutton_enabled,true)
    sim.setBoolParam(sim.boolparam_statustext_open,true)
    sim.setBoolParam(sim.boolparam_infotext_visible,true)
    
    
end


function sysCall_nonSimulation()
    -- is executed when simulation is not running
    before_simulation_system_time=sim.getSystemTime()
    
    if(before_simulation_system_time-init_system_time>=30) then
        print('[ERROR] Uh oh! Simulation was not started. Please try again.')
        organize_screen_end(inInts,inFloats,inStrings,inBuffer)
    end        
end

function sysCall_beforeSimulation()
    -- is executed before a simulation starts

end

function sysCall_afterSimulation()
    -- is executed before a simulation ends
    init_system_time=sim.getSystemTime()    
end


function sysCall_resume()
    -- If students pauses simulation

    -- Simulation is about to resume
    sim_state = sim.getSimulationState()
    if(sim_state==sim.simulation_paused)then
        print('[ERROR] Simulation paused by the user.')
        print('Evaluation can not proceed further.')
        end_program()       -- end program will remove disc and reenable all toolbars
    
    end 
end


function get_required_data_custom( inInts, inFloats, inStrings, inBuffer)
    -- This function sends back eval data to python script

    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''

    eval_result = {eval_no_of_joints, no_of_joints, torque, force}

    eval_data_to_be_sent = { table.concat(eval_result,","), table.concat(individual_values,",")}

    -- print('Customization data_to_be_sent:')
    -- print(data_to_be_sent)
    return inInts,inFloats,eval_data_to_be_sent,inBuffer

end 


function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details