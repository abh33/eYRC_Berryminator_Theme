--[[
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This Lua script is to evaluate Task 4 of Berryminator (BM) Theme (eYRC 2021-22).
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

    if(pcall(handles))then
        
        if(pcall(organize_screen_init))then
            print("Evaluation started successfully.")
            print("")
            -- print('Getting handles complete.')
            r = all_objects_check()
            if r then

                if (pcall(set_all_objects_special_property)) then
                    
                    r = arm_check()
                    rotate_plants()
                    set_camera_pose()

                    if r then
                        sim.setStringSignal('gfh36801nc','1')    -- indicating to python that scene check is complete
                        -- After this python will replace the BM_Bot and start running their task_4_primary() function
                    else
                        flag_error = 1                                                                                                                                                                                                                                    
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

    local vertical_rack_1 = sim.getObjectHandle('vertical_rack_1')
    local vertical_rod_1 = sim.getObjectHandle('vertical_rod_1')
    local base_2 = sim.getObjectHandle('base_2')
    local base_3 = sim.getObjectHandle('base_3')

    lemon_tree_pot = sim.getObjectHandle('lemon_tree_pot')
    blueberry_tree_pot = sim.getObjectHandle('blueberry_tree_pot')
    strawberry_tree_pot = sim.getObjectHandle('strawberry_tree_pot')

    local collection_box_1 = sim.getObjectHandle('collection_box_1')
    local cb_qr_1 = sim.getObjectHandle('cb_qr_1')

    local qr_plane_handle = sim.getObjectHandle("QR_Plane")
    local floor_handle = sim.getObjectHandle('Floor')
    
end


function set_all_objects_special_property()

    local all_handles = sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0)
    local total_objects_count = #all_handles
    -- local index = 1

    for index=1, total_objects_count, 1
    do
        sim.setObjectSpecialProperty(all_handles[index],1)    -- turning collidable property ON for all objects in scene

    end

    -- Berries present in the scene
    local berries_list = { 
        'lemon_1', 'lemon_2', 'lemon_3', 'lemon_4', 'blueberry_1', 'blueberry_2', 'blueberry_3', 'blueberry_4', 'strawberry_1', 'strawberry_2', 'strawberry_3', 'strawberry_4'        
    }

    local qr_plane_handle = sim.getObjectHandle("QR_Plane")
    sim.setObjectSpecialProperty(qr_plane_handle, 512)       -- QR Plane - only renderable


    -- local index = 1
    for index=1, #berries_list, 1
    do
        local berry_handle = sim.getObjectHandle(berries_list[index])
        sim.setObjectSpecialProperty( berry_handle, 513)    -- All berries - collidable and renderable

    end

    local floor_handle = sim.getObjectHandle('Floor')
    sim.setObjectSpecialProperty(floor_handle, 0)       -- Floor - nothing ON

    sim.setObjectSpecialProperty(dummy_handle, 0)       -- Dummy - nothing ON

end


function all_objects_check()
    -- This function will [deactivate any script], detect extra objects added in the scene other than the default
    r = true

    all_objects_in_scene = sim.getObjectsInTree( sim.handle_scene, sim.handle_all, 0)
    -- print(all_objects_in_scene)
    -- Get all objects inside BM_Bot_handle
    objects = sim.getObjectsInTree(BM_Bot_handle,sim.handle_all, 0)
    -- print(objects)


    names = {'DefaultCamera', 'Floor', 'QR_Plane', 'DefaultLights', 'DefaultLightA', 'DefaultLightB', 'DefaultLightC', 'DefaultLightD',
         'XYZCameraProxy', 'DefaultNXViewCamera', 'DefaultNYViewCamera', 'DefaultNZViewCamera', 'DefaultXViewCamera', 'DefaultYViewCamera', 'DefaultZViewCamera',
         'collection_box_1', 'cb_qr_1', 'vertical_rack_1', 'lemon_tree_pot', 'lemon_tree_branch', 'lemon_tree_leaves',
         'force_sensor_l1', 'force_sensor_l2', 'force_sensor_l3', 'force_sensor_l4', 'lemon_1', 'lemon_2', 'lemon_3', 'lemon_4',
         'vertical_rod_1', 'base_2', 'base_3',
         'blueberry_tree_pot', 'blueberry_tree_branch', 'blueberry_tree_leaves',
         'force_sensor_b1', 'force_sensor_b2', 'force_sensor_b3', 'force_sensor_b4', 'blueberry_1', 'blueberry_2', 'blueberry_3', 'blueberry_4',
         'strawberry_tree_pot', 'strawberry_tree_branch', 'strawberry_tree_leaves',
         'force_sensor_s1', 'force_sensor_s2', 'force_sensor_s3', 'force_sensor_s4', 'strawberry_1', 'strawberry_2', 'strawberry_3', 'strawberry_4',
         'eval_bm', 'target'}



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


    -- Detecting extra objects in the scene and deactivating their scripts(outside BM_Bot) if any, except ours
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
    
    print('Total number of joints                     : '..no_of_joints)
    print('Total torque required                      : '..torque..' Nm')
    print('Total force required                        : '..force..' N')
    print("Recorded torque/force for joints in order  : ")
    print(individual_values)
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
    -- sim.setIntegerSignal('eval_no_of_joints', eval_no_of_joints)
    -- sim.setFloatSignal('torque',torque)
    -- sim.setFloatSignal('force', force)

    
    -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
    
    -- r = bb_vol()
    
    return r
end


function rotate_plants()

    -- angles_choice = {0, 1.57, 3.14, 4.71}
    angles_choice = {0, 0.610, -0.610}
    -- print( angles_choice[ math.random( #angles_choice ) ] )

    sim.setObjectOrientation(lemon_tree_pot     ,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})
    sim.setObjectOrientation(blueberry_tree_pot ,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})
    
    -- Because of the position of the collection box, we can not rotate strawberry_tree_pot. 
    --sim.setObjectOrientation(strawberry_tree_pot,-1,{0,0,angles_choice[ math.random( #angles_choice ) ]})

    sim.setObjectOrientation(strawberry_tree_pot,-1,{0,0,0})

end

function set_camera_pose()

    -- local pose = {1.692300797, -0.9232017994, -1.257745504, 0.05531254411, -0.4251197577, -0.5650444627, 0.7049387097}
    local pose = {2.075736284, -1.156342268, -1.624120235, 0.05531253666, -0.4251212478, -0.5650471449, 0.7049356103}

    -- sim.setObjectParent(camera_handle,BM_Bot_handle, true)
    sim.setObjectPose(camera_handle, BM_Bot_handle, pose)

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
    
    sim.setNavigationMode(sim.navigation_passive)
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


function organize_screen_end( inInts, inFloats, inStrings, inBuffer)
    -- This function remove the Dummy model and enable all the menus and toolbars

    sim.setObjectParent(camera_handle,-1, true)
    
    sim.removeModel(dummy_handle)
    print("")
    print("Evaluation stopped.")
    
    -- enable undo/redo functionality
    sim.setInt32Param(sim.intparam_settings,22)
    
    sim.setNavigationMode(15873)
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