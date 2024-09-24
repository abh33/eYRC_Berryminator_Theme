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
    inInts={}
    inFloats={}
    inStrings={''}
    inBuffer=''

    dummy_handle  = sim.getObjectHandle('BM_Dummy_3')

    -- Clearing status bar
    sim.addStatusbarMessage(nil)

    -- real time ON
    sim.setBoolParam(sim.boolparam_realtime_simulation,true)

    sim.setStringSignal('gfh36801nc','0')
    flag_error = 0
    
    if(pcall(organize_screen_init))then
        print("Evaluation started successfully.")
        print("")

        if(pcall(handles))then

            -- print('Getting handles complete.')
            r = all_objects_check()
            if r then

                sim.setStringSignal('gfh36801nc','1')    -- indicating to python that scene check is complete
                -- After this python will replace the BM_Bot and start running their task_3_primary() function

            else

                flag_error = 1

            end

        else

            print("[ERROR] Couldn't find BM_Bot / vision_sensor_1 / DefaultCamera in the scene")
            flag_error=1

        end

    else
        print("[ERROR] Re-download exe file")
    end


    if(flag_error == 1)then

        sim.setStringSignal('gfh36801nc','0')
        end_program()
        print('[ERROR] Evaluation can not proceed further.')

    end


end


function handles()
    -- Getting handles

    BM_Bot_handle = sim.getObjectHandle('BM_Bot')
    vs_handle     = sim.getObjectHandle('vision_sensor_1')
    camera_handle = sim.getObjectHandle('DefaultCamera')
    
end


function all_objects_check()
    -- This function will deactivate any script, detect extra objects added in the scene other than the default
    r = true

    all_objects_in_scene = sim.getObjectsInTree( sim_handle_scene, sim.handle_all, 0)
    -- print(all_objects_in_scene)
    -- Get all objects inside BM_Bot_handle
    objects = sim.getObjectsInTree(BM_Bot_handle,sim.handle_all, 0)
    -- print(objects)


    names = {'DefaultCamera', 'Floor', 'QR_Plane', 'DefaultLights', 'DefaultLightA', 'DefaultLightB', 'DefaultLightC', 'DefaultLightD',
         'XYZCameraProxy', 'DefaultNXViewCamera', 'DefaultNYViewCamera', 'DefaultNZViewCamera', 'DefaultXViewCamera', 'DefaultYViewCamera', 'DefaultZViewCamera',
         'BM_Dummy_3'}



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

            else
                if sim.getObjectName(all_objects_in_scene[i]) ~= 'BM_Dummy_3' then
                    -- Deactivating their scripts for each object if any
                    student_child_script_handle = sim.getScriptAssociatedWithObject(obj_handle)
                    student_cutomization_script_handle = sim.getCustomizationScriptAssociatedWithObject(obj_handle)
                    if (student_child_script_handle ~= -1) then
                        sim.setScriptAttribute( student_child_script_handle, sim.scriptattribute_enabled, false)
                        -- print("child done")
                    elseif (student_cutomization_script_handle ~= -1) then
                        sim.setScriptAttribute( student_cutomization_script_handle, sim.scriptattribute_enabled, false)
                        -- print("custom done")
                    end
                end

            end
        end

    end

    -- Setting DefaultCamera's position and orientation
    sim.setObjectPosition( camera_handle, -1, {2.81, 3.1, 14.678})
    sim.setObjectOrientation( camera_handle, -1,{ -3.14, 0, 3.14})

    return r

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
    

    -- Highest priority for our scripts ( BM_Dummy_3)
    self_handle = dummy_handle
    child_script_handle = sim.getScriptAssociatedWithObject(self_handle)
    cutomization_script_handle = sim.getCustomizationScriptAssociatedWithObject(self_handle)
    sim.setScriptAttribute(cutomization_script_handle,sim.scriptattribute_executionorder, sim.scriptexecorder_first) 
    sim.setScriptAttribute(child_script_handle,sim.scriptattribute_executionorder, sim.scriptexecorder_first)

end


function organize_screen_end( inInts, inFloats, inStrings, inBuffer)
    -- This function remove the Dummy model and enable all the menus and toolbars
    
    sim.removeModel(dummy_handle)
    print("")
    print("Evaluation stopped.")
    
    
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
    
end

function sysCall_beforeSimulation()
    -- is executed before a simulation starts

end

function sysCall_afterSimulation()
    -- is executed before a simulation ends
    
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

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details