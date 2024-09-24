'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement the Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[  1010  ]
# Author List:		[ D KARTHIK SAINADH REDDY, AVVARU YASWANTH, G BHANU CHANDANA, P TEJAS KRISHNA ]
# Filename:			theme_implementation.py
# Functions:		
# Global variables:	
# 					[ B_cb1,B_cb2,S_cb1,S_cb2,L_cb1,L_cb2,emptybuff,berries_in_cb1, berries_in_cb2 ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode

from task_3 import *
from task_2a import *
from task_1b import *
##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim
    
except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()

##################global variables############################

B_cb1,B_cb2 = 0,0                   # Blueberry in cb1, Blueberry in cb2
S_cb1,S_cb2 = 0,0                   # Strawberry in cb1, Strawberry in cb2
L_cb1,L_cb2 = 0,0                   # Lemon in cb1, Lemon in cb2
emptybuff = bytearray()
berries_in_cb1, berries_in_cb2 = 0,0

##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):

    """
    Purpose:
    ---
    Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.
    
    NOTE: 
    1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
        Hence before plucking, the correct berry should be identified and sent via this function.

    2.	Accuracy of detection should be +-0.025m.

    Input Arguments:
    ---
    `client_id` 	:  [ integer ]
        the client_id generated from start connection remote API, it should be stored in a global variable

    'berry_name'		:	[ string ]
            name of the detected berry.

    'x_coor'			:	[ float ]
            x-coordinate of the centroid of the detected berry.

    'y_coor'			:	[ float ]
            y-coordinate of the centroid of the detected berry.

    'depth'			:	[ float ]
            z-coordinate of the centroid of the detected berry.

    Returns:
    ---
    `return_code`		:	[ integer ]
            A remote API function return code
            https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

    Example call:
    ---
    return_code=send_identified_berry_data(berry_name,x_coor,y_coor)
    
    """

    ##################################################
    ## You are NOT allowed to make any changes in the code below. ##
    emptybuff = bytearray()

    if(type(berry_name)!=str):
        berry_name=str(berry_name)

    if(type(x_coor)!=float):
        x_coor=float(x_coor)

    if(type(y_coor)!=float):
        y_coor=float(y_coor)	
    
    if(type(depth)!=float):
        depth=float(depth)
    
    data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
    return return_code

def open_close_gripper(client_id,command):

    """
    Purpose:
    ---
        open and close the gripper based on command transfered.
        
    Input Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `command`    :    [ String ] 
        open or close to instruct the functioning of the gripper.
        
    Returns:
    ---
        'None'
        
    Example Call :
    ---
    open_close_gripper(client_id, "open")
    
    """
    
    ###################### GLOBAL VARIABLES #####################
    
    global emptybuff
        
    ############################ OUR CODE ##############################
    
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'gripper',sim.sim_scripttype_childscript,'open_close', [], [],[command], emptybuff,sim.simx_opmode_blocking)
    _, joint_handle = sim.simxGetObjectHandle(client_id, 'RG2_openCloseJoint', sim.simx_opmode_blocking)
    
    if(command == "open"):
        while True:
            distance = sim.simxGetJointPosition(client_id, joint_handle, sim.simx_opmode_oneshot)
            if(distance[1] > 0.0455):
                break
    
    if(command == "close"):
        while True:
            distance = sim.simxGetJointPosition(client_id, joint_handle, sim.simx_opmode_oneshot)
            if(distance[1] < 0.0005):
                break
    time.sleep(1)
    
    ###################################################################
    
    return 

def pick(client_id,berry_coord):
    
    """
    Purpose:
    --- 
    Calls the pick function defined in lua code child script
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `berry_coord`     :     [ list ]
        3D coordinates of berry with respect to vision sensor.
        
    Returns:
    ---
    'None'
        
    """
    
    ###################### GLOBAL VARIABLES #####################
    
    global emptybuff
        
    ############################ OUR CODE ##############################
    
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'pick', [], berry_coord, [], emptybuff,sim.simx_opmode_blocking)
    time.sleep(2.75)
    
    ###################################################################
    
    return

def homePosition(client_id,berry_coord):
    
    """
    Purpose:
    ---
       Calls lua script function which sets the robotic arm to home position of itself.
       
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
    
    `berry_coord`    :     [ list ]
        3D coordinates of detected berry
        
    Returns:
    ---
    None
    
    """
    
    ###################### GLOBAL VARIABLES #####################
    
    global emptybuff
    
    ############################ OUR CODE ##############################
    
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'homePosition', [], berry_coord, [],emptybuff, sim.simx_opmode_blocking)
    time.sleep(2.5)
    
    ###################################################################
    
    return

def place(client_id,basket_left_right):
    
    """
    Purpose:
    ---
        Calls lua script function to place the target dummy 
        over the respective sub basket based on the parameter value
        
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `basket_left_right`    :    `berry_coord` [ integer ]
        the left basket indicated by `1` and right basket by `2`
        
    Returns:
    ---
    None
    
    """
    
    ###################### GLOBAL VARIABLES #####################
    
    global emptybuff
    
    ############################ OUR CODE ##############################
    
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'place', [], [basket_left_right], [], emptybuff,sim.simx_opmode_blocking)
    time.sleep(2.5)
    
    ###################################################################
    
    return
               
def pick_and_place(client_id):
    
    """
    
    Purpose:
    --- 
        Calls all pick and place mechanism functions in order to pick and place the fruit in respective basket.
        
    Aruments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ############################ GLOBAL VARIABLES #############################
    
    global emptybuff
    global B_cb1,B_cb2,S_cb1,S_cb2,L_cb1,L_cb2
        
    ############################# OUR CODE ############################
    
    berries_dictionary, berries_position_dictionary = getcoord(client_id)
    targets, data_to_send = select_berries_to_pluck(client_id, berries_position_dictionary)
    
    # set all wheel velocities to 0 to stop the bot for pick and place
    wheel_velocity_setting(client_id,0,0,0,0)
    
    # sort and pick the fruits which are near to visionsensor first
    data_to_send = sorted(data_to_send, key=lambda x:x[4])
    
    for ele in data_to_send:
        
        # send berry data to evaluation script
        send_identified_berry_data(client_id, ele[1], ele[2], ele[3], ele[4])
        
        # gets 3d coordinartes of berry from data_to_send
        berry_data = list(ele[2:5])
        
        # pick
        open_close_gripper(client_id,"open")
        pick(client_id,berry_data)
        open_close_gripper(client_id,"close")
        
        #home position setting
        homePosition(client_id,berry_data)
        
        # placing
        place(client_id,ele[5])
        open_close_gripper(client_id,"open")
        homePosition(client_id,berry_data)
    
    # clean the inverse kinematics environment in coppleasim after finishing the pick and place in room
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'cln', [], [], [],emptybuff,sim.simx_opmode_blocking)
    
    ###################################################################
    
    return 
    
def getcoord(client_id):
    
    """
    
    Purpose:
    ---
        Finds the berries coordinates with respect to vision sensor 2 in scene and returns its 3d coordinates.
        
    Arguments:
    ---
    `client_id`     :     [ integer ]
        Connection in coppleasim using remote api

    Returns:
    ---
    `berries_dictinary`     :      [ dictionary ]
        Contains detected berries data

    `berries_position_dictionary`     :     [ dictionary ] 
        Contains 3D coordinate data of berries to be picked

    """

    ################################# OUR CODE ###############################
    
    _,vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_oneshot)
    vision_sensor_image, image_resolution, return_code = Get_vision_sensor_image(client_id)

    vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id, vision_sensor_handle)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
    
    berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)
    
    ##########################################################################
    
    return berries_dictionary,berry_positions_dictionary

def select_berries_to_pluck(client_id,berries_position_dictionary):
    
    """
    
    Purpose:
    ---
        Filters the berries to be picked depending on required berries.
        
    Arguments:
    ---
    `client_id`     :      [ integer ]
        Connection status to coppleasim.
        
    `berries_position_dictionary`      :      [ dictionary ]
        3D coordinates of berries to be picked

    Returns:
    ---
        `data_to_send`  :   list of lists containg berry 3d coordinates, name and basket it should be fallen in.
        
    """

    targets=[] 
    data_to_send=[]
    
    ######################## GLOBAL VARIABLES #########################
    
    global B_cb1,B_cb2,S_cb1,S_cb2,L_cb1,L_cb2
    
    ############################# OUR CODE ############################
    
    bcb1,bcb2 = B_cb1,B_cb2
    scb1,scb2 = S_cb1,S_cb2
    lcb1,lcb2 = L_cb1,L_cb2
    
    # ------------------- Blueberry Filter ------------------- #
    
    #removing duplicates
    berries_position_dictionary["Blueberry"] = list(set(berries_position_dictionary["Blueberry"]))
    berries_position_dictionary["Lemon"] = list(set(berries_position_dictionary["Lemon"]))
    berries_position_dictionary["Strawberry"] = list(set(berries_position_dictionary["Strawberry"]))
    
    
    if(max(bcb1, bcb2) >= len(berries_position_dictionary["Blueberry"]) ):
        
        if(bcb1 == 0):
            bcb2 = bcb2 - len(berries_position_dictionary["Blueberry"])
        
        else:
            bcb1 = bcb1 - len(berries_position_dictionary["Blueberry"])
    
    else:
        diff = abs(max(bcb1, bcb2) - len(berries_position_dictionary["Blueberry"]))
    
        for i in range(0,diff):
            berries_position_dictionary["Blueberry"].remove(berries_position_dictionary["Blueberry"][-1])
    
            if(bcb1 == 0):
                bcb2 = bcb2 - 1
    
            else:
                bcb1 = bcb1 - 1


    # -------------------- Lemon Filter -------------------- #
    
    if(max(lcb1, lcb2) >= len(berries_position_dictionary["Lemon"]) ):
        
        if(lcb1 == 0):
            lcb2 = lcb2 - len(berries_position_dictionary["Lemon"])
        
        else:
            lcb1 = lcb1 - len(berries_position_dictionary["Lemon"])
    
    else:
        diff = abs(max(lcb1, lcb2) - len(berries_position_dictionary["Lemon"]))
    
        for i in range(0,diff):
            berries_position_dictionary["Lemon"].remove(berries_position_dictionary["Lemon"][-1])
    
            if(lcb1 == 0):
                lcb2 = lcb2 - 1
    
            else:
                lcb1 = lcb1 - 1
    
    
    # -------------------- Strawberry Filter -------------------- #
    
    if(max(scb1, scb2) >= len(berries_position_dictionary["Strawberry"]) ):
    
        if(scb1 == 0):
            scb2 = scb2 - len(berries_position_dictionary["Strawberry"])
    
        else:
            scb1 = scb1 - len(berries_position_dictionary["Strawberry"])
    
    else:
        diff = abs(max(scb1, scb2) - len(berries_position_dictionary["Strawberry"]))
    
        for i in range(0,diff):
            berries_position_dictionary["Strawberry"].remove(berries_position_dictionary["Strawberry"][-1])
    
            if(scb1 == 0):
                scb2 = scb2 - 1
    
            else:
                scb1 = scb1 - 1
   
   
    for ele in berries_position_dictionary:
        cb_no = 1
    
        if(len(berries_position_dictionary[ele]) != 0):
    
            for i in range(0,len(berries_position_dictionary[ele])):
                coord = berries_position_dictionary[ele][i]   
    
                if(ele == "Strawberry" and S_cb1 == 0):
                    cb_no = 2
    
                if(ele == "Blueberry" and B_cb1 == 0):
                    cb_no = 2
    
                if(ele == "Lemon" and L_cb1 == 0):
                    cb_no = 2
    
                data = [client_id,ele,coord[0],coord[1],coord[2],cb_no]
                targets.append(coord)
                data_to_send.append(data)
    
    B_cb1,B_cb2,S_cb1,S_cb2,L_cb1,L_cb2 = bcb1,bcb2,scb1,scb2,lcb1,lcb2     # update global variables
        
    ######################################################################
    
    return targets,data_to_send

def Get_vision_sensor_image(client_id):
    
    """

    Purpose: 
    ---
    Scans the image of vision_sensor_2 and gets the image with image resolution values
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
  
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
  
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
    
    """ 
    
    ############################## OUR CODE ##################################
       
    _,cam_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, cam_handle, 0, sim.simx_opmode_blocking)
    
    ###########################################################################
    
    return vision_sensor_image, image_resolution, return_code

def berries_to_pick(client_id,decoded_data):
    
    """
    
    Purpose:
    ---
        Depending on values of json file, the berries will be assigned to global variables and will be filtered out.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `decoded_data`    :     [ list ] 
        data decoded from json file

    Returns:
    ---
    `berries_targets`     :    [ list [ list ] ]
        List having the details of all the berries
        
    `total_targets`    :     [ integer ]
        Count of all the targets
        
    """
    
    ######################## GLOBAL VARIABLES #########################
    
    global B_cb1,B_cb2,S_cb1,S_cb2,L_cb1,L_cb2,berries_in_cb1, berries_in_cb2
    
    ########################## OUR CODE ###########################
    
    # blueberries
    berries_count = int(decoded_data['B'][0])
    cb_number = int(decoded_data['B'][-1])
    
    if(cb_number == 1 ):
        B_cb1 = berries_count
        berries_in_cb1 = berries_in_cb1 + 1
        
    else:
        B_cb2 = berries_count
        berries_in_cb2 = berries_in_cb1 + 2
        
    Blueberry_list = ["Blueberry", berries_count, cb_number]
    
    # lemon
    berries_count = int(decoded_data['L'][0])
    cb_number = int(decoded_data['L'][-1])
    
    if(cb_number == 1 ):
        L_cb1 = berries_count
        berries_in_cb1 = berries_in_cb1 + 1
        
    else:
        L_cb2 = berries_count
        berries_in_cb2 = berries_in_cb1 + 2
        
    Lemon_list = ["Lemon", berries_count, cb_number]
    
    # strawberry
    berries_count = int(decoded_data['S'][0])
    cb_number = int(decoded_data['S'][-1])
    
    if(cb_number == 1 ):
        S_cb1 = berries_count
        berries_in_cb1 = berries_in_cb1 + 1
        
    else:
        S_cb2 = berries_count
        berries_in_cb2 = berries_in_cb1 + 2
        
    Strawberry_list = ["Strawberry", berries_count, cb_number]
    
    total_targets = Blueberry_list[1] + Lemon_list[1] + Strawberry_list[1]
    berries_targets = [Blueberry_list, Lemon_list, Strawberry_list]
    
    ################################################################
    
    return berries_targets, total_targets

def decide_rooms(client_id,target_berries, total_target_berries, rooms_entry):
    
    """
    Purpose:
    ---
    To decide the order of rooms to be traversed depending on the distances of the rooms entry coordinates.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `target_berries`     :     [ list ]
        berries with type of berry
        
    `total_target_berries`     :     [ integer ]
        total berries to be picked now
        
    `rooms_entry`    :    [ list ]
        List having the entry points of all rooms

    Returns:
    ---
    `rooms_to_go`   :    [ list ]
        Contains the list of rooms to be traversed
        
    """

    ############################## OUR CODE ##############################
    
    rooms_available = [4,3,2,1]
    rooms_to_go = []
    rooms_list = {}
    current_position = [4,4]

    #deciding first room
    for i in rooms_available:
        rooms_list[i] = get_dist(client_id,current_position, rooms_entry[i-1])
    
    sorted_rooms= {k: v for k, v in sorted(rooms_list.items(), key=lambda item: item[1])}  
    
    # creating a temporary variable
    temp = list(sorted_rooms)[0]
    rooms_to_go.append(temp)
    current_position = rooms_entry[temp-1]
    rooms_available.remove(temp)
   
    #deciding second room 
    rooms_list = {}
    
    for j in rooms_available:
        rooms_list[j] = get_dist(client_id,current_position, rooms_entry[j-1])
    
    sorted_rooms= {k: v for k, v in sorted(rooms_list.items(), key=lambda item: item[1])}
 
    temp = list(sorted_rooms)[0]
    
    rooms_to_go.append(temp)
    current_position = rooms_entry[temp - 1]
    rooms_available.remove(temp)

    #deciding third room
    rooms_list = {}
    
    for k in rooms_available:
        rooms_list[k] = get_dist(client_id,current_position, rooms_entry[k-1])
    
    sorted_rooms= {k: v for k, v in sorted(rooms_list.items(), key=lambda item: item[1])}
    temp = list(sorted_rooms)[0]
    
    rooms_to_go.append(temp)
    current_position = rooms_entry[temp - 1]
    rooms_available.remove(temp)
    
    # deciding fourth room
    rooms_to_go.append(rooms_available[0])
    
    for room in rooms_to_go:
        
        if(rooms_entry[room-1] == (3,6)):
        
            for i in range(len(rooms_to_go)):
        
                if(rooms_to_go[i] == 1):
        
                    temp_var = rooms_to_go.pop(i)
                    rooms_to_go.append(temp_var)
                    return rooms_to_go
                
        if(rooms_entry[room-1]==(5,6) or rooms_entry[room-1]==(5,8)):
            
            for i in range(len(rooms_to_go)):
            
                if(rooms_to_go[i] == 2):
            
                    temp_var = rooms_to_go.pop(i)
                    rooms_to_go.append(temp_var)
                    return rooms_to_go
    
    #####################################################################
    
    return rooms_to_go

def check_targets(client_id):
    
    """
    Purpose:
    ---
    To check whether all fruits from targets have been picked
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
        int: 1 if all fruits are picked and viceversa.
        
    """

    ######################### GLOBAL VARIABLES ##########################
    
    global B_cb1,B_cb2,L_cb1,L_cb2,S_cb1,S_cb2
    
    ################################### OUR CODE #######################################
    
    if(B_cb1 <= 0 and B_cb2 <= 0 and L_cb1 <= 0and L_cb2 <= 0and S_cb1 <= 0 and S_cb2 <= 0):
        return 1
    
    ###################################################################################
    return 0
    
def route_of_bot(client_id,entry_coord, coords):
    
    """
    Purpose:
    ---
    Decides the bot to enter the respective room, calls pick and place to pick the berries, 
    remembers the path it went in, after finishing its work,
    returns to entry coordinate using the data in remember path
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`     :     [ tuple ]
        entry coordinate of the room
        
    `coords`     :      [ list ]
        The list of tuples which bot should follow in path.
        
    Returns:
    ---
    None
        
    """
    global v
    
    ################################### OUR CODE ################################
    
    target_points,remember_path = [],[]
    
    setVelocity(client_id,12)
    
    target_points,remember_path = [],[]
    
    target_points = list(coords[0:1])
    remember_path = target_points
    task_3_primary(client_id, target_points)
    adjPos(client_id,entry_coord)    
    
    target_points = list(coords[1:3])
    remember_path = remember_path + target_points
    task_3_primary(client_id, target_points)

    x,y = getCentroid(client_id)
    setVelocity(client_id, 4)
    
    target_points = list(coords[3:])
    remember_path = remember_path + target_points
    task_3_primary(client_id, target_points)
    
    check_rack_position(client_id,entry_coord)
        
    remember_path = [ele for ele in reversed(remember_path)]
    target_points = remember_path[0:1]
    task_3_primary(client_id, target_points)
              
    open_close_gripper(client_id, "close")
      
    target_points = remember_path[1:2]
    task_3_primary(client_id, target_points)
    
    target_points = remember_path[2:3]
    task_3_primary(client_id, target_points)
    setVelocity(client_id, 3)
    
    # apply position adjustment only in necessary conditions
    if(len(remember_path) == 5):
        min_max(client_id, 1, y-3,y+4)
    
    setVelocity(client_id, 8)
        
    target_points = remember_path[3:]
    task_3_primary(client_id, target_points)
    
    min_max(client_id,0, 230,270)       # sets x axis position adjustments
    min_max(client_id,1, 230,270)       # sets y axis position adjustments
    
    ############################################################################
    
    return
    
def go_to_room1(client_id,entry_coord):
    
    """
    Purpose:
    ---
    Depending on entry coordinate, decides the list of tuples of path , bot should follow
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`    :     [ tuple ]
        room entry point
        
    Returns:
    ---
    None
        
    """
    
    ############################### OUR CODE ###############################
    
    # ----------- decide path to room 1------------ #
    
    a,b = entry_coord[0],entry_coord[1]
    
    if(a<3):
        route_of_bot(client_id,entry_coord, [(a,4),(a,b),(a,b+1),(1,7)])
    
    if(a == 3):
        route_of_bot(client_id,entry_coord, [(4,b),(a,b),(a-1,b),(1,6),(1,7)])
    
    ########################################################################
    return None
       
def go_to_room2(client_id,entry_coord):
    
    """
    Purpose:
    ---
    Depending on entry coordinate, decides the list of tuples of path , bot should follow
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`    :     [ tuple ]
        room entry point
        
    Returns:
    ---
    None
        
    """
    
    ############################### OUR CODE ###############################
    
    # ----------- decide path to room 2 ------------ #
    
    a,b = entry_coord[0],entry_coord[1]
    
    if(a > 5):
        route_of_bot(client_id,entry_coord, [(a,4),(a,b),(a,b+1),(7,7)])
    
    if(a == 5):
        route_of_bot(client_id,entry_coord, [(4,b),(a,b),(a+1,b),(a+2,b),(7,7)])
    
    #######################################################################
    return None
    
def go_to_room3(client_id,entry_coord):
    
    """
    Purpose:
    ---
    Depending on entry coordinate, decides the list of tuples of path , bot should follow
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`    :     [ tuple ]
        room entry point
        
    Returns:
    ---
    None
        
    """
    
    ############################### OUR CODE ###############################
    
    # ----------- decide path to room 3 ------------ #
    
    a,b = entry_coord[0],entry_coord[1]
    
    if(a > 5):
        route_of_bot(client_id,entry_coord, [(a,4),(a,b),(a,b-1),(7,1)])  
    
    if(a == 5):
        route_of_bot(client_id,entry_coord, [(4,b),(a,b),(a+1,b),(a+1,b-1),(7,1)])
    
    ########################################################################
    return None
    
def go_to_room4(client_id,entry_coord):
    
    """
    Purpose:
    ---
    Depending on entry coordinate, decides the list of tuples of path , bot should follow
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`    :     [ tuple ]
        room entry point
        
    Returns:
    ---
    None
        
    """
    
    ############################### OUR CODE ###############################
    
    # ----------- decide path to room 4------------ #
    
    a,b = entry_coord[0],entry_coord[1]
    
    if(a < 3 ):
        route_of_bot(client_id,entry_coord, [(a,4),(a,b),(a,b-1),(1,1)])
    
    if(a == 3):
        route_of_bot(client_id,entry_coord, [(4,b),(a,b),(a-1,b),(a-2,b),(1,1)])
    
    ########################################################################
    return None
    
def move_right(client_id,distance):
    
    """
    
    Purpose:
    ---
    To move the bot right side for a particular distance
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `distance`     :     [ integer ]
        
    Returns:
    ---
    None
    
    """
    
    ############################### OUR CODE ################################
    
    wheel_joints = init_setup(client_id)
    set_bot_movement(client_id, wheel_joints, 0, 2.5, 0)
    prev_wheels_data = encoders(client_id)
    
    while True:
        wheels_data = encoders(client_id)
        if(abs(wheels_data[0]-prev_wheels_data[0]) > distance):
            set_bot_movement(client_id, wheel_joints, 0,0,0)
            break

    ##########################################################################
    return None

def move_left(client_id,distance):
    
    """
    
    Purpose:
    ---
    To move the bot left side for a particular distance
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `distance`     :     [ integer ]
        
    Returns:
    ---
    None
    
    """
    
    ############################### OUR CODE ################################
    
    wheel_joints = init_setup(client_id)
    set_bot_movement(client_id, wheel_joints, 0, -2.5, 0)
    prev_wheels_data = encoders(client_id)
    
    while (True):
        wheels_data = encoders(client_id)
        if(abs(wheels_data[0]-prev_wheels_data[0]) > distance):
            set_bot_movement(client_id, wheel_joints, 0,0,0)
            break

    ########################################################################
    return None

def move_up(client_id,distance):
    
    """
    
    Purpose:
    ---
    To move the bot up side for a particular distance
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `distance`     :     [ integer ]
        
    Returns:
    ---
    None
    
    """
    
    ############################### OUR CODE ################################
    
    wheel_joints = init_setup(client_id)
    set_bot_movement(client_id, wheel_joints, 1.5, 0, 0)
    prev_wheels_data = encoders(client_id)
    
    while (True):
        wheels_data = encoders(client_id)
        if(abs(wheels_data[0]-prev_wheels_data[0]) > distance):
            break   

    #########################################################################
    return None

def move_down(client_id,distance):
    
    """
    
    Purpose:
    ---
    To move the bot down side for a particular distance
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `distance`     :     [ integer ]
        
    Returns:
    ---
    None
    
    """
    
    ############################### OUR CODE ################################
    
    wheel_joints = init_setup(client_id)
    set_bot_movement(client_id, wheel_joints, -1.5, 0, 0)
    prev_wheels_data = encoders(client_id)
    
    while (True):
        wheels_data = encoders(client_id)
        if(abs(wheels_data[0]-prev_wheels_data[0]) > distance):
            break  
    
    ######################################################################### 
    return None

def check_rack_position(client_id,entry_coord):
    
    """
    
    Purpose:
    ---
    Checks the room and rotates the bot in such a way that 
    1. vision sensor can scan the fruits and the pick and place in basket.
    2. After picking and placing, bot resets the position.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    `entry_coord`    :     [ tuple ]
        room entry point
        
    Returns:
    ---
    None
        
    """
    
    ######################### OUR CODE ###########################
    
    count = 0          # to count the rotations made by bot
    
    if(entry_coord[0] < 4 and entry_coord[1] < 4 ): #ROOM4
        min_max(client_id, 0,240,260)
        min_max(client_id, 1,240,260)
        rotate_bot(client_id, 1)
        count = count-1
    
    if(entry_coord[0] > 4 and entry_coord[1] < 4 ): #ROOM3
        min_max(client_id, 0,240,260)
        min_max(client_id, 1,290,320)   
        rotate_bot(client_id,0)
        rotate_bot(client_id,0)
        count = count+2
        
    if(entry_coord[0] > 4 and entry_coord[1] > 4 ): #ROOM2
        min_max(client_id,0,365,375)
        min_max(client_id,1,200,220)
        rotate_bot(client_id,0)
        count = count+1 

    if(entry_coord[0] < 4 and entry_coord[1] > 4 ): #ROOM1
        min_max(client_id,0,240,270)
        min_max(client_id,1,240,270)
        
    #----------phase 1---pick and place---------#
    move_up(client_id, 0.4)
    move_left(client_id, 6.25)
    pick_and_place(client_id)
    
    #----------phase 2---pick and place---------#
    move_right(client_id, 12.75)
    pick_and_place(client_id)
    move_left(client_id, 6.5)
    move_down(client_id, 0.4)
    
    # ROTATION OF BOT - RESETTING
    while ( count != 0 ):
    
        if( count < 0 ):
            rotate_bot(client_id,0)
            count = count + 1
    
        if( count > 0 ):
            rotate_bot(client_id,1)
            count = count - 1
        
    if(entry_coord[0] < 4 and entry_coord[1] < 4 ): #ROOM4
        min_max(client_id,0,330,365) 
        min_max(client_id,1,175,205)
      
    if(entry_coord[0] > 4 and entry_coord[1] < 4 ): #ROOM3
        min_max(client_id,1,330,360) 
        min_max(client_id,0,200,230)
    
    if(entry_coord[0] > 4 and entry_coord[1] > 4 ): #ROOM2
        min_max(client_id,0,335,360) 
        min_max(client_id,1,210,240)
    
    if(entry_coord[0] < 4 and entry_coord[1] > 4 ): #ROOM1
        min_max(client_id,0,325,360) 
        min_max(client_id,1,200,230)
    
    #################################################################
    return 

def drop_in_cb1(client_id):
    
    """
    
    Purpose:
    ---
    Navigates the bot to collection box 1 and adjust its position. 
    Then lifts the basket to drop the fruits in collection box1.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ############################## OUR CODE ###########################

    target_points = [(4,9),(2,11)]
    task_3_primary(client_id, target_points)
    min_max(client_id,0,367,381)
    min_max(client_id,1,335,350)
    print(getCentroid(client_id))
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'liftLeftBasket', [], [], [], emptybuff,sim.simx_opmode_blocking)
    _, basket_joint = sim.simxGetObjectHandle(client_id, 'basket_rj_r1', sim.simx_opmode_blocking)
    
    while True:
    
        var=sim.simxGetJointPosition(client_id, basket_joint, sim.simx_opmode_oneshot)
        var = var[1]*180/math.pi
    
        if(var>70):
            return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'downLeftBasket', [], [], [], emptybuff,sim.simx_opmode_blocking)           
            break
    
    while True:
        
        var=sim.simxGetJointPosition(client_id, basket_joint, sim.simx_opmode_oneshot)
        var = var[1]*180/math.pi
        if(var > 25):
            break
    
    ####################################################################
    return
    
def drop_in_cb2(client_id):
    
    """
    
    Purpose:
    ---
    Navigates the bot to collection box 2 and adjust its position.
    Then lifts the basket to drop the fruits in collection box2.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ########################### OUR CODE ###############################

    target_points = [(6,11)]
    task_3_primary(client_id, target_points)
    min_max(client_id,0,100,150)
    min_max(client_id,1,330,360)
    
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'liftRightBasket', [], [], [], emptybuff,sim.simx_opmode_blocking)
    _, basket_joint = sim.simxGetObjectHandle(client_id, 'basket_rj_r2', sim.simx_opmode_blocking)
    
    while True:
    
        var=sim.simxGetJointPosition(client_id, basket_joint, sim.simx_opmode_oneshot)
        var = var[1]*180/math.pi
    
        if(var < -75):
            return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',sim.sim_scripttype_childscript,'downRightBasket', [], [], [], emptybuff,sim.simx_opmode_blocking)           
            break
    
    while True:
    
        var=sim.simxGetJointPosition(client_id, basket_joint, sim.simx_opmode_oneshot)
        var = var[1]*180/math.pi
    
        if(var > -25):
            break
    
    ########################################################################
    return
    
def comeback_to_home_position(client_id):
    
    """
    
    Purpose:
    ---
    Makes the bot reach the home position if necessary.
    
    Arguments:
    ---
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ###################### OUR CODE ##########################
    
    target_points = [(4,4)]
    task_3_primary(client_id, target_points)
    
    ##########################################################
    return
    
def rotate_bot(client_id,direction):
    
    """
    Purpose:
    ---
    To rotate the bot for a particular direction

    Arguments:
    ---
    `direction`    :      0 -> turn clockwise 90deg, 
                          1 -> turn anticlockwise 90 deg 
                          
    Returns:
    ---
    None
    
    """

    ################################ OUR CODE ###############################
    
    prev_angle = get_bot_orientation(client_id)
    wheel_joints = init_setup(client_id)
    
    if(direction == 0):
        prev_wheel_data = encoders(client_id)
        set_bot_movement(client_id, wheel_joints, 0,0,3)
    
        while True:
            wheels_data = encoders(client_id)
            wd = abs(wheels_data[0] - prev_wheel_data[0])
            angle = get_bot_orientation(client_id)
            angle_rotated = abs(abs(prev_angle)-abs(angle))
            # print(angle_rotated, " wheels = ", wd)
    
            if(wd < 7):
                set_bot_movement(client_id, wheel_joints, 0,0,4)
    
            elif(wd> 7 and wd<9):
                set_bot_movement(client_id, wheel_joints, 0,0,2.5)
    
            elif(wd>9):
                set_bot_movement(client_id, wheel_joints, 0,0,0.5)
    
            if((angle_rotated < 2 or angle_rotated > 88.75) and wd>11):
                break
    
            if(wd > 12):
                break
    
    elif(direction == 1):    
        prev_wheel_data = encoders(client_id)
        set_bot_movement(client_id, wheel_joints, 0,0,-3)
    
        while True:
            wheels_data = encoders(client_id)
            wd = abs(wheels_data[0] - prev_wheel_data[0])
            angle = get_bot_orientation(client_id)
            angle_rotated = abs(abs(prev_angle)-abs(angle))
            # print(angle_rotated, " wheels = ", wd)
    
            if(wd < 7):
                set_bot_movement(client_id, wheel_joints, 0,0,-4)
    
            elif(wd> 7 and wd<9):
                set_bot_movement(client_id, wheel_joints, 0,0,-2.5)
    
            elif(wd>9):
                set_bot_movement(client_id, wheel_joints, 0,0,-0.5)
    
            if((angle_rotated < 1 or angle_rotated > 88.75) and wd>11):
                break 
    
            if(wd > 12):
                break
    
    wheel_velocity_setting(client_id, 0,0,0,0)
    
    ##################################################################
    return

def come_near_to_homeposition(client_id,rooms_entry,room_no,rooms_to_go):
    
    """
    
    Purpose:
    ---
    Stopping coordinate for the bot before moving to next room.
    
    Arguments:
    ---
    `client_id`      :      [ integer ]
        remote api connection
    `room_no`        :      [ integer ]
        room number which is previously traversed by bot
    `rooms_entry`    :      [ list ]
        room entry coordinate
    `rooms_to_go`    :      [ list ]
        order of rooms to be traversed
    Returns:
    ---
    None
    
    """
    
    ################################ OUR CODE ###############################
    
    a, b, c, d = [4,3], [3,4], [4,5],[5,4]
    rooms_rank_list = [[a, get_dist(client_id,rooms_entry[room_no - 1], a)],[b, get_dist(client_id,rooms_entry[room_no - 1], b)],[c, get_dist(client_id,rooms_entry[room_no - 1], c)],[d, get_dist(client_id,rooms_entry[room_no - 1], d)]]
    
    rooms_rank_list = sorted(rooms_rank_list, key = lambda x:x[1])
    
    temp = rooms_rank_list[0][0]    
    target_coord = []
    target_coord.append(temp)
    dist = get_dist(client_id, rooms_entry[room_no - 1], [4,4])
    next_room = 0
    i = 0
    
    for croom in rooms_to_go:
        
        if(room_no == croom):
            i=1
            continue
        
        if(i == 1):
            next_room = croom
            break
    
    next_room_entry = rooms_entry[next_room-1]
    current_room_entry = rooms_entry[room_no-1]
    distance = get_dist(client_id, current_room_entry, next_room_entry)
    
    if(distance == 2 or next_room_entry == (0,5) or next_room_entry == (8,3)):
        return
    
    task_3_primary(client_id, target_coord)
    
    ##########################################################################
    
    return
    
def theme_implementation_primary( client_id, rooms_entry):
    
    """
    Purpose:
    ---
    This is the only function that is called from the main function. Make sure to fill it
    properly, such that the bot completes the Theme Implementation.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()

    `rooms_entry`         :   [ list of tuples ]
        Room entry co-ordinate of each room in order.

    
    Returns:
    ---
    
    Example call:
    ---
    theme_implementation_primary(client_id, rooms_entry)
    
    """
    ######################### OUR CODE ##########################
    
    global B_cb1,B_cb2,L_cb1,L_cb2,S_cb1,S_cb2
    wheel_velocity_setting(client_id,0,0,0,0)
    
    # ------------- Decoding JSON file ------------- #
    json_file_data = open('Theme_Config.json')
    decoded_data = json.load(json_file_data)
    
    # ---------------Deciding rooms----------------- #
    target_berries, total_target_berries = berries_to_pick(client_id,decoded_data)
    rooms_to_go = decide_rooms(client_id,target_berries, total_target_berries, rooms_entry)    
    # ---------------------------------------------- #

    # ------------ go to decided rooms ------------- # 
    for room_no in rooms_to_go:
        
        if(room_no == 1):
            go_to_room1(client_id,rooms_entry[0])
        
        elif(room_no == 2):
            go_to_room2(client_id,rooms_entry[1])
        
        elif(room_no == 3):
            go_to_room3(client_id,rooms_entry[2])
        
        elif(room_no == 4):
            go_to_room4(client_id,rooms_entry[3])

        temp_var = check_targets(client_id)
        
        come_near_to_homeposition (client_id, rooms_entry, room_no, rooms_to_go )
        
        if(temp_var == 1 and (rooms_entry[room_no - 1][0] == 3 or rooms_entry[room_no - 1][0] == 5)):
            break
        
        come_near_to_homeposition (client_id, rooms_entry, room_no, rooms_to_go )
                
    # ---------- go to collection boxes ---------- #
    setVelocity(client_id, 12)
    if(berries_in_cb1 != 0):
        drop_in_cb1(client_id)      # goes near to collection box 1 to dump fruits in it
    time.sleep(3)
    
    if(berries_in_cb2 != 0):
        drop_in_cb2(client_id)      # goes near to collection box 2 to dump fruits in it
    wheel_velocity_setting(client_id,0,0,0,0)
    
    ############################################################
    
    return

if __name__ == "__main__":
    
    # Room entry co-ordinate
    # rooms_entry = [(3,6), (5,8), (6,3), (3,0)]
    # rooms_entry = [(2,5), (5,8), (5,2), (2,3)]     # example list of tuples
    # rooms_entry = [(2,5), (5,6), (5,2), (3,2)]
    rooms_entry = [(3,6), (6,5), (5,2), (3,2)]
    ###############################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        client_id = init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = start_simulation(client_id)

                if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
                    print('\nSimulation started correctly in CoppeliaSim.')

                else:
                    print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
                    print('start_simulation function is not configured correctly, check the code!')
                    print()
                    sys.exit()

            except Exception:
                print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!')
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
            print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        # Running student's logic
        theme_implementation_primary(client_id, rooms_entry)

        try:
            return_code = stop_simulation(client_id)                            
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
                        print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

                    else:
                        print('\n[ERROR] Failed disconnecting from Remote API server!')
                        print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

                except Exception:
                    print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()
                                      
            else:
                print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
                print('[ERROR] stop_simulation function is not configured correctly, check the code!')
                print('Stop the CoppeliaSim simulation manually.')
          
            print()
            sys.exit()

        except Exception:
            print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    except KeyboardInterrupt:
        print('\n[ERROR] Script interrupted by user!')