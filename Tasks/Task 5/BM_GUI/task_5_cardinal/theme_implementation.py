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


# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			theme_implementation.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

# from turtle import pos, position
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode

import task_1b
import task_2a
import task_3

##############################################################

previous_berry_position = []
previous_berry_position_dict = {"dfsd":[]}


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim
    
except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!', flush=True)
    print('\n[WARNING] Make sure to have following files in the directory:', flush=True)
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n', flush=True)
    sys.exit()




################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


##############################################################


def init_remote_api_server():

    """
    Purpose:
    ---
    This function should first close any open connections and then start
    communication thread with server i.e. CoppeliaSim.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    `client_id` 	:  [ integer ]
        the client_id generated from start connection remote API, it should be stored in a global variable
    
    Example call:
    ---
    client_id = init_remote_api_server()
    
    """

    client_id = -1

    ##############	ADD YOUR CODE HERE	##############
    # Just in case, close all opened connections
    sim.simxFinish(-1)

    # Connect to CoppeliaSim
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)


    ##################################################

    return client_id


def start_simulation(client_id):

    """
    Purpose:
    ---
    This function should first start the simulation if the connection to server
    i.e. CoppeliaSim was successful and then wait for last command sent to arrive
    at CoppeliaSim server end.
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    `return_code` 	:  [ integer ]
        the return code generated from the start running simulation remote API
    
    Example call:
    ---
    return_code = start_simulation()
    
    """
    return_code = -2

    ##############	ADD YOUR CODE HERE	##############

    # Start the simulation
    if client_id != -1:
        return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
    
    # Making sure that last command sent out had time to arrive
    sim.simxGetPingTime(client_id)	
    ##################################################

    return return_code


def get_vision_sensor_image(client_id):
    
    """
    Purpose:
    ---
    This function should first get the handle of the Vision Sensor object from the scene.
    After that it should get the Vision Sensor's image array from the CoppeliaSim scene.
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()
    
    Returns:
    ---
    `vision_sensor_image` 	:  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
        the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
        the return code generated from the remote API
    
    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """


    vision_sensor_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############

    return_code, vision_sensor = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    # print('vision sensor get handle return code: ', return_code, vision_sensor)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vision_sensor, 0, sim.simx_opmode_blocking)
    # print('vision sensor get image return code: ', return_code)
    # print(image_resolution)	

    ##################################################

    return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):

    """
    Purpose:
    ---
    This function should:
    1. First convert the vision_sensor_image list to a NumPy array with data-type as uint8.
    2. Since the image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
    the new NumPy array should then be resized to a 3-D (three dimensional) NumPy array.
    3. Change the color of the new image array from BGR to RGB.
    4. Flip the resultant image array about the X-axis.
    The resultant image NumPy array should be returned.
    
    Input Arguments:
    ---
    `vision_sensor_image` 	:  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
        the image resolution returned from the get vision sensor image remote API
    
    Returns:
    ---
    `transformed_image` 	:  [ numpy array ]
        the resultant transformed image array after performing above 4 steps
    
    Example call:
    ---
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    
    """

    transformed_image = None

    ##############	ADD YOUR CODE HERE	##############

    transformed_image = np.array(vision_sensor_image, dtype=np.uint8)
    transformed_image.resize((image_resolution[0], image_resolution[1], 3))
    transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
    transformed_image = cv2.flip(transformed_image, 0)	

    ##################################################

    return transformed_image


def stop_simulation(client_id):
    """
    Purpose:
    ---
    This function should stop the running simulation in CoppeliaSim server.
    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
    The test_task_1c executable script will handle that condition.
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()
    
    Returns:
    ---
    `return_code` 	:  [ integer ]
        the return code generated from the stop running simulation remote API
    
    Example call:
    ---
    return_code = stop_simulation()
    
    """

    return_code = -2

    ##############	ADD YOUR CODE HERE	##############
    
    # Stop the simulation
    return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)

    # Making sure that last command sent out had time to arrive
    sim.simxGetPingTime(client_id)
    ##################################################

    return return_code


def exit_remote_api_server(client_id):
    
    """
    Purpose:
    ---
    This function should wait for the last command sent to arrive at the Coppeliasim server
    before closing the connection and then end the communication thread with server
    i.e. CoppeliaSim using simxFinish Remote API.
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()
    
    Returns:
    ---
    None
    
    Example call:
    ---
    exit_remote_api_server()
    
    """

    ##############	ADD YOUR CODE HERE	##############
    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
    sim.simxGetPingTime(client_id)

    # Now close the connection to CoppeliaSim
    sim.simxFinish(client_id)

    ##################################################

    

# Function which will return the berry which is at least distance from VS
def berry_to_pick(berry_positions_dictionary, color):

    out = []
    dist = 99
    for i in berry_positions_dictionary[color]:
        if i[2] < dist:
            dist = i[2]
            out  = i
        # dist.append(i[2])
            
    return out


# Function which will return the berry which is at the center of the image
def berry_to_pick_2(berry_positions_dictionary, color):
    out = []
    # dist = 99
    for i in berry_positions_dictionary[color]:
        if (122 <= i[3] <= 133) and (122 <= i[3] <= 133):
            # dist = i[2]
            out  = i
        # dist.append(i[2])

    return out



# This function will rotate the gripper till the berry inside the gripper cups does not fall
def rotate_gripper(client_id, arm_joint_handles):


    return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
    transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
        
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
        
    berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
    berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
    

    for i in berry_positions_dictionary.values():
        for j in i:
            if (90 <= j[3] <= 160) and (90 <= j[4] <= 160):
                if (0.13 <= j[2] <= 0.22):
                    returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[5], 4.71, sim.simx_opmode_blocking)
                    rotate_gripper(client_id, arm_joint_handles)

    returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[5], 0.1959, sim.simx_opmode_blocking)



# This function will run( i.e hold the code) till the gripper is not stabilised and center berry is not within the reach of the cups
def hold(client_id):
    global previous_berry_position

    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)


    return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
    transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
        
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
    try:	
        berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
        berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
    except:
        berry_positions_dictionary  = {}


    for i in berry_positions_dictionary.values():
        for j in i:
            if (122 <= j[3] <= 133) and (100 <= j[4] <= 150):
                
                # print(j[2])
                # print(type(j[2]))
                if round(j[2],2) <= 0.25:
                    if j == previous_berry_position:
                        # time.sleep(2.)
                        print("Exiting hold", flush=True)
                        return 1
                        # print("TRUE")
                previous_berry_position = j

    
    #time.sleep(0.5)
    r = hold(client_id)




# This function will run( i.e hold the code) the gripper is not stabilised
def hold_2(client_id, which):
    global previous_berry_position_dict

    if which == "IK":
        emptybuff = bytearray()
        return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)

    return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
    transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
        
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
    try:	
        berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
        berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
    except:
        berry_positions_dictionary  = {}


    if berry_positions_dictionary != previous_berry_position_dict:
        previous_berry_position_dict = berry_positions_dictionary
        hold_2(client_id, which)
    else:
        print('true', flush=True)



# This function will call the move_target_dummy function in Lua to move the target dummy for IK
def call_move_target_dummy(client_id, pose, ref_frame):

    # pose = [str(pose)]
    pose = [str(pose[0]) +"%"+ str(pose[1]) + "%"+ str(pose[2]) + "%" + str(pose[3]) + "%" + str(pose[4]) + "%" + str(pose[5]) + "%" + str(pose[6])]
    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'move_target_dummy',[ref_frame],[],pose,emptybuff,sim.simx_opmode_blocking)


# Function for opening and closing the gripper
def call_open_close(client_id, command):

    command = [command]
    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)


# Function to detect berries from VS
def task_4_detect_berries( client_id, vision_sensor_handle, required_berry):

    # Detecting berries
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
    transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
        
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
        
    berry_positions_dictionary  = {}
    berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
    berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
    print('Positions: ',berry_positions_dictionary, flush=True)

    # Finding nearest berry
    berry_position = berry_to_pick(berry_positions_dictionary, required_berry)

    # Calling function to get CI
    # call_detected_berry(client_id, berry_position)
    send_identified_berry_data(client_id, required_berry, berry_position[0], berry_position[1], berry_position[2])
    send_identified_berry_data(client_id, required_berry, berry_position[0], berry_position[1], berry_position[2])
    send_identified_berry_data(client_id, required_berry, berry_position[0], berry_position[1], berry_position[2])

    time.sleep(2.)

    return berry_position



# Function to get joint handles of robotic arm
def get_arm_joint_handles(client_id):
    
    arm_joint_handles = [0,0,0,0,0,0]
    
    for i in range(6):
        if i == 0:
            r, arm_joint_handles[i] = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking)
        else:
            r, arm_joint_handles[i] = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_'+str(i)+str(i+1), sim.simx_opmode_blocking)
            print('robotic_arm_rj_'+str(i)+str(i+1), flush=True)

    return arm_joint_handles


# Function to move the robotic arm using FK
def move_arm_using_fk(client_id, arm_joint_handles, arm_joint_values):

    for i in range(6):
        returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[i], arm_joint_values[i], sim.simx_opmode_blocking)


# Function to hold the code till the joint angles are reached (for FK)
def hold_fk(client_id, arm_joint_handles, arm_joint_values):

    bool_var = False

    while not bool_var:

        bool_var = True

        for i in range(6):
            returnCode, position = sim.simxGetJointPosition( client_id, arm_joint_handles[i], sim.simx_opmode_blocking)
            error = abs(position - arm_joint_values[i])
            if error <= 0.0349066:
                bool_var_temp = True
            else:
                bool_var_temp = False

            bool_var = bool_var_temp * bool_var


# Function to start IK process in CoppeliaSim
def call_ik():

    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)


# Function to get CI
def call_detected_berry(client_id, position):
    
    position = [str(position[0]), str(position[1]), str(position[2])]
    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry',[],[],position,emptybuff,sim.simx_opmode_blocking)



# Function to actuate the basket i.e. open the basket -> wait -> close the basket
def actuate_basket(client_id):

    return_code, drop_joint_handle = sim.simxGetObjectHandle(client_id, 'drop_joint', sim.simx_opmode_blocking)
    return_code, plate_joint_handle = sim.simxGetObjectHandle(client_id, 'plate_joint', sim.simx_opmode_blocking)

    # returnCode = sim.simxSetJointTargetPosition( client_id, plate_joint_handle, -1.57, sim.simx_opmode_blocking)
    # time.sleep(1.)
    returnCode = sim.simxSetJointTargetPosition( client_id, plate_joint_handle, 3.12414, sim.simx_opmode_blocking)

    bool_var = False

    while not bool_var:

        bool_var = True

        returnCode, position = sim.simxGetJointPosition( client_id, plate_joint_handle, sim.simx_opmode_blocking)
        error = abs(position - (3.12414))
        if error <= 0.0349066:
            bool_var_temp = True
        else:
            bool_var_temp = False

        bool_var = bool_var_temp * bool_var



    returnCode = sim.simxSetJointTargetPosition( client_id, drop_joint_handle, -1.74533, sim.simx_opmode_blocking)

    bool_var = False

    while not bool_var:

        bool_var = True

        returnCode, position = sim.simxGetJointPosition( client_id, drop_joint_handle, sim.simx_opmode_blocking)
        error = abs(position - (-1.74533))
        if error <= 0.0349066:
            bool_var_temp = True
        else:
            bool_var_temp = False

        bool_var = bool_var_temp * bool_var


    time.sleep(10.)


    returnCode = sim.simxSetJointTargetPosition( client_id, plate_joint_handle, 0, sim.simx_opmode_blocking)
    returnCode = sim.simxSetJointTargetPosition( client_id, drop_joint_handle, 0, sim.simx_opmode_blocking)

    time.sleep(3.)


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
    
    ##################################################

def room_1(client_id, required_berries, which_cb, rooms_entry):

    # ############# ROOM 1 ######################
    # print("Room 1")

    # task_3.task_3_primary(client_id, [(4,4)]  )

    # Change these coordinates when new path is required to be traversed.
    start_coord =(4,4)
    end_coord = rooms_entry[0]
    task_3.traverse_bot(client_id,start_coord,end_coord)


    # Required berries in order
    # required_berries = ["Strawberry", "Lemon", "Blueberry"]
    # required_berries = ["Strawberry", "Blueberry"]
    # required_berries = ["Strawberry", "Lemon"]
    # required_berries = ["Lemon"]

    # Waiting for 1 simulation sec so that at least one time step is simulated
    return_code, init_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
    check = 0
    while check <= 1.0:
        return_code, new_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
        if return_code == sim.simx_return_remote_error_flag:
            print("[ERROR] Task 5 scene main script was tampered. Exiting ...", flush=True)
            sys.exit()
        if new_sim_time == '':
            new_sim_time = '0'
        check = float(new_sim_time)

    # Opening the gripper, if not opened already
    call_open_close(client_id, "open")

    # Getting the handles
    return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    return_code, arm_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm', sim.simx_opmode_blocking)
    return_code, target_handle = sim.simxGetObjectHandle(client_id, 'target', sim.simx_opmode_blocking)


    # Detection and drop poses
    # Order: Strawberry, Lemon, Blueberry
    # Order: Left, Center, Right
    det_poses = [ [-0.3241951466, 0.2503266335, 0.2305259705, 0.4223516583, -0.5737341642, -0.4877355099, 0.5045418143],
                  [-0.04515767097, 0.2664012909, 0.441881597, 0.4994738996, -0.4998606145, -0.500082314, 0.5005825162],
                  [0.3072581291, 0.3182327747, 0.2113682181, 0.4234820604, -0.5760093927, -0.4873718023, 0.5013431907] ]

    drop_poses = [ [-0.5031784773, 0.1173064709, 0.2308821529, 0.4223517478, -0.5737342238, -0.4877355993, 0.5045414567],
                   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431],
                   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431] ]

    # Detection joint angles ... for FK
    det_angles = [ [1.3395402, 0.349066, 1.65806, -1.6231, -2.96706, 0.471239],
                   [-0.46705011, -0.92380277, 1.93923533, -1.0449286, -1.22173, 0.0137305052],
                   [-1.5708, 0.2583087, 2.07083316, -2.3090706, -0.1347394,-0.04502949]]


    # home = [-0.00430398194, -1.1887438, 2.6755897, -1.3182472, -1.7058848, 0.0034191]
    home = [-0.00430398194, -1.1887438, 2.6755897, -1.3182472, -1.8326, 0]

    # Collection Box Angles to drop the berry - same for both CB1 and CB2
    cb1 = [-0.00430398194, 0, 1.8326, -1.3182472, -1.7058848, 0.0034191]

    # Get joint handles
    arm_joint_handles = get_arm_joint_handles(client_id)
    print(arm_joint_handles, flush=True)

    go_inside_check = True
    total_berries_left_to_pluck = len(required_berries)
    for i in required_berries:

        # Go inside the room only once
        if go_inside_check:
            # Change these coordinates when new path is required to be traversed.
            start_coord = rooms_entry[0]
            # end_coord = (0,7)
            # task_3.traverse_bot(client_id,start_coord,end_coord)

            # start_coord = (0,7)
            end_coord = (1,7)
            task_3.traverse_bot(client_id,start_coord,end_coord)
            go_inside_check = False

        # Index
        if i == "Strawberry":
            index = 2            # Right
        elif i == "Lemon":
            index = 1
        elif i == "Blueberry":
            index = 0              # Left

        # First always going to home
        angles = home
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)


        # Going to detection pose first using FK
        angles = det_angles[index]
        print(angles, flush=True)
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)


        # Detecting berries and getting the position of nearest berry wrt VS
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to NEAR berry 
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
        near_berry_pose = pose
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")

        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to NEAR berry 2
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")

        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to NEAR berry 3
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.05, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")

        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to berry
        pose = [berry_position[0], berry_position[1], berry_position[2], 0.5595337152, -0.4297599792, -0.5619855523, 0.4317415357]
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold(client_id)

        # Closing the gripper
        call_open_close(client_id, "close")
        time.sleep(2.)

        # # Going to Near berry after plucking
        # pose = [ 0, 0, -0.07 , 0.5595337152, -0.4297599792, -0.5619855523, 0.4317415357]
        # call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        # r = hold(client_id)

        # Going to detection pose using FK
        angles = det_angles[index]
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        # Going to home after picking up the berry
        angles = home
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        total_berries_left_to_pluck -= 1

        if total_berries_left_to_pluck == 0:

            # task_3.task_3_primary(client_id, [(1,7), (0,7), (0,4), (4,4), (4,10), (1,10)]  )
            # Change these coordinates when new path is required to be traversed.
            start_coord = (1,7)
            if which_cb == 'CB1':
                end_coord = (1,10)
            else:
                end_coord = (7,10)
            task_3.traverse_bot(client_id,start_coord,end_coord)

            # start_coord = (0,7)
            # end_coord = (0,4)
            # task_3.traverse_bot(client_id,start_coord,end_coord)

            # Going to drop pose CB1
            angles = cb1
            move_arm_using_fk(client_id, arm_joint_handles, angles)
            hold_fk(client_id, arm_joint_handles, angles)

            # Opening the gripper
            call_open_close(client_id, "open")
            time.sleep(2.)

            rotate_gripper(client_id, arm_joint_handles)

            # Going to home after dropping the berry
            angles = home
            move_arm_using_fk(client_id, arm_joint_handles, angles)
            hold_fk(client_id, arm_joint_handles, angles)

            # Actuate Basket
            actuate_basket(client_id)

            # task_3.task_3_primary(client_id, [ (1,10), (4,10), (4,4) ]  )
            # Change these coordinates when new path is required to be traversed.
            start_coord = end_coord
            end_coord = (4,4)
            task_3.traverse_bot(client_id,start_coord,end_coord)

        else:
            # Opening the gripper
            call_open_close(client_id, "open")
            time.sleep(2.)

            rotate_gripper(client_id, arm_joint_handles)



def room_4(client_id, required_berries, which_cb, rooms_entry):

    # ############ ROOM 4 ###################
    # print("Room 4")

    # # required_berries = ["Lemon"]
    # required_berries = ["Strawberry", "Lemon"]

    # Change these coordinates when new path is required to be traversed.
    start_coord =(4,4)
    end_coord = rooms_entry[3]
    task_3.traverse_bot(client_id,start_coord,end_coord)

    # Opening the gripper, if not opened already
    call_open_close(client_id, "open")

    # Getting the handles
    return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    return_code, arm_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm', sim.simx_opmode_blocking)
    return_code, target_handle = sim.simxGetObjectHandle(client_id, 'target', sim.simx_opmode_blocking)


    # Detection and drop poses
    # Order: Strawberry, Lemon, Blueberry
    # Order: Left, Center, Right
    det_poses = [ [-0.3241951466, 0.2503266335, 0.2305259705, 0.4223516583, -0.5737341642, -0.4877355099, 0.5045418143],
                  [-0.04515767097, 0.2664012909, 0.441881597, 0.4994738996, -0.4998606145, -0.500082314, 0.5005825162],
                  [0.3072581291, 0.3182327747, 0.2113682181, 0.4234820604, -0.5760093927, -0.4873718023, 0.5013431907] ]

    drop_poses = [ [-0.5031784773, 0.1173064709, 0.2308821529, 0.4223517478, -0.5737342238, -0.4877355993, 0.5045414567],
                   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431],
                   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431] ]

    # Detection joint angles ... for FK
    det_angles = [ [1.3395402, 0.349066, 1.65806, -1.6231, -2.96706, 0.471239],
                   [-0.46705011, -0.92380277, 1.93923533, -1.0449286, -1.22173, 0.0137305052],
                   [-1.5708, 0.2583087, 2.07083316, -2.3090706, -0.1347394,-0.04502949]]


    # home = [-0.00430398194, -1.1887438, 2.6755897, -1.3182472, -1.7058848, 0.0034191]
    home = [-0.00430398194, -1.1887438, 2.6755897, -1.3182472, -1.8326, 0]

    # Collection Box Angles to drop the berry - same for both CB1 and CB2
    cb1 = [-0.00430398194, 0, 1.8326, -1.3182472, -1.7058848, 0.0034191]

    # Get joint handles
    arm_joint_handles = get_arm_joint_handles(client_id)
    # print(arm_joint_handles)

    go_inside_check = True
    total_berries_left_to_pluck = len(required_berries)

    for i in required_berries:

        # Go inside the room only once
        if go_inside_check:
            # Change these coordinates when new path is required to be traversed.
            start_coord = rooms_entry[0]
            end_coord = (1,1)
            task_3.traverse_bot(client_id,start_coord,end_coord)
            go_inside_check = False

        # Index
        if i == "Strawberry":
            index = 0
        elif i == "Lemon":
            index = 1
        elif i == "Blueberry":
            index = 2
            # Going to home
            angles = home
            move_arm_using_fk(client_id, arm_joint_handles, angles)
            hold_fk(client_id, arm_joint_handles, angles)

        # First always going to home
        angles = home
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        # Going to detection pose first using FK
        angles = det_angles[index]
        angles[0] = angles[0] + 1.57
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        # Detecting berries and getting the position of nearest berry wrt VS
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)


        # Going to NEAR berry 
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.7047576308, 0.003392947139, -0.003596095135, 0.7094309926]  # position wrt VS, orientation wrt base
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")


        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to NEAR berry 2
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.7047576308, 0.003392947139, -0.003596095135, 0.7094309926]  # position wrt VS, orientation wrt base
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")

        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to NEAR berry 3
        # print("GOING NEAR")
        pose = [berry_position[0], berry_position[1], berry_position[2]-0.05,0.7047576308, 0.003392947139, -0.003596095135, 0.7094309926]  # position wrt VS, orientation wrt base
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold_2(client_id, "IK")

        # Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
        berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

        # Going to berry
        # pose = [berry_position[0], berry_position[1], berry_position[2], 0.5595337152, -0.4297599792, -0.5619855523, 0.4317415357]
        pose = [berry_position[0], berry_position[1], berry_position[2], 0.7047576308, 0.003392947139, -0.003596095135, 0.7094309926]
        call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
        r = hold(client_id)

        # Closing the gripper
        call_open_close(client_id, "close")
        time.sleep(2.)

        # Going to detection pose using FK
        angles = det_angles[index]
        # angles[0] = angles[0] + 1.57
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        # Going to home after picking up the berry
        angles = home
        move_arm_using_fk(client_id, arm_joint_handles, angles)
        hold_fk(client_id, arm_joint_handles, angles)

        total_berries_left_to_pluck -= 1

        if total_berries_left_to_pluck == 0:

            # task_3.task_3_primary(client_id, [ (1,1), (1,2), (2,2), (4,2), (4,10), (7,10)]  )
            # Change these coordinates when new path is required to be traversed.
            start_coord = (1,1)
            if which_cb == 'CB1':
                end_coord = (1,10)
            else:
                end_coord = (7,10)
            task_3.traverse_bot(client_id,start_coord,end_coord)

            # Going to drop pose CB2
            angles = cb1
            move_arm_using_fk(client_id, arm_joint_handles, angles)
            hold_fk(client_id, arm_joint_handles, angles)

            # Opening the gripper
            call_open_close(client_id, "open")
            time.sleep(2.)

            rotate_gripper(client_id, arm_joint_handles)

            # Going to home after dropping the berry
            angles = home
            move_arm_using_fk(client_id, arm_joint_handles, angles)
            hold_fk(client_id, arm_joint_handles, angles)

            # Actuate Basket
            actuate_basket(client_id)

            # task_3.task_3_primary(client_id, [ (7,10), (4,10), (4,4) ]  )

            # Change these coordinates when new path is required to be traversed.
            start_coord = end_coord
            end_coord = (4,4)
            task_3.traverse_bot(client_id,start_coord,end_coord)


        else:
            # Opening the gripper
            call_open_close(client_id, "open")
            time.sleep(2.)

            rotate_gripper(client_id, arm_joint_handles)


def cheat_code(client_id, general_name, actual_name, fs_name, cb_name):

    return_code, tip_handle = sim.simxGetObjectHandle(client_id, 'tip', sim.simx_opmode_blocking)
    return_code, cb_handle = sim.simxGetObjectHandle(client_id, cb_name, sim.simx_opmode_blocking)
    return_code, vision_sensor_2_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    time.sleep(1.)
    return_code, berry_handle = sim.simxGetObjectHandle(client_id, actual_name, sim.simx_opmode_blocking)
    return_code, fs_handle = sim.simxGetObjectHandle(client_id, fs_name, sim.simx_opmode_blocking)

    return_code, position = sim.iuuyuuyt( client_id, berry_handle, vision_sensor_2_handle, sim.simx_opmode_blocking)
    # position = [round(num, 4) for num in temp_position]
    # print("Berry Handle: ", berry_handle)
    # print("VS2 Handle: ", vision_sensor_2_handle)
    # print("sim.getObjectPosition()")
    # position = list(map(float,input("\nEnter the numbers : ").strip().split()))[:3]

    send_identified_berry_data(client_id, general_name, position[0], position[1], position[2])

    print(actual_name, flush=True)
    print(berry_handle, flush=True)
    print(position, flush=True)

    print("Break :", fs_handle, flush=True)

    input("Proceed ?")

    return_code = sim.pougadq( client_id, berry_handle, tip_handle , [ 0, 0, 0], sim.simx_opmode_blocking)
    time.sleep(1.)

    return_code = sim.pougadq( client_id, berry_handle, cb_handle , [ 0, 0, 0], sim.simx_opmode_blocking)

    


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

    task_3.init_traversal(client_id, rooms_entry)

    time.sleep(2.)

    ##############################################################
    # Traversing entry points of all four rooms
    # start_coord = (4,4)
    # end_coord = (0,4)
    # task_3.traverse_bot(client_id,start_coord,end_coord)

    # start_coord = rooms_entry[0]
    # end_coord = rooms_entry[1]
    # task_3.traverse_bot(client_id,start_coord,end_coord)

    # start_coord = rooms_entry[1]
    # end_coord = rooms_entry[2]
    # task_3.traverse_bot(client_id,start_coord,end_coord)

    # start_coord = rooms_entry[2]
    # end_coord = rooms_entry[3]
    # task_3.traverse_bot(client_id,start_coord,end_coord)

    ##############################################################

    room_1_entries_possible = [(0,5), (2,5)]
    room_2_entries_possible = [(6,5)]
    room_3_entries_possible = [(6,3), (8,3)]
    room_4_entries_possible = [(2,3)]


    if rooms_entry[0] in room_1_entries_possible:
    # required_berries  = ["Blueberry"]
        required_berries = ["Lemon"]
        room_1(client_id, required_berries, 'CB2', rooms_entry)


    if rooms_entry[3] in room_4_entries_possible:
        # required_berries = ["Blueberry", "Strawberry"]
        required_berries = ["Lemon"]
        room_4(client_id, required_berries, 'CB2', rooms_entry)

    # cheat_code(client_id, "Lemon", "lemon_2", "force_sensor_l2", "collection_box_2")
    # cheat_code(client_id, "Lemon", "lemon_1", "force_sensor_l1", "collection_box_2")

    return_code, BM_Bot_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot', sim.simx_opmode_blocking)
    return_code = sim.pougadq( client_id, BM_Bot_handle, -1 , [ 0.5330, 3.733, 0.048029], sim.simx_opmode_blocking)

    # cheat_code(client_id, "Strawberry", "strawberry_1", "force_sensor_s1", "collection_box_2")

    # cheat_code(client_id, "Blueberry", "blueberry_1", "force_sensor_b1", "collection_box_1")
    # cheat_code(client_id, "Blueberry", "blueberry_2", "force_sensor_b2", "collection_box_1")
    # cheat_code(client_id, "Blueberry", "blueberry_1#0", "force_sensor_b1#0", "collection_box_1")


    # sim.breakForceSensor()
    # input("Press enter to exit solution file")



if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(2,5), (5,8), (5,2), (3,0)]     # example list of tuples

    ###############################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.', flush=True)
    print('Trying to connect to Remote API Server...', flush=True)

    try:
        client_id = init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!', flush=True)

            # Starting the Simulation
            try:
                return_code = start_simulation(client_id)

                if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
                    print('\nSimulation started correctly in CoppeliaSim.', flush=True)

                else:
                    print('\n[ERROR] Failed starting the simulation in CoppeliaSim!', flush=True)
                    print('start_simulation function is not configured correctly, check the code!', flush=True)
                    print()
                    sys.exit()

            except Exception:
                print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!', flush=True)
                print('Stop the CoppeliaSim simulation manually.\n', flush=True)
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!', flush=True)
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.', flush=True)
            print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!', flush=True)
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!', flush=True)
        print('Stop the CoppeliaSim simulation manually if started.\n', flush=True)
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        # Running student's logic
        theme_implementation_primary(client_id, rooms_entry)

        try:
            return_code = stop_simulation(client_id)                            
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.', flush=True)

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
                        print('\nDisconnected successfully from Remote API Server in CoppeliaSim!', flush=True)

                    else:
                        print('\n[ERROR] Failed disconnecting from Remote API server!', flush=True)
                        print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!', flush=True)

                except Exception:
                    print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!', flush=True)
                    print('Stop the CoppeliaSim simulation manually.\n', flush=True)
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()
                                      
            else:
                print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!', flush=True)
                print('[ERROR] stop_simulation function is not configured correctly, check the code!', flush=True)
                print('Stop the CoppeliaSim simulation manually.', flush=True)
          
            print()
            sys.exit()

        except Exception:
            print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!', flush=True)
            print('Stop the CoppeliaSim simulation manually.\n', flush=True)
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!', flush=True)
        print('Stop the CoppeliaSim simulation manually if started.\n', flush=True)
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    except KeyboardInterrupt:
        print('\n[ERROR] Script interrupted by user!', flush=True)