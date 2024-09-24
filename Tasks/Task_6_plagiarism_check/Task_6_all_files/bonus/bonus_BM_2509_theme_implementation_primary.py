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


# Team ID:			[ 2509 ]
# Author List:		[ Rahul Pandove,Anagh Benjwal,Saurav Kumar]
# Filename:			theme_implementation.py
# Functions:		call_open_close(client_id, command),start_ik(client_id,cd1, command),get_vision_sensor_image(client_id),
# 					transform_vision_sensor_image(vision_sensor_image, image_resolution),detect_qr_codes(transformed_image), 
# 					set_bot_movement(client_id,wheel_joints,forw_back_vel, left_right_vel, rot_vel), init_setup(client_id),encoders(client_id),nav_logic(list_pos),
# 					shortest_path(target_points,client_id),berry_dict(client_id),basket_op(client_id,command),arm_nav(client_id,wheel_joints,data_1,oh1,oh12,oh23),berry_info(data), nav_plan(rooms_entry),
# 					accuracy_imp(client_id,coor,wheel_joints),accuracy_imp(client_id,coor,wheel_joints),centroid_cal(img),angle_calculation(img)
# Global variables:
# 					[ previous_error,previous_time,integral_sum,sum_error,Kp,Kd,Ki,correction,where2go,i,previous_coordinate,last_pv,list_pos,prev_pos,prev_where2go,prev_direction,i,t,flag]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os
import sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()

from task_1b import *
from task_2a import *


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

previous_error = 0
previous_time = time.time()
integral_sum = 0
sum_error = 0
Kp = 1.5
Kd = 2
Ki = 1
correction = 0
where2go = 0
i = 0
t = 0
previous_coordinate = (0, 0)
last_pv = 0
list_pos = [0, 0, 0, 0]
prev_pos = [0, 0, 0, 0]
prev_where2go = 0
prev_direction = 0
flag = 0


def basket_op(client_id, command):
    """
    This function helps in rotation of basket for dropping mechanism

    Input:
    client_id:[integer]
    command:[string]

    Returns:
    ---
    `return_code`		:	[ integer ]"""
    command = [command]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'basket', sim.sim_scripttype_childscript, 'basket_op', [], [], command, emptybuff, sim.simx_opmode_blocking)


def call_open_close(client_id, command):
    """
    This function helps in open close mechanism of gripper

    Input:
    client_id:[integer]
    command:[string]

    Returns:
    ---
    `return_code`		:	[ integer ]"""

    command = [command]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [], command, emptybuff, sim.simx_opmode_blocking)


def start_ik(client_id, cd1, command):
    """
    This function helps in solving ik equations for navigation of robotic arm.

    Input:
    client_id:[integer]
    command:[string]

    Returns:
    ---
    `return_code`		:	[ integer ]"""

    command = [command]
    l = [cd1[0], cd1[1], cd1[2]]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'start_operation', [], l, command, emptybuff, sim.simx_opmode_blocking)


def berry_dict(client_id):
    """
    This function helps in getting 3-D location of berries.

    Input:
    client_id:[integer]

    Returns:
    ---
    `berry_positions_dictionary'		:	[ dictionary ]"""
# Get object handle of vision sensor
    return_code, vision_sensor_handle = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    # Get image array
    vision_sensor_image_1, image_resolution_1, return_code = get_vision_sensor_image2(
        client_id)
    # Transform Image array
    transformed_image = transform_vision_sensor_image1(
        vision_sensor_image_1, image_resolution_1)
    # Get depth image array
    vision_sensor_depth_image, image_resolution, return_code = get_vision_sensor_depth_image(
        client_id, vision_sensor_handle)
    # Transform depth image array
    transformed_depth_image = transform_vision_sensor_depth_image(
        vision_sensor_depth_image, image_resolution)
    # Get berry dictionary
    berries_dictionary = detect_berries(
        transformed_image, transformed_depth_image)
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)
    print(berry_positions_dictionary)
    # print(berry_positions_dictionary)
    return berry_positions_dictionary


def send_identified_berry_data(client_id, berry_name, x_coor, y_coor, depth):
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

    if(type(berry_name) != str):
        berry_name = str(berry_name)

    if(type(x_coor) != float):
        x_coor = float(x_coor)

    if(type(y_coor) != float):
        y_coor = float(y_coor)

    if(type(depth) != float):
        depth = float(depth)

    data_to_send = [berry_name, str(x_coor), str(y_coor), str(depth)]
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'eval_bm', sim.sim_scripttype_childscript, 'detected_berry_by_team', [], [], data_to_send, emptybuff, sim.simx_opmode_blocking)
    return return_code

    ##################################################


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
    return_code = 1

    ##############	ADD YOUR CODE HERE	##############
    return_code, vision_sensor1 = sim.simxGetObjectHandle(
            client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
            client_id, vision_sensor1, 0, sim.simx_opmode_streaming )
    
    while(return_code != 0):
        return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
            client_id, vision_sensor1, 0, sim.simx_opmode_buffer)
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
    np_vision_image = np.array(vision_sensor_image, dtype=np.uint8)
    np_vision_image = np.resize(
        np_vision_image, (image_resolution[1], image_resolution[0], 3))
    lab_image = cv2.cvtColor(np_vision_image, cv2.COLOR_BGR2RGB)
    transformed_image = cv2.flip(lab_image, 0)

    ##################################################

    return transformed_image


def detect_qr_codes(transformed_image):
    """
    Purpose:
    ---
    This function receives the transformed image from the vision sensor and detects qr codes in the image

    Input Arguments:
    ---
    `transformed_image` 	:  [ numpy array ]
            the transformed image array

    Returns:
    ---
    qr_codes:  [List of tuple]

    Example call:
    ---
    detect_qr_codes()

    """

    ##############	ADD YOUR CODE HERE	##############
    qr_codes = []
    qrCodes = decode(transformed_image)
    for qrcode in qrCodes:
        qrcodeData = qrcode.data.decode("utf-8")
        if(len(qrcodeData) == 6):
            qr_codes = (int(qrcodeData[1]), int(qrcodeData[4]))
        elif(len(qrcodeData) == 7):
            qr_codes = (int(qrcodeData[1]), int(qrcodeData[5])+10)

    ##################################################

    return qr_codes


def set_bot_movement(client_id, wheel_joints, forw_back_vel, left_right_vel, rot_vel):
    """
    Purpose:
    ---
    This function takes desired forward/back, left/right, rotational velocites of the bot as input arguments.
    It should then convert these desired velocities into individual joint velocities(4 joints) and actuate the joints
    accordingly.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints`      :   [ list]
            Python list containing joint object handles of individual joints

    `forw_back_vel'     :   [ float ]
            Desired forward/back velocity of the bot

    `left_right_vel'    :   [ float ]
            Desired left/back velocity of the bot

    `rot_vel'           :   [ float ]
            Desired rotational velocity of the bot

    Returns:
    ---
    None

    Example call:
    ---
    set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)

    """
    ##############	ADD YOUR CODE HERE	##############

    fl = wheel_joints[0]
    fr = wheel_joints[1]
    rl = wheel_joints[2]
    rr = wheel_joints[3]

    global correction
    global where2go

    # rot
    if where2go == 90 or where2go == -90 or where2go == -901 or where2go == 901 or where2go == 902 or where2go == 180 or where2go == -1801 or where2go == 1801:
        sim.simxSetJointTargetVelocity(
            client_id, fl, rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, -rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, -rot_vel, sim.simx_opmode_blocking)

    # up
    if where2go == 1 :

        sim.simxSetJointTargetVelocity(
            client_id, fl, forw_back_vel + correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, forw_back_vel - correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, forw_back_vel - correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, forw_back_vel + correction, sim.simx_opmode_blocking)
    if  where2go == 11:

        sim.simxSetJointTargetVelocity(
            client_id, fl, forw_back_vel + correction +rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, forw_back_vel - correction-rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, forw_back_vel - correction+rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, forw_back_vel + correction-rot_vel, sim.simx_opmode_blocking)

    # right
    if where2go == 2:
        sim.simxSetJointTargetVelocity(
            client_id, fl, correction + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, correction - left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, correction - left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, correction + left_right_vel, sim.simx_opmode_blocking)

    if where2go == 22 :
        sim.simxSetJointTargetVelocity(
            client_id, fl, correction + left_right_vel+ rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, correction - left_right_vel- rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, correction - left_right_vel+ rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, correction + left_right_vel- rot_vel, sim.simx_opmode_blocking)

    # up-right
    if where2go == 3 or where2go == 33:
        sim.simxSetJointTargetVelocity(
            client_id, fl, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)

    # Up-left
    if where2go == 4 or where2go == 44:
        sim.simxSetJointTargetVelocity(
            client_id, fl, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, correction, sim.simx_opmode_blocking)
    # Down
    if where2go == -1 :
        sim.simxSetJointTargetVelocity(
            client_id, fl, - forw_back_vel + correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, - forw_back_vel - correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, - forw_back_vel - correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, - forw_back_vel + correction, sim.simx_opmode_blocking)
    if where2go == -11:
        sim.simxSetJointTargetVelocity(
            client_id, fl, - forw_back_vel + correction+rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, - forw_back_vel - correction-rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, - forw_back_vel - correction+rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, - forw_back_vel + correction-rot_vel, sim.simx_opmode_blocking)
    # Left
    if where2go == -2 :
        sim.simxSetJointTargetVelocity(
            client_id, fl, -correction - left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, -correction + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, -correction + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, -correction - left_right_vel, sim.simx_opmode_blocking)

    if where2go == -22:
        sim.simxSetJointTargetVelocity(
            client_id, fl, -correction - left_right_vel+rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, -correction + left_right_vel-rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, -correction + left_right_vel+rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, -correction - left_right_vel-rot_vel, sim.simx_opmode_blocking)
    # down-left
    if where2go == -3 or where2go == -33:
        sim.simxSetJointTargetVelocity(
            client_id, fl, (- forw_back_vel - left_right_vel), sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, (- forw_back_vel - left_right_vel), sim.simx_opmode_blocking)
    # down-right
    if where2go == -4 or where2go == -44:
        sim.simxSetJointTargetVelocity(
            client_id, fl, correction, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, (- forw_back_vel - left_right_vel), sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, (- forw_back_vel - left_right_vel), sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, correction, sim.simx_opmode_blocking)
    if where2go == 5:
        sim.simxSetJointTargetVelocity(
            client_id, fl, rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, -rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, rot_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, -rot_vel, sim.simx_opmode_blocking)
    if where2go == 6:  # 3
        sim.simxSetJointTargetVelocity(
            client_id, fl, - forw_back_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, - forw_back_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, - forw_back_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, - forw_back_vel, sim.simx_opmode_blocking)
    if where2go == 7:  # 4
        sim.simxSetJointTargetVelocity(
            client_id, fl, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, 0, sim.simx_opmode_blocking)
    if where2go == 8:
        sim.simxSetJointTargetVelocity(
            client_id, fl, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, fr, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rl, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(
            client_id, rr, forw_back_vel + left_right_vel, sim.simx_opmode_blocking)

    ##################################################


def init_setup(client_id):
    """
    Purpose:
    ---
    This function will get the object handles of all the four joints in the bot, store them in a list
    and return the list

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    'wheel_joints`      :   [ list]
            Python list containing joint object handles of individual joints

    Example call:
    ---
    init setup(client_id)

    """
    ##############	ADD YOUR CODE HERE	##############

    return_code, handle_fl = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_fl', sim.simx_opmode_blocking)
    return_code, handle_fr = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
    return_code, handle_rl = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
    return_code, handle_rr = sim.simxGetObjectHandle(
        client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)

    wheel_joints = [handle_fl, handle_fr, handle_rl, handle_rr]

    ##################################################

    return wheel_joints


def encoders(client_id):
    """
    Purpose:
    ---
    This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
    and return a list which contains the total joint position of all joints    

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Returns:
    ---
    'joints_position`      :   [ list]
            Python list containing the total joint position of all joints

    Example call:
    ---
    encoders(client_id)

    """
    return_code, signal_value = sim.simxGetStringSignal(
        client_id, 'combined_joint_position', sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")

    for index, joint_val in enumerate(joints_position):
        joints_position[index] = float(joint_val)

    return joints_position


def nav_logic(list_pos):
    """
    Purpose:
    ---
    This function implements the PID controller using the difference in wheel encoder values as the error to minimize drift in the bot's movement. 
    
    Input Arguments:
    ---
    `list_pos`         :   [ list ]
            the encoder values of the wheel joints returned by encoders(client_id)

    Returns:
    ---
    'Output`      :   [ float]
            output of the PID controler

    Example call:
    ---
    nav_logic(list_pos)
    """

    global where2go
    global previous_time
    global previous_error
    global last_pv
    global sum_error
    global Kp, Kd, Ki
    current_time = time.time()
    time_step = current_time - previous_time
    process_variable = 0

    if where2go == 1 or where2go == 11:  # Forward motion
        Kp = 0.67
        Kd = 0.09
        Ki = 0.01
        process_variable = -0.27 + \
            (abs(list_pos[0]) + abs(list_pos[1])) - \
            (abs(list_pos[2]) + abs(list_pos[3]))
    elif where2go == -1 or where2go == -11:
        Kp = 0.05569
        Kd = 0.090
        Ki = 0.01
        process_variable = -0.14 + \
            (abs(list_pos[0]) + abs(list_pos[1])) - \
            (abs(list_pos[2]) + abs(list_pos[3]))
    elif where2go == 2 or where2go == 22:
        Kp = 0.05562
        Kd = 0.09
        Ki = 0.01
        process_variable = 1 + \
            (abs(list_pos[0]) + abs(list_pos[2])) - \
            (abs(list_pos[1]) + abs(list_pos[3]))
    elif where2go == -2 or where2go == -22:
        Kp = 0.05562
        Kd = 0.09
        Ki = 0.01
        process_variable = -1 + \
            (abs(list_pos[1]) + abs(list_pos[3])) - \
            (abs(list_pos[0]) + abs(list_pos[2]))

    elif where2go == 3 or where2go == 33:  # Diagonal motion
        Kp = 0.056
        Kd = 0.09
        Ki = 0.01
        process_variable = -0.271 + (abs(list_pos[0]) - abs(list_pos[2]))
    elif where2go == -3 or where2go == -33:
        Kp = 0.05569
        Kd = 0.09
        Ki = 0.01
        process_variable = -0.271+(abs(list_pos[0]) - abs(list_pos[2]))

    elif where2go == 4 or where2go == 44:
        Kp = 0.05562
        Kd = 0.09
        Ki = 0.01
        process_variable = (abs(list_pos[1]) - abs(list_pos[3]))
    elif where2go == -4 or where2go == -44:
        Kp = 0.05562
        Kd = 0.09
        Ki = 0.01
        process_variable = -0.07 + (abs(list_pos[1]) - abs(list_pos[3]))

    error = 0 - process_variable
    dInput = (process_variable - last_pv)/time_step
    sum_error += error
    proportional_term = Kp * (error)
    derivative_term = Kd * dInput
    integral_term = Ki * sum_error * time_step

    output = proportional_term - derivative_term

    previous_error = error
    last_pv = process_variable
    previous_time = current_time

    return output


def shortest_path(target_points, client_id):
    '''This functions takes client_id and target_points and returns the direction where
       bot needs to traverse to get to the desired coordinates.

       Input:
       ---

       This function takes input of target_points: [List] and client id: [Integer].

       The functions returns the following values: 1,-1,2,-2,3,-3,4,-4,5,6,90,-90,180,901,-901,-1801
   Returns:
       ---
       When the bot is in default orientation these values stand for:

        1    :  Move forward
       -1    :  Move backward
        2    :  Move Right
       -2    : Move Left
        3    : Move Upward-Right Diagonally
       -3    : Move Downward-Left Diagonally
        4    : Move Upward-Lefft Diagonally
       -4    : Move Downward-Right Diagonally
        5    : To stop and drop the last berries and program finishes
        6    : To stop and pick the berries
        90   : Rotate 90 degree clockwise
       -90   : Rotate 90 degree anti clockwise
        180  : Rotate 180 degree 
        901  : Rotate 90 degree clockwise,then stop and pick berries and then rotate
                       90 degree anti clockwise
       -901  : Rotate 90 degree anti clockwise,then stop and pick berries and then rotate
                       90 degree clockwise
       -1801 : Rotate 90 degree anti clockwise ,pick berries and then rotate 90 degree anti clockwise again
        1801 : Rotate 90 degree  clockwise clockwise ,pick berries and then rotate 90 degree clockwise again

            Example Call:

            ---

            shortest_path(target_points,client_id)

       '''
    global correction
    global list_pos
    global prev_pos
    global t
    global where2go
    global i
    global previous_coordinate
    global prev_direction

    # Get image from vision sensor transform it and then detect QR code from it
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
        client_id)
    transformed_image = transform_vision_sensor_image(
        vision_sensor_image, image_resolution)
    c = detect_qr_codes(transformed_image)

    wheel_joints = init_setup(client_id)
    # To get the shortest path directions when bot is in default orientation
    if((t % 4) == 0):
        # Check whether QR code is in frame or not
        if(len(c) > 1):
            # Store previous coordinates to give the direction to bot even when QR code is not in frame
            previous_coordinate = (c[0], c[1])

            '''Using coordinates decoded from QR codes below part of code
			   compares the present coordinates of bot with the target coordinates 
			   and therefore gives the direction accordingly'''

            if(c[0] < target_points[i][0] and c[1] < target_points[i][1]):
                '''Reset values returned by encoder to 0 whenever bot changes direction
                   for proper functioning of PID'''
                if(prev_direction != 3 and prev_direction != 33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                # Store the previous direction of traversal
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    prev_direction = 33
                    return 33
                else:
                    prev_direction = 3
                    return 3
            elif(c[0] == target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != 1 and prev_direction != 11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    prev_direction = 11
                    return 11
                else:
                    prev_direction = 1
                    return 1
            elif (c[0] < target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != 2 and prev_direction != 22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    # set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]-1):
                    prev_direction = 22
                    return 22
                else:
                    prev_direction = 2
                    return 2
            elif(c[0] > target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -3 and prev_direction != -33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    # set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    prev_direction = -33
                    return -33
                else:
                    prev_direction = -3
                    return -3
            elif(c[0] == target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -1 and prev_direction != -11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[1] != target_points[i][1]+1):
                    prev_direction = -11
                    return -11
                else:
                    prev_direction = -1
                    return -1
            elif (c[0] > target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != -2 and prev_direction != -22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]+1):
                    prev_direction = -22
                    return -22
                else:
                    prev_direction = -2
                    return -2
            elif(c[0] > target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != 4 and prev_direction != 44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    prev_direction = 44
                    return 44
                else:
                    prev_direction = 4
                    return 4
            elif(c[0] < target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -4 and prev_direction != -44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # print(coor)

                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    prev_direction = -44
                    return -44
                else:
                    prev_direction = -4
                    return -4

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 10):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # #print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:
                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.1)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.1)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                print(coor)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                basket_op(client_id, "palat_r")
                time.sleep(3)
                basket_op(client_id, "default_r")

                i += 1
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 10):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                # print(coor)
                accuracy_imp(client_id, coor, wheel_joints)
                where2go=1

                set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)
                time.sleep(0.7)
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)


                return 5
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == target_points[i+1][0]+2 and ((c[0] == 4 and c[1] == 8) or (c[0] == 4 and c[1] == 7) or (c[0] == 4 and c[1] == 6))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                # Use QR Code to improve accuracy of navigation
                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)


                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == target_points[i+1][0]-2 and ((c[0] == 4 and c[1] == 8) or (c[0] == 4 and c[1] == 7) or (c[0] == 4 and c[1] == 6))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)


                # Use QR Code to improve accuracy of navigation
                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 90

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == target_points[i+1][0]+2 and ((c[0] == 4 and c[1] == 0) or (c[0] == 4 and c[1] == 1) or (c[0] == 4 and c[1] == 2))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)


                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == target_points[i+1][0]-2 and ((c[0] == 4 and c[1] == 1) or (c[0] == 4 and c[1] == 2))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)


                i += 1
                return 90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 1):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -901

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 7):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 6
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 7):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 901
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 4 and c[1] == 11):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)


                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[1] == target_points[i+1][1]+2 and ((c[0] == 6 and c[1] == 4) or (c[0] == 7 and c[1] == 4) or (c[0] == 8 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 180

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 4 and c[1] == 4):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.1)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                # print(angle)
                i += 1
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 4 and c[1] == 10):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0

                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.9):
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.1):
                            where2go = 90
                            if((angle == 90 or angle == 0)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0]==2 and c[1]==6) or (c[0]==2 and c[1]==2))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:
                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:
                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                i += 1
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and  (c[0]==2 and c[1]==4)):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                i += 1
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1]):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:
                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:
                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                i += 1

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)
        # Below part of code works when QR code is not in frame and provides direction of navigation
        # on basis of prevoius scanned QR code.
        else:
            c = previous_coordinate
            if(int(c[0]) < target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return 33
                else:
                    return 3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return 11
                else:
                    return 1
            elif (int(c[0]) < target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1):
                    return 22
                else:
                    return 2
            elif(int(c[0]) > target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return -33
                else:
                    return -3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[1] != target_points[i][1]+1):
                    return -11
                else:
                    return -1
            elif (int(c[0]) > target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1):
                    return -22
                else:
                    return -2
            elif(int(c[0]) > target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return 44
                else:
                    return 4
            elif(int(c[0]) < target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return -44
                else:
                    return -4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
        # If the last target coordinate is reached the program terminates.
        if(i == len(target_points)):
            return 5
    # When orientation of bot is rotated 90 degrees clockwise wrt default orientation.
    if((t % 4) == 1):
        if(len(c) > 1):
            previous_coordinate = (c[0], c[1])
            if(c[0] < target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != 4 and prev_direction != 44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    prev_direction = 44
                    return 44
                else:
                    prev_direction = 4
                    return 4
            elif(c[0] == target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -2 and prev_direction != -22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    prev_direction = -22
                    return -22
                else:
                    prev_direction = -2
                    return -2
            elif (c[0] < target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != 1 and prev_direction != 11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 2
                if(c[0] != target_points[i][0]-1):
                    prev_direction = 11
                    return 11
                else:
                    prev_direction = 1
                    return 1
            elif(c[0] > target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -4 and prev_direction != -44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    prev_direction = -44
                    return -44

                else:
                    prev_direction = -4
                    return -4
            elif(c[0] == target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 2 and prev_direction != 22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[1] != target_points[i][1]+1):
                    prev_direction = 22
                    return 22
                else:
                    prev_direction = 2
                    return 2
            elif (c[0] > target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != -1 and prev_direction != -11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]+1):
                    prev_direction = -11
                    return -11
                else:
                    prev_direction = -1
                    return -1
            elif(c[0] > target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -3 and prev_direction != -33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    prev_direction = -33
                    return -33
                else:
                    prev_direction = -3
                    return -3
            elif(c[0] < target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 3 and prev_direction != 33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]

                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    prev_direction = 33
                    return 33
                else:
                    prev_direction = 3
                    return 3
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 7):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 6
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 7):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -901
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0] == 6 and c[1] == 4) or (c[0] == 7 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)


                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0] == 2 and c[1] == 4) or (c[0] == 1 and c[1] == 4) or (c[0] == 0 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 4 and c[1] == 9):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 1):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                # vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                #     client_id)
                # transformed_image = transform_vision_sensor_image(
                #     vision_sensor_image, image_resolution)
                # coor = centroid_cal(transformed_image)
                # accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 901
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1]):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
        else:
            c = previous_coordinate
            if(int(c[0]) < target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return 44
                else:
                    return 4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return -22
                else:
                    return -2
            elif (int(c[0]) < target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1):
                    return 11
                else:
                    return 1
            elif(int(c[0]) > target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return -44
                else:
                    return -4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[1] != target_points[i][1]+1):
                    return 22
                else:
                    return 2
            elif (int(c[0]) > target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1):
                    return -11
                else:
                    return -1
            elif(int(c[0]) > target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return -33
                else:
                    return -3
            elif(int(c[0]) < target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return 33
                else:
                    return 3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
        if(i == len(target_points)):
            return 5
    # When orientation of bot is 180 degrees rotated clockwise wrt default orientation.
    if((t % 4) == 2 or t == -2):
        if(len(c) > 1):
            previous_coordinate = (c[0], c[1])
            if(c[0] < target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -3 and prev_direction != -33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -3
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return -33
                else:
                    return -3
            elif(c[0] == target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -1 and prev_direction != -11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -1
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return -11
                else:
                    return -1
            elif (c[0] < target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != -2 and prev_direction != -22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -2
                if(c[0] != target_points[i][0]-1):
                    return -22
                else:
                    return -2
            elif(c[0] > target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 3 and prev_direction != 33):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 3
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return 33
                else:
                    return 3
            elif(c[0] == target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 1 and prev_direction != 11):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 1
                if(c[1] != target_points[i][1]+1):
                    return 11
                else:
                    return 1
            elif (c[0] > target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != 2 and prev_direction != 22):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 2
                if(c[0] != target_points[i][0]+1):
                    return 22
                else:
                    return 2
            elif(c[0] > target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -4 and prev_direction != -44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -4
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return -44
                else:
                    return -4
            elif(c[0] < target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 4 and prev_direction != 44):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 4
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return 44
                else:
                    return 4
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 1):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1

                return 6
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0] == 4 and c[1] == 2) or (c[0] == 4 and c[1] == 1) or (c[0] == 4 and c[1] == 0))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                i += 1
                return 180
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0] == 4 and c[1] == 6) or (c[0] == 4 and c[1] == 7))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -90
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and ((c[0] == 2 and c[1] == 4) or (c[0] == 0 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 180
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1]):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
        else:
            c = previous_coordinate
            if(int(c[0]) < target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return -33
                else:
                    return -3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return -11
                else:
                    return -1
            elif (int(c[0]) < target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1):
                    return -22
                else:
                    return -2
            elif(int(c[0]) > target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return 33
                else:
                    return 3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[1] != target_points[i][1]+1):
                    return 11
                else:
                    return 1
            elif (int(c[0]) > target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1):
                    return 22
                else:
                    return 2
            elif(int(c[0]) > target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return -44
                else:
                    return -4
            elif(int(c[0]) < target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return 44
                else:
                    return 4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
        if(i == len(target_points)):
            return 5
    # When orintation of bot is rotated 270 degrees clockwise wrt to default position.
    if(t == -1):
        if(len(c) > 1):
            previous_coordinate = (c[0], c[1])
            if(c[0] < target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != -4):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -4
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return -44
                else:
                    return -4
            elif(c[0] == target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != 2):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 2
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return 22
                else:
                    return 2
            elif (c[0] < target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != -1):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -1
                if(c[0] != target_points[i][0]-1):
                    return -11
                else:
                    return -1
            elif(c[0] > target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != 4):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 4
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return 44
                else:
                    return 4
            elif(c[0] == target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -2):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -2
                if(c[1] != target_points[i][1]+1):
                    return -22
                else:
                    return -2
            elif (c[0] > target_points[i][0] and c[1] == target_points[i][1]):
                if(prev_direction != 1):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 1
                if(c[0] != target_points[i][0]+1):
                    return 11
                else:
                    return 1
            elif(c[0] > target_points[i][0] and c[1] < target_points[i][1]):
                if(prev_direction != 3):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = 3
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return 33
                else:
                    return 3
            elif(c[0] < target_points[i][0] and c[1] > target_points[i][1]):
                if(prev_direction != -3):
                    prev_pos[0] = list_pos[0]
                    prev_pos[1] = list_pos[1]
                    prev_pos[2] = list_pos[2]
                    prev_pos[3] = list_pos[3]
                    list_pos[0] = 0
                    list_pos[1] = 0
                    list_pos[2] = 0
                    list_pos[3] = 0
                    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                    # vision_sensor_image, image_resolution, return_code=get_vision_sensor_image(client_id)
                    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    # coor=centroid_cal(transformed_image)
                    # accuracy_imp(client_id,coor,wheel_joints)
                    # #print(coor)
                else:
                    list_pos[0] -= prev_pos[0]
                    list_pos[1] -= prev_pos[1]
                    list_pos[2] -= prev_pos[2]
                    list_pos[3] -= prev_pos[3]
                prev_direction = -3
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return -33
                else:
                    return -3
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 1):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 6
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 1 and c[1] == 7):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return 1801
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[0] == 7 and c[1] == 1):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                # vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                #     client_id)
                # transformed_image = transform_vision_sensor_image(
                #     vision_sensor_image, image_resolution)
                # coor = centroid_cal(transformed_image)
                # accuracy_imp_2(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                return -901
            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[1] == target_points[i+1][1]+2 and ((c[0] == 6 and c[1] == 4) or (c[0] == 7 and c[1] == 4) or (c[0] == 8 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                # print("?")
                return -90

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1] and c[1] == target_points[i+1][1]-2 and ((c[0] == 2 and c[1] == 4) or (c[0] == 1 and c[1] == 4) or (c[0] == 0 and c[1] == 4))):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1
                # print("?")
                return 90

            elif(c[0] == target_points[i][0] and c[1] == target_points[i][1]):
                prev_pos[0] = list_pos[0]
                prev_pos[1] = list_pos[1]
                prev_pos[2] = list_pos[2]
                prev_pos[3] = list_pos[3]
                list_pos[0] = 0
                list_pos[1] = 0
                list_pos[2] = 0
                list_pos[3] = 0
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                angle = angle_calculation(transformed_image)
                # print(angle)

                if (angle != 0 and angle != 90):
                    if(angle > 85):
                        where2go = 90
                        ang = 0
                        while(angle < 89.5):
                            if((angle == 90 or angle == 0 or (angle > 0.5 and angle < 5))):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, -0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)
                    elif(angle < 5):
                        ang = 0
                        while(angle > 0.5):
                            where2go = 90
                            if((angle == 90 or angle == 0) or (angle < 89.5 and angle > 85)):
                                break
                            else:

                                set_bot_movement(
                                    client_id, wheel_joints, 0, 0, 0.15)
                                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                                    client_id)
                                transformed_image = transform_vision_sensor_image(
                                    vision_sensor_image, image_resolution)
                                angle = angle_calculation(transformed_image)
                                # print(angle)

                vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                    client_id)
                transformed_image = transform_vision_sensor_image(
                    vision_sensor_image, image_resolution)
                coor = centroid_cal(transformed_image)
                accuracy_imp(client_id, coor, wheel_joints)
                # print(coor)

                i += 1

        else:
            c = previous_coordinate
            if(int(c[0]) < target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]-1):
                    return -44
                else:
                    return -4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] == target_points[i][0] and c[1] != target_points[i][1]-1):
                    return 22
                else:
                    return 2
            elif (int(c[0]) < target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1):
                    return -11
                else:
                    return -1
            elif(int(c[0]) > target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]+1):
                    return 44
                else:
                    return 4
            elif(int(c[0]) == target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[1] != target_points[i][1]+1):
                    return -22
                else:
                    return -2
            elif (int(c[0]) > target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1):
                    return 11
                else:
                    return 1
            elif(int(c[0]) > target_points[i][0] and int(c[1]) < target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]+1 and c[1] != target_points[i][1]-1):
                    return 33
                else:
                    return 3
            elif(int(c[0]) < target_points[i][0] and int(c[1]) > target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
                if(c[0] != target_points[i][0]-1 and c[1] != target_points[i][1]+1):
                    return -33
                else:
                    return -3
            elif(int(c[0]) == target_points[i][0] and int(c[1]) == target_points[i][1]):
                list_pos[0] -= prev_pos[0]
                list_pos[1] -= prev_pos[1]
                list_pos[2] -= prev_pos[2]
                list_pos[3] -= prev_pos[3]
        if(i == len(target_points)):
            return 5


def arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23):
    ''' This function executes navigation of arm for picking and dropping of berries.
            It takes client_id: [Integer],wheel_joints: [List],data_1(Information about berries to pick):[Dictionary] and object
            handles of 3 joints of arm. 
            oh1  : Joint between base and link 1
            oh12 : Joint between link 1 and link 2
            oh23 : JOint between link 2 and link 3

            Working:Thw vision sensor detects the berry after which the arm navigates to desired location 
            and picks up the berry after which it navigates back to its default postion and then navigates to the basket
            and drops the berry into the basket and then again navigates back to its default position.This process repeats 
            untill required amount of berries are picked. 

            Example call:
            ---

            arm_nav(client_id,wheel_joints,data_1,oh1,oh12,oh23)'''

    global where2go

    # Detect position of berries
    berry_positions_dictionary = berry_dict(client_id)

    # Determine the number of berries to pick from the present room

    # Store the amount of each type of berry present in the room.
    a = len(berry_positions_dictionary["Strawberry"])
    b = len(berry_positions_dictionary["Lemon"])
    c = len(berry_positions_dictionary["Blueberry"])
    # c1=[1,2,3]

    # Calculate the number of each type of berries to picked from this particular room
    if(data_1["Strawberry"][0] > a):
        data_1["Strawberry"][0] -= a

    elif(data_1["Strawberry"][0] <= a):
        a = data_1["Strawberry"][0]
        data_1["Strawberry"][0] = 0

    if(data_1["Lemon"][0] > b):
        data_1["Lemon"][0] -= b

    elif(data_1["Lemon"][0] <= b):
        b = data_1["Lemon"][0]
        data_1["Lemon"][0] = 0

    if(data_1["Blueberry"][0] > c):
        data_1["Blueberry"][0] -= c

    elif(data_1["Blueberry"][0] <= c):
        c = data_1["Blueberry"][0]
        data_1["Blueberry"][0] = 0

    listi = ["Strawberry", "Lemon", "Blueberry"]  # Code for working of arm
    if(berry_positions_dictionary["Lemon"][0][0] < 0.20 and berry_positions_dictionary["Lemon"][0][0] > -0.20):
        listi[0], listi[1] = listi[1], listi[0]
    elif(berry_positions_dictionary["Blueberry"][0][0] < 0.20 and berry_positions_dictionary["Blueberry"][0][0] > -0.20):
        listi[0], listi[2] = listi[2], listi[0]
    for d in listi:
        r = 0  # Number of berries picked
        ni = 0  # Index of berry being picked in berry_positions_dictionary

        # For picking strawberries
        if(d == 'Strawberry'):

            rbc = 0  # Removed Berry Count
            while(r < a and ni < len(berry_positions_dictionary[d])):
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)

                # If berry is in a position where it is inaccessible then ignore it
                if((((berry_positions_dictionary[d][ni][0] >= 0.18 and berry_positions_dictionary[d][ni][0] <= 0.27) or (berry_positions_dictionary[d][ni][0] <= -0.18 and berry_positions_dictionary[d][r][0] >= -0.27)) and berry_positions_dictionary[d][ni][1]>-0.15)):
                    ni += 1
                    rbc += 1
                    if(len(berry_positions_dictionary[d])-rbc < a):
                        data_1[d][0] += 1
                    continue

                # Below block of code helps to neutralize drift that occurs while picking berries
                if(len(berry_positions_dictionary[d]) > 0):
                    if(berry_positions_dictionary[d][ni][0] <= 0.15 and berry_positions_dictionary[d][ni][0] >= -0.15):
                        posg = "_c"
                        where2go = 6
                        set_bot_movement(client_id, wheel_joints, -0.005, 0, 0)
                    elif(berry_positions_dictionary[d][ni][0] > 0 and berry_positions_dictionary[d][ni][0] < 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] >= 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.048, 0.048, 0)
                    elif(berry_positions_dictionary[d][ni][0] < 0 and berry_positions_dictionary[d][ni][0] > -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] <= -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.048, 0.048, 0)

                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                # Store the coordinates of berry
                x = berry_positions_dictionary[d][ni][0]
                z = berry_positions_dictionary[d][ni][1]
                y = berry_positions_dictionary[d][ni][2]
                cd1 = [x, y, z]

                send_identified_berry_data(client_id, d, x, z, y)

                r += 1
                ni += 1
                call_open_close(client_id, "open")
                start_ik(client_id, cd1, "pick")
                # Below block of code helps to know whenever the arm reaches desired position

                '''Store previous position of all three joints of the arm and whenever the present 
				and previous position of all three joints is same terminate the loop'''

                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)

                call_open_close(client_id, "close")
                time.sleep(0.8)

                start_ik(client_id, cd1, "default"+posg)

                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                start_ik(client_id, cd1, "drop"+data_1["Strawberry"][1])
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)

                call_open_close(client_id, "open")

                # start_ik(client_id,cd1,"default"+posg)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)

        # For picking blueberry
        elif(d == 'Blueberry'):
            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            rbc = 0
            while(r < c and ni < len(berry_positions_dictionary[d])):
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                if((((berry_positions_dictionary[d][ni][0] >= 0.18 and berry_positions_dictionary[d][ni][0] <= 0.26) or (berry_positions_dictionary[d][ni][0] <= -0.18 and berry_positions_dictionary[d][r][0] >= -0.26)) and berry_positions_dictionary[d][ni][1]>-0.15)):
                    ni += 1
                    rbc += 1
                    if(len(berry_positions_dictionary[d])-rbc < c):
                        data_1[d][0] += 1
                    continue
                if(len(berry_positions_dictionary[d]) > 0):
                    if(berry_positions_dictionary[d][ni][0] <= 0.15 and berry_positions_dictionary[d][ni][0] >= -0.15):
                        posg = "_c"
                        where2go = 6
                        set_bot_movement(client_id, wheel_joints, -0.005, 0, 0)
                    elif(berry_positions_dictionary[d][ni][0] > 0 and berry_positions_dictionary[d][ni][0] < 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] >= 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.056, 0.056, 0)
                    elif(berry_positions_dictionary[d][ni][0] < 0 and berry_positions_dictionary[d][ni][0] > -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] <= -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.056, 0.056, 0)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                x = berry_positions_dictionary[d][ni][0]
                z = berry_positions_dictionary[d][ni][1]
                y = berry_positions_dictionary[d][ni][2]
                cd1 = [x+0.005, y, z]
                send_identified_berry_data(client_id, d, x, z, y)
                r += 1
                ni += 1
                call_open_close(client_id, "open")
                start_ik(client_id, cd1, "pick")
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                call_open_close(client_id, "close")
                time.sleep(0.9)
                start_ik(client_id, cd1, "default"+posg)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                start_ik(client_id, cd1, "drop"+data_1["Blueberry"][1])
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                call_open_close(client_id, "open")
                # start_ik(client_id,cd1,"default"+posg)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)

        # For picking lemons
        else:
            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            rbc = 0
            while(r < b and ni < len(berry_positions_dictionary[d])):
                set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                if((((berry_positions_dictionary[d][ni][0] >= 0.18 and berry_positions_dictionary[d][ni][0] <= 0.28) or (berry_positions_dictionary[d][ni][0] <= -0.18 and berry_positions_dictionary[d][r][0] >= -0.28)) and berry_positions_dictionary[d][ni][1]>-0.15)):
                    ni += 1
                    rbc += 1
                    if(len(berry_positions_dictionary[d])-rbc < b):
                        data_1[d][0] += 1
                    continue
                if(len(berry_positions_dictionary[d]) > 0):
                    if(berry_positions_dictionary[d][ni][0] <= 0.15 and berry_positions_dictionary[d][ni][0] >= -0.15):
                        posg = "_c"
                        where2go = 6
                        set_bot_movement(client_id, wheel_joints, -0.005, 0, 0)
                    elif(berry_positions_dictionary[d][ni][0] > 0 and berry_positions_dictionary[d][ni][0] < 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] >= 0.45):
                        posg = "_r"
                        where2go = 8
                        set_bot_movement(
                            client_id, wheel_joints, 0.054, 0.054, 0)
                    elif(berry_positions_dictionary[d][ni][0] < 0 and berry_positions_dictionary[d][ni][0] > -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.028, 0.028, 0)
                    elif(berry_positions_dictionary[d][ni][0] <= -0.45):
                        posg = "_l"
                        where2go = 7
                        set_bot_movement(
                            client_id, wheel_joints, 0.048, 0.048, 0)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                x = berry_positions_dictionary[d][ni][0]
                z = berry_positions_dictionary[d][ni][1]
                y = berry_positions_dictionary[d][ni][2]
                cd1 = [x, y, z]
                send_identified_berry_data(client_id, d, x, z, y)
                r += 1
                ni += 1
                call_open_close(client_id, "open")
                start_ik(client_id, cd1, "pick")
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                call_open_close(client_id, "close")
                time.sleep(0.9)
                start_ik(client_id, cd1, "default"+posg)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                start_ik(client_id, cd1, "drop"+data_1["Lemon"][1])
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)
                call_open_close(client_id, "open")
                # start_ik(client_id,cd1,"default"+posg)
                prev_pos1 = 0
                prev_pos12 = 0
                prev_pos23 = 0
                pos1 = 1
                pos12 = 1
                pos23 = 1
                while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

                    returnCode = 1
                    returnCode1 = 1
                    returnCode2 = 1
                    prev_pos1 = pos1

                    while(returnCode != 0):
                        returnCode, pos1 = sim.simxGetJointPosition(
                            client_id, oh1, sim.simx_opmode_blocking)
                        pos1 = round(pos1, 4)
                    prev_pos12 = pos12
                    while(returnCode1 != 0):
                        returnCode1, pos12 = sim.simxGetJointPosition(
                            client_id, oh12, sim.simx_opmode_blocking)
                        pos12 = round(pos12, 4)

                    prev_pos23 = pos23
                    while(returnCode2 != 0):
                        returnCode2, pos23 = sim.simxGetJointPosition(
                            client_id, oh23, sim.simx_opmode_blocking)
                        pos23 = round(pos23, 4)

    # Arm returns back to  its starting position before the traversal begins again
    call_open_close(client_id, "close")
    start_ik(client_id, cd1, "ultra_default")
    prev_pos1 = 0
    prev_pos12 = 0
    prev_pos23 = 0
    pos1 = 1
    pos12 = 1
    pos23 = 1
    while(prev_pos1 != pos1 or prev_pos12 != pos12 or prev_pos23 != pos23):

        returnCode = 1
        returnCode1 = 1
        returnCode2 = 1
        prev_pos1 = pos1

        while(returnCode != 0):
            returnCode, pos1 = sim.simxGetJointPosition(
                client_id, oh1, sim.simx_opmode_blocking)
            pos1 = round(pos1, 4)
        prev_pos12 = pos12
        while(returnCode1 != 0):
            returnCode1, pos12 = sim.simxGetJointPosition(
                client_id, oh12, sim.simx_opmode_blocking)
            pos12 = round(pos12, 4)

        prev_pos23 = pos23
        while(returnCode2 != 0):
            returnCode2, pos23 = sim.simxGetJointPosition(
                client_id, oh23, sim.simx_opmode_blocking)
            pos23 = round(pos23, 4)

    return data_1


def angle_calculation(img):
    '''Angle calculation function helps to determine orientation of bot.
       Here with the help of OPENCV we detect the angle by which QR code is rotated 
       which therefore helps to determine orientation of bot.

       The outermost square of QR code is bounded by minimum rectangle to 
       calculate the angle of rotation.

       Input:
       ---
       img : [Numpy Array]

       Returns:
       ---
       angle: [float]

       Example call:
            ---

            angle_calculation(img)'''

    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 127, 255, 0)
    contours1, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_size = 0
    i = 0
    # Determine max length of square present in frame
    for contour in contours1:

        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        if(len(approx) == 4):
            x, y, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if(aspectRatio >= 0.90 and aspectRatio <= 1.10):
                max_size = max(w, max_size)
        i += 1
    j = 0
    cnt = []
    # Extract the contour of square with maximum length
    for contour in contours1:

        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        if(len(approx) == 4):
            x, y, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if(aspectRatio >= 0.90 and aspectRatio <= 1.10):
                if(w == max_size):
                    cv2.drawContours(img, contours1, j, (0, 255, 0), 3)
                    cnt = contour

        j += 1
    # Bound the extracted contour into minimum area rectangle
    rect = cv2.minAreaRect(cnt)
    angle = rect[2]
    return angle


def centroid_cal(img):
    '''
    This funtions helps to calculate Centroid of the outermost square of QR code.
    It takes image as input and returns coordinates of centroid of largest square in frame

    Input:
    ---
    img: [Numpy array]

    Output:
    ---
    coor:[List]
    Example call:
    ---
    centroid_cal(img)
    '''

    # Convert the image into grayscale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Apply threshold to it
    ret, thresh = cv2.threshold(img_gray, 127, 255, 0)
    # Extract the contours
    contours1, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_size = 0
    i = 0
    # Determine max length of square present in frame
    for contour in contours1:

        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        if(len(approx) == 4):
            x, y, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if(aspectRatio >= 0.90 and aspectRatio <= 1.10):
                max_size = max(w, max_size)
        i += 1
    j = 0
    cnt = []
    # Extract the contour of square with maximum length
    for contour in contours1:

        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        approx = cv2.approxPolyDP(
            contour, 0.01*cv2.arcLength(contour, True), True)
        if(len(approx) == 4):
            x, y, w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if(aspectRatio >= 0.90 and aspectRatio <= 1.10):
                if(w == max_size):
                    cv2.drawContours(img, contours1, j, (0, 255, 0), 3)
                    cnt = contour

        j += 1
    # Calculate the centroid of of the largest square present in frame
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    coor = [cx, cy]
    return coor


def accuracy_imp(client_id, coor, wheel_joints):
    ''' Accuracy improvement function uses coordinates of centroid of largest square in frame
            to recenter the bot and improve the accuracy of navigation.

            Input:
            ---
            It takes client_id: [Integer],coor(coordinates of centroid of largest square in frame): [List] and wheel joints:[List] as input.

            Returns;
            ---
            []
            Example call:
            ---
            accuracy_imp(client_id,coor,wheel_joints)'''

    global where2go
    global t

    coor1 = [1, 2]
    coor1[0] = coor[0]
    coor1[1] = coor[1]

    '''Below code block takes the coordinate of centroid of largest square present in the frame and  
	   ties to align it with center of frame by following the required direction with an error of +10 or -10'''

    if((coor1[0] < 200 or coor1[0] > 330) or (coor1[1] < 200 or coor1[1] > 330)):
        if((t % 4) == 0 or (t % 4 == 1) or t == -1 or (t % 4 == 2)):
            # print(coor1[0],coor1[1])
            if(coor1[0] > 296 and coor1[1] > 296):
                while(coor1[0] > 276 and coor1[1] > 276):
                    # print("1")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -4
                    set_bot_movement(client_id, wheel_joints, 0.7, 0.7, 0)
            if(coor1[0] < 216 and coor1[0] < 216):
                while(coor1[0] < 236 and coor1[1] < 236):
                    # print("2")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 4
                    set_bot_movement(client_id, wheel_joints, 0.7, 0.7, 0)
            if(coor1[0] > 296 and coor1[1] < 216):
                while(coor1[0] > 276 and coor1[1] < 236):
                    # print("3")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 3
                    set_bot_movement(client_id, wheel_joints, 0.7, 0.7, 0)
            if(coor1[0] < 216 and coor1[1] > 296):
                while(coor1[0] < 236 and coor1[1] > 276):
                    #print("Aya hu vroo")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -3
                    set_bot_movement(client_id, wheel_joints, 0.7, 0.7, 0)
            if(coor1[0] < 216 and (coor1[1] > 216 and coor1[1] < 296)):
                while(coor1[0] < 236 and (coor1[1] > 236 and coor1[1] < 276)):
                    # print("5")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -2
                    set_bot_movement(client_id, wheel_joints, 0.7, 0.7, 0)
            if(coor1[0] > 296 and (coor1[1] > 216 and coor1[1] < 296)):
                while(coor1[0] > 276 and (coor1[1] > 236 and coor1[1] < 276)):
                    # print("6")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 2
                    set_bot_movement(client_id, wheel_joints, 0.5, 0.5, 0)
            if(coor1[1] < 216 and (coor1[0] > 216 and coor1[0] < 296)):
                while(coor1[1] < 236 and (coor1[0] > 236 and coor1[0] < 276)):
                    # print("7")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 1
                    set_bot_movement(client_id, wheel_joints, 0.5, 0.5, 0)
            if(coor1[1] > 296 and (coor1[0] > 216 and coor1[0] < 296)):
                while(coor1[1] > 276 and (coor1[0] > 236 and coor1[0] < 276)):
                    # print("8")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -1
                    set_bot_movement(client_id, wheel_joints, 0.5, 0.5, 0)


def accuracy_imp_2(client_id, coor, wheel_joints):
    ''' Accuracy improvement function uses coordinates of centroid of largest square in frame
            to recenter the bot and improve the accuracy of navigation.

            This function is created for more precise correction.

            Input:
            ---
            It takes client_id: [Integer],coor(coordinates of centroid of largest square in frame): [List] and wheel joints:[List] as input.

            Returns;
            ---
            []

            Example call:
            ---
            accuracy_imp_2(client_id,coor,wheel_joints)'''

    global where2go
    global t

    coor1 = [1, 2]
    coor1[0] = coor[0]
    coor1[1] = coor[1]

    '''Below code block takes the coordinate of centroid of largest square present in the frame and  
	   ties to align it with center of frame by following the required direction with an error of +10 or -10'''
    print(coor1)
    if((coor1[0] < 246 or coor1[0] > 266) or (coor1[1] < 246 or coor1[1] > 266)):
        print(coor1)
        if((t % 4) == 0 or (t % 4 == 1) or t == -1 or (t % 4 == 2)):
            print(coor1[0], coor1[1])
            if(coor1[0] > 266 and coor1[1] > 266):
                while(coor1[0] > 266 and coor1[1] > 266):
                    # print("1")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -4
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[0] < 246 and coor1[0] < 246):
                while(coor1[0] < 246 and coor1[1] < 246):
                    # print("2")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 4
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[0] > 266 and coor1[1] < 246):
                while(coor1[0] > 266 and coor1[1] < 246):
                    # print("3")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 3
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[0] < 246 and coor1[1] > 266):
                while(coor1[0] < 246 and coor1[1] > 266):
                    #print("Aya hu vroo")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -3
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[0] < 246 and (coor1[1] > 246 and coor1[1] < 266)):
                while(coor1[0] < 246 and (coor1[1] > 246 and coor1[1] < 266)):
                    # print("5")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -2
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[0] > 266 and (coor1[1] > 246 and coor1[1] < 266)):
                while(coor1[0] > 266 and (coor1[1] > 246 and coor1[1] < 266)):
                    # print("6")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 2
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[1] < 246 and (coor1[0] > 246 and coor1[0] < 266)):
                while(coor1[1] < 246 and (coor1[0] > 246 and coor1[0] < 266)):
                    # print("7")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = 1
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)
            if(coor1[1] > 266 and (coor1[0] > 246 and coor1[0] < 266)):
                while(coor1[1] > 276 and (coor1[0] > 236 and coor1[0] < 276)):
                    # print("8")
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    coor1 = centroid_cal(transformed_image)
                    # print(coor1)
                    where2go = -1
                    set_bot_movement(client_id, wheel_joints, 0.4, 0.4, 0)


def nav_plan(rooms_entry):
    """This function creates a navigation plan for completion of task.
            It takes rooms_entry coordinates and according to the coordinates given
            returns a list of navigation plan which bot follows during the task.

            Input:
            ---
            rooms_entry: [List]

            Output:
            ---
            rooms_entry_f: [List]

            Example call:
            ---

            nav_plan(data)"""

    rooms_entry_f = [1, 2, 3, 4]
    rooms_entry_f[0] = rooms_entry[0]
    rooms_entry_f[1] = rooms_entry[1]
    rooms_entry_f[2] = rooms_entry[2]
    rooms_entry_f[3] = rooms_entry[3]
    k = 0
    j = 1
# For each room different navigation plans are created based to coordinates of entry
    for pos in rooms_entry:
        k += 1
        if(pos == (2, 5) or pos == (1, 5) or pos == (0, 5)):  # Room1
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(j+2, (1, 7))  # Coordinate to pick berries
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(j+5, (4, 4))  # Home position
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (3, 6) or pos == (3, 7)):  # Room1
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+2, (1, 7))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (5, 8) or pos == (5, 7) or pos == (5, 6)):  # Room2
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+2, (7, 7))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (6, 5) or pos == (7, 5)):  # Room2
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(j+2, (7, 7))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (3, 0) or pos == (3, 1) or pos == (3, 2)):  # Room3
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+2, (1, 1))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (2, 3) or pos == (1, 3) or pos == (0, 3)):  # Room3
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(j+2, (1, 1))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (6, 3) or pos == (7, 3) or pos == (8, 3)):  # Room4
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(j+2, (7, 1))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0], rooms_entry[k-1][1]-1))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0], rooms_entry[k-1][1]+1))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
        elif(pos == (5, 0) or pos == (5, 1) or pos == (5, 2)):  # Room4
            rooms_entry_f.insert(
                j-1, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+1, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+2, (7, 1))
            rooms_entry_f.insert(
                j+3, (rooms_entry[k-1][0]+1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(
                j+4, (rooms_entry[k-1][0]-1, rooms_entry[k-1][1]))
            rooms_entry_f.insert(j+5, (4, 4))
            rooms_entry_f.pop(j)
            j += 6
    # print(rooms_entry_f)
    return rooms_entry_f


def berry_info(data):
    '''This function simplifies dictionary given by json file.

    It takes the dictionary given by json file and returns the simplified dictionary. 

    Input:
    ---
    data: [Dictionary]

    Returns:
    ---
    data_1: [Dictionary]

    Example call:
    ---
    berry_info(data)'''

    data_1 = {}

    for k in data:
        if(k == "B"):
            if(data_1.get("Blueberry") == None):

                data_1.update({"Blueberry": [int(data[k][0]), data[k][4]]})
        if(k == "L"):
            if(data_1.get("Lemon") == None):

                data_1.update({"Lemon": [int(data[k][0]), data[k][4]]})
        if(k == "S"):
            if(data_1.get("Strawberry") == None):

                data_1.update({"Strawberry": [int(data[k][0]), data[k][4]]})
    return data_1


##############################################################


def theme_implementation_primary(client_id, rooms_entry):
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
    global where2go  # Variables tells the direction to follow
    global correction  # Correction given by PID
    global list_pos  # Returned by encoder
    global t  # Measure of orientation of bot
    global flag  # Variable used to know whether all berries are picked or not
    global i  # Variable used to iterate target points

    CB1=0 #Gives 1 if any berry has to be dropped in CB1
    CB2=0 #Gives 1 if any berry has to be dropped in CB2
    max_berry=0 #Maximum berries of 1 type to be dropped

    json_file = open('Theme_Config.json')
    data = json.load(json_file)

    # Get object handle of 3 joints of robotic arm
    _, oh1 = sim.simxGetObjectHandle(
        client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking)
    _, oh12 = sim.simxGetObjectHandle(
        client_id, 'robotic_arm_rj_12', sim.simx_opmode_blocking)
    _, oh23 = sim.simxGetObjectHandle(
        client_id, 'robotic_arm_rj_23', sim.simx_opmode_blocking)

    wheel_joints = init_setup(client_id)
    where2go = 90
    set_bot_movement(client_id, wheel_joints, 0, 0, 0)

    # Get berry information dictionary
    data_1 = berry_info(data)

    for d in data_1:
        if(data_1[d][1]=='1'):
            CB1=1
        elif(data_1[d][1]=='2'):
            CB2=1
        if(data_1[d][0]>max_berry):
            max_berry=data_1[d][0]

    # Change order of rooms depending on number of berries to be picked
    if(max_berry>6):
        rooms_entry[0], rooms_entry[2] = rooms_entry[2], rooms_entry[0]
        rooms_entry[0], rooms_entry[3] = rooms_entry[3], rooms_entry[0]
        rooms_entry[1], rooms_entry[3] = rooms_entry[3], rooms_entry[1]
    elif(max_berry>4):
        rooms_entry[0], rooms_entry[2] = rooms_entry[2], rooms_entry[0]
        rooms_entry[0], rooms_entry[3] = rooms_entry[3], rooms_entry[0]
        rooms_entry[1], rooms_entry[3] = rooms_entry[3], rooms_entry[1]
        rooms_entry[1], rooms_entry[2] = rooms_entry[2], rooms_entry[1]
        rooms_entry[2], rooms_entry[3] = rooms_entry[3], rooms_entry[2]



    rooms_entry.append((4, 4))

# Get navigation plan
    rooms_entry_f = nav_plan(rooms_entry)

    # List to store navigation coordinates after all berries have been picked
    drop_coor = []
    # print(rooms_entry_f)

    while True:
        # print(t)
        list_pos = encoders(client_id)
        if(flag == 0):
            where2go = shortest_path(rooms_entry_f, client_id)
        else:
            where2go = shortest_path(drop_coor, client_id)

        correction = nav_logic(list_pos)
        # print(where2go)
        # Rotate 90 degrees clockwise
        if where2go == 90:
            correction = 0
            angle = 91
            ang = 0  # count of iterations
            prev_angle = 0
            while(angle > 0.6 or angle < 0.01):

                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break

                else:

                    ang += 1
                    prev_angle = angle
                    # 3 levels of velocity have been used to get greater accuracy
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    elif(angle < 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)

                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t += 1  # Measure of orientation

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp(client_id, coor, wheel_joints)
            # print(coor)

        elif where2go == -90:
            correction = 0
            angle = 1
            ang = 0
            prev_angle = 0
            while(angle < 89.2 or angle > 89.9):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle > 85):
                    # print("*")
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle < 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -5.5)
                    elif(angle > 40 and angle < 80):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, -0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t -= 1
            if(t == -3):
                t = 1
            if(t == -2):
                t = 2
            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp(client_id, coor, wheel_joints)
            # print(coor)

        elif where2go == 901:
            correction = 0
            angle = 91
            ang = 0
            prev_angle = 0
            while(angle > 0.8 or angle < 0.01):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t += 1

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            time.sleep(0.3)
            data_1 = arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23)

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            where2go = 901
            angle = 1
            ang = 0
            prev_angle = 0
            while(angle < 89.2 or angle > 89.9):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle > 85):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle < 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -5.5)
                    elif(angle > 40 and angle < 80):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, -0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t -= 1
            set_bot_movement(client_id, wheel_joints, 0, 0, 0)

            # Checks if all the berries are picked or not
            # If all berries are picked it starts berry droping navigation else it continues the previous nav plan
            if(data_1["Blueberry"][0] == 0 and data_1["Strawberry"][0] == 0 and data_1["Lemon"][0] == 0):
                flag = 1
                p = i
                if((t % 4) == 1 or t == -1):
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                else:
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                i = 0

        elif where2go == -901:
            correction = 0
            angle = 1
            ang = 0
            prev_angle = 0
            while(angle < 89.2 or angle > 89.9):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle > 85):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle < 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -5.5)
                    elif(angle > 40 and angle < 80):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, -0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t -= 1

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)
            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            time.sleep(0.3)
            # Arm Navigation
            data_1 = arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23)

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            where2go = -901
            angle = 91
            ang = 0
            prev_angle = 0
            while(angle > 0.8 or angle < 0.01):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t += 1

            # Checks if all the berries are picked or not
            # If all berries are picked it starts berry droping navigation else it continues the previous nav plan
            if(data_1["Blueberry"][0] == 0 and data_1["Strawberry"][0] == 0 and data_1["Lemon"][0] == 0):
                flag = 1
                p = i
                if((t % 4) == 1 or t == -1):
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                else:
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]

                i = 0
        elif where2go == -1801:
            correction = 0

            angle = 1
            ang = 0
            prev_angle = 0
            while(angle < 89.2 or angle > 89.9):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle > 86.7):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle < 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -5.5)
                    elif(angle > 40 and angle < 80):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, -0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t += 1

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            time.sleep(0.3)
            # Arm Navigation
            data_1 = arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23)

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            angle = 1
            ang = 0
            prev_angle = 0
            while(angle < 89.2 or angle > 89.9):
                if((angle == 90 or angle == 0) and ang > 1 and prev_angle > 86.5):
                    #print("hue hue")
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle < 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -5.5)
                    elif(angle > 40 and angle < 80):
                        set_bot_movement(client_id, wheel_joints, 0, 0, -1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, -0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t += 1

            # Checks if all the berries are picked or not
            # If all berries are picked it starts berry droping navigation else it continues the previous nav plan
            if(data_1["Blueberry"][0] == 0 and data_1["Strawberry"][0] == 0 and data_1["Lemon"][0] == 0):
                flag = 1
                p = i
                if((t % 4) == 1 or t == -1):
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                else:
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]

                i = 0
        elif where2go == 1801:
            correction = 0
            angle = 91
            ang = 0
            prev_angle = 0
            while(angle > 0.8 or angle < 0.01):
                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break
                else:
                    ang += 1
                    prev_angle = angle
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t += 1

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            time.sleep(0.3)
            # Arm Navigation
            data_1 = arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23)

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            where2go = 1801
            angle = 91
            ang = 0
            prev_angle = 0
            while(angle > 0.8 or angle < 0.01):
                #print("aya hu")
                if((angle == 90 or angle == 0) and ang > 2 and prev_angle < 5):
                    break
                else:
                    #print("Yha bhi aya")
                    ang += 1
                    prev_angle = angle
                    if(angle > 40):
                        #print("Idhar bhi aa gya vroooo")
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    else:
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)
                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle)
            t += 1
            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            where2go = 3

            if(data_1["Blueberry"][0] == 0 and data_1["Strawberry"][0] == 0 and data_1["Lemon"][0] == 0):
                flag = 1
                p = i
                if((t % 4) == 1 or t == -1):
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                else:
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]

                i = 0

        elif where2go == 1:
            correction = 0
            if t == 1:
                set_bot_movement(client_id, wheel_joints, 3, 0, 0)
            else:
                set_bot_movement(client_id, wheel_joints, 3, 0, 0)

        elif where2go == 11:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 7, 0, -0.1)  # 4.3 #0.06
        elif where2go == 2:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 0, 3.5, 0)  # 1.65
        elif where2go == 22:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 0, 7, -0.12)  # 4.3 #-0.12
        elif where2go == 3:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 1.75, 1.75, 0)  # 1.55
        elif where2go == 33:
            set_bot_movement(client_id, wheel_joints, 1.5, 1.5, 0)
        elif where2go == 4:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 1.7, 1.7, 0)
        elif where2go == 44:
            set_bot_movement(client_id, wheel_joints, 2.5, 2.5, 0)
        elif where2go == -1:
            correction = 0
            if (t == 0 or t == 2):
                set_bot_movement(client_id, wheel_joints, 2.8, 0, 0)
            elif(t == -1 or t == 1):
                set_bot_movement(client_id, wheel_joints, 2.8, 0, 0)

        elif where2go == -11:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 7, 0, 0.08)  # 4.3 #0
        elif where2go == -2:
            correction = -correction
            correction = 0
            if t == 1:
                set_bot_movement(client_id, wheel_joints, 0, 3.3, 0)  # 1.7
            else:
                set_bot_movement(client_id, wheel_joints, 0, 3.3, 0)  # 2.1
        elif where2go == -22:
            correction = -correction
            correction = 0
            set_bot_movement(client_id, wheel_joints, 0, 7,0.12)  # 4.3 #0.12
        elif where2go == -3:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 1.7, 1.7, 0)
        elif where2go == -33:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 2.2, 2.2, 0)
        elif where2go == -4:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 1.7, 1.7, 0)
        elif where2go == -44:
            correction = 0
            set_bot_movement(client_id, wheel_joints, 2.5, 2.5, 0)
        elif where2go == 6:
            # set_bot_movement(client_id, wheel_joints, -0.006, 0, 0)
            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            set_bot_movement(client_id, wheel_joints, 0, 0, 0)
            time.sleep(0.3)

            # Arm navigation
            data_1 = arm_nav(client_id, wheel_joints, data_1, oh1, oh12, oh23)

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp_2(client_id, coor, wheel_joints)
            # print(coor)

            # Checks if all the berries are picked or not
            # If all berries are picked it starts berry droping navigation else it continues the previous nav plan
            if(data_1["Blueberry"][0] == 0 and data_1["Strawberry"][0] == 0 and data_1["Lemon"][0] == 0):
                flag = 1
                p = i
                if((t % 4) == 1 or t == -1):
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                else:
                    if(CB1==1 and CB2==0):
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (3, 10), (1, 10)]
                    else:
                        drop_coor = [rooms_entry_f[p], rooms_entry_f[p+1],
                                    (4, 9), (5, 10), (7, 10), (5, 10), (3, 10), (1, 10)]
                i = 0
        elif where2go == 180:
            angle = 91
            ang = 0  # count of iterations
            prev_angle = 0
            while(angle > 0.6 or angle < 0.01):

                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break

                else:

                    ang += 1
                    prev_angle = angle
                    # 3 levels of velocity have been used to get greater accuracy
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    elif(angle < 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)

                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t += 1  # Measure of orientation

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp(client_id, coor, wheel_joints)

            where2go = 180

            angle = 91
            ang = 0  # count of iterations
            prev_angle = 0
            while(angle > 0.6 or angle < 0.01):

                if((angle == 90 or angle == 0) and ang != 1 and prev_angle < 5):
                    break

                else:

                    ang += 1
                    prev_angle = angle
                    # 3 levels of velocity have been used to get greater accuracy
                    if(angle > 40):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 5.5)
                    elif(angle < 40 and angle > 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 1.5)
                    elif(angle < 10):
                        set_bot_movement(client_id, wheel_joints, 0, 0, 0.15)

                    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                        client_id)
                    transformed_image = transform_vision_sensor_image(
                        vision_sensor_image, image_resolution)
                    angle = angle_calculation(transformed_image)
                    # print(angle,prev_angle)
            t += 1  # Measure of orientation

            vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(
                client_id)
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution)
            coor = centroid_cal(transformed_image)
            accuracy_imp(client_id, coor, wheel_joints)

        if where2go == 5:

            basket_op(client_id, "palat_l")
            time.sleep(5)
            basket_op(client_id, "default_l")
            return


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(3, 6), (5, 6), (5, 2), (3, 0)]     # example list of tuples

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
                    print(
                        '\n[ERROR] Failed starting the simulation in CoppeliaSim!')
                    print(
                        'start_simulation function is not configured correctly, check the code!')
                    print()
                    sys.exit()

            except Exception:
                print(
                    '\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print('\n[ERROR] Failed connecting to Remote API server!')
            print('[WARNING] Make sure the CoppeliaSim software is running and')
            print(
                '[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
            print(
                '[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print()
            sys.exit()

    except Exception:
        print(
            '\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
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
                        print(
                            '\nDisconnected successfully from Remote API Server in CoppeliaSim!')

                    else:
                        print(
                            '\n[ERROR] Failed disconnecting from Remote API server!')
                        print(
                            '[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

                except Exception:
                    print(
                        '\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print(
                    '\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
                print(
                    '[ERROR] stop_simulation function is not configured correctly, check the code!')
                print('Stop the CoppeliaSim simulation manually.')

            print()
            sys.exit()

        except Exception:
            print(
                '\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
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
