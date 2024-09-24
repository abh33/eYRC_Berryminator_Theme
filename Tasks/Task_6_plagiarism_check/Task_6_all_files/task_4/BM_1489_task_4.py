'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ BM-1489 ]
# Author List:		[ Shaik, Gurkirat, Kaushik, Harika ]
# Filename:			task_4.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################


from http import client
from re import I
import cv2
import numpy as np
import os
import sys
import traceback
import math
import time
from pyzbar.pyzbar import decode
#############################################################
#############################################################
import task_1b
import task_2a
import task_3
##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


# ADD UTILITY FUNCTIONS HERE #################+
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def trajectory(client_id, current, target):

    current_copy = current.copy()
    target_copy = target.copy()
    step_size = 0.25

    while True:
        error = np.array(target_copy)-np.array(current_copy)

        for i in range(3):
            if error[i] >= step_size:
                current_copy[i] += step_size
            elif -1*error[i] >= step_size:
                current_copy[i] -= step_size

        if abs(error[0]) < step_size and abs(error[1]) < step_size and abs(error[2]) < step_size:
            break

        reachTarget(client_id, current_copy)
    reachTarget(client_id, target_copy)
    return

# def trajectory(client_id, current, target):

#     current_copy = current.copy()
#     target_copy = target.copy()
#     target_copy[1] = ((0.74**2-target[0]**2-target[2] **
#                       2)**0.5)*(target[1])/abs(target[1])
#     reachTarget(client_id, target_copy)
#     reachTarget(client_id, target)
#     call_open_close(client_id, "close")
#     current_copy[2] = 0
#     # current_copy[1] = 0
#     reachTarget(client_id, current_copy)
#     reachTarget(client_id, current)

#     return


def call_open_close(client_id, command):
    command = [command]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [], command, emptybuff, sim.simx_opmode_oneshot_wait)
    return_code, left_cup_handle = sim.simxGetObjectHandle(
        client_id, 'ball_holder_cup_1_dyn', sim.simx_opmode_blocking)
    return_code, right_cup_handle = sim.simxGetObjectHandle(
        client_id, 'ball_holder_cup_2_dyn', sim.simx_opmode_blocking)
    count = 0
    while True:
        return_code, left_cup_linear, left_cup_angular = sim.simxGetObjectVelocity(
            client_id, left_cup_handle, sim.simx_opmode_oneshot_wait)
        return_code, right_cup_linear, right_cup_angular = sim.simxGetObjectVelocity(
            client_id, right_cup_handle, sim.simx_opmode_oneshot_wait)
        # print("left ", count)
        left_cup_linear = np.array(left_cup_linear)
        left_cup_angular = np.array(left_cup_angular)
        left_cup_linear_speed = np.linalg.norm(left_cup_linear)
        left_cup_angular_speed = np.linalg.norm(left_cup_angular)
        # print(left_cup_linear_speed, left_cup_angular_speed)

        # print("right ", count)
        right_cup_linear = np.array(right_cup_linear)
        right_cup_angular = np.array(right_cup_angular)
        right_cup_linear_speed = np.linalg.norm(right_cup_linear)
        right_cup_angular_speed = np.linalg.norm(right_cup_angular)
        # print(right_cup_linear_speed, right_cup_angular_speed)
        # count += 1
        if left_cup_linear_speed < 0.02 and right_cup_linear_speed < 0.02:
            # print("i am done")
            break


def reachTarget(client_id, coor):
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'setPosition', [], coor, [], emptybuff, sim.simx_opmode_oneshot_wait)

    return_code, tip_handle = sim.simxGetObjectHandle(
        client_id, 'tip', sim.simx_opmode_blocking)
    return_code, top_handle = sim.simxGetObjectHandle(
        client_id, 'robotic_arm_rj_23', sim.simx_opmode_blocking)
    return_code, middle_handle = sim.simxGetObjectHandle(
        client_id, 'robotic_arm_rj_12', sim.simx_opmode_blocking)
    count = 0
    while True:

        return_code, tip_linear, tip_angular = sim.simxGetObjectVelocity(
            client_id, tip_handle, sim.simx_opmode_oneshot_wait)
        return_code, top_linear, top_angular = sim.simxGetObjectVelocity(
            client_id, top_handle, sim.simx_opmode_oneshot_wait)
        return_code, middle_linear, middle_angular = sim.simxGetObjectVelocity(
            client_id, middle_handle, sim.simx_opmode_oneshot_wait)

        # print("tip")
        tip_linear = np.array(tip_linear)
        tip_angular = np.array(tip_angular)
        tip_linear_speed = np.linalg.norm(tip_linear)
        tip_angular_speed = np.linalg.norm(tip_angular)
        # print(tip_linear_speed, tip_angular_speed*180/math.pi)

        # print("middle_joint", count)
        middle_linear = np.array(middle_linear)
        middle_angular = np.array(middle_angular)
        middle_linear_speed = np.linalg.norm(middle_linear)
        middle_angular_speed = np.linalg.norm(middle_angular)
        # print(middle_linear_speed, middle_angular_speed*180/math.pi)

        # print("top_joint", count)
        top_linear = np.array(top_linear)
        top_angular = np.array(top_angular)
        top_linear_speed = np.linalg.norm(top_linear)
        top_angular_speed = np.linalg.norm(top_angular)
        # print(top_linear_speed, top_angular_speed*180/math.pi)

        if tip_linear_speed < 0.05:
            # print("i am done")
            break
        count += 1
        x = 0

##############################################################


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


def task_4_primary(client_id):
    # """
    # Purpose:
    # ---
    # This is the only function that is called from the main function. Make sure to fill it
    # properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

    # Input Arguments:
    # ---
    # `client_id`         :   [ integer ]
    # 	the client id of the communication thread returned by init_remote_api_server()

    # Returns:
    # ---

    # Example call:
    # ---
    # task_4_primary(client_id)

    # """

    wheel_joints = task_3.init_setup(client_id)
    # vision_sensor_image1, image_resolution1, return_code1 = task_3.get_vision_sensor_image(
    #     client_id)
    # transformed_image1 = task_3.transform_vision_sensor_image(
    #     vision_sensor_image1, image_resolution1)
    # qr_code = task_3.detect_qr_codes(transformed_image1)
    # target_points = [(4, 4)]
    # print(qr_code)
    # task_3.nav_logic(client_id, qr_code, wheel_joints, (int(qr_code[0]), int(qr_code[1])),
    #                  target_points, transformed_image1)

    return_code, vision_sensor2_handle = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(
        client_id, vision_sensor2_handle)
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(
        client_id, vision_sensor2_handle)
    transformed_image = task_1b.transform_vision_sensor_image(
        vision_sensor_image, image_resolution)
    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(
        vision_sensor_depth_image, depth_image_resolution)
    berries_dictionary = task_2a.detect_berries(
        transformed_image, transformed_depth_image)
    berry_positions_dictionary = task_2a.detect_berry_positions(
        berries_dictionary)
    print(berry_positions_dictionary)
    box_x = 0.1917
    box_y = -0.23302
    box = [[box_x, box_y, -0.3373],
           [box_x, box_y, -0.2403], [box_x, box_y, -0.1383]]
    default_pos = [0, -1.2, -0.12]
    sleep_time = 0.5
    close_time = 0.25
    coor = []
    # 0 strawberry
    # 1 Lemon
    # 2 Blueberry
    fruit_list = ["Strawberry", "Lemon", "Blueberry"]
    call_open_close(client_id, "open")
    ####################################################################################
    for j in range(len(fruit_list)):
        berry_positions_dictionary[fruit_list[j]].sort(key=lambda x: x[1])
        for i in range(len(berry_positions_dictionary[fruit_list[j]])):
            x, y, z = berry_positions_dictionary[fruit_list[j]][i]
            # return_code = send_identified_berry_data(
            #     client_id, 'Strawberry', x, y, z)
            coor = [x, y, z]
            reachTarget(client_id, coor)
            # trajectory(client_id, default_pos, coor)
            call_open_close(client_id, "close")
            reachTarget(client_id, default_pos)
            # trajectory(client_id, coor, default_pos)
            reachTarget(client_id, box[j])
            call_open_close(client_id, "open")
            reachTarget(client_id, default_pos)

    # #########################################


if __name__ == "__main__":

    ##################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        client_id = task_1b.init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = task_1b.start_simulation(client_id)

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

        task_4_primary(client_id)
        # time.sleep(1)

        try:
            return_code = task_1b.stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    task_1b.exit_remote_api_server(client_id)
                    if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
        print(
            '\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
