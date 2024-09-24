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


# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_4.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]


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
import task_1b
import task_2a
import task_3
################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def waitForMovementExecuted(client_id, movt_id):
    """
    Purpose:
    ---
    This function is called after a function for movement of joints or target is called. 
    This creates a time delay till the previous command is completely executed.

    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    'movt_id'			:	[string]
            the movt_id is the id given to the movement that is being called before this function is called
    Return:
    ---
    retCode,s=sim.simxGetStringSignal(client_id,'_executedMovId',sim.simx_opmode_streaming)

    Example call:
    waitForMovementExecuted(client_id, movt_id)

    """
    sim.simxSetStringSignal(client_id, '_executedMovId',
                            "notdummy", sim.simx_opmode_oneshot)
    retCode, s = sim.simxGetStringSignal(
        client_id, '_executedMovId', sim.simx_opmode_streaming)

    while s != movt_id:
        retCode, s = sim.simxGetStringSignal(
            client_id, '_executedMovId', sim.simx_opmode_buffer)
        if retCode == sim.simx_return_ok and type(s) == bytearray:
            s = s.decode('ascii')
    retCode, s = sim.simxGetStringSignal(
        client_id, '_executedMovId', sim.simx_opmode_discontinue)

####################################################################

def get_berry_dictionary(client_id, vision_sensor_2, position):
    """
    Purpose:
    ---
    This function gets all the details of the berries that are detected by vision_sensor_2.

    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().
    'vision_sensor_2'   :	[integer]
            it is the object handle of the vision_sensor_2.

    Return:
    ---
    return berry_positions_dictionary

    Example call:
    get_berry_dictionary(client_id, vision_sensor_2)

    """
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(
        client_id, vision_sensor_2)

    if ((return_code == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(vision_sensor_image) > 0)):
        transformed_image = task_1b.transform_vision_sensor_image(
            vision_sensor_image, image_resolution)
    else:
        print('vision sensor image not captured properly')

    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(
        client_id, vision_sensor_2)
    if ((return_code_2 == sim.simx_return_ok) and (len(depth_image_resolution) == 2) and (len(vision_sensor_depth_image) > 0)):
        transformed_depth_image = task_2a.transform_vision_sensor_depth_image(
            vision_sensor_depth_image, depth_image_resolution)
    else:
        print('vision sensor depth image not captured properly')

    berries_dictionary = task_2a.detect_berries(
        transformed_image, transformed_depth_image)
    berry_positions_dictionary = task_2a.detect_berry_positions(
        berries_dictionary,position)
    return berry_positions_dictionary

####################################################################

def call_open_close(client_id, command):
    """
    Purpose:
    ---
    This function when called opens and closes the gripper according to the command argument that is 
    passed on while calling the function

    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    'command'			:	[string]
            the command tells if the gripper has to open or close
            can either be "open" or "close"

    Return:
    ---
    retCode,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)

    Example call:
    call_open_close(client_id, command)

    """
    command = [command]
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'gripper', sim.sim_scripttype_childscript, 'open_close', [], [], command, emptybuff, sim.simx_opmode_blocking)

####################################################################

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

def pluckberry(client_id, coord, basket,vision_sensor_1,exact_pos_temp,wheel_joints,orientation):
    """
    Purpose:
    ---
    This function plucks the berry from the plant and then drops it in the required basket.

    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()
    'coord'             "   [list]
            this has the coord of the berry to be collected
    'basket'            :   [integer]
            this indicates if the berry is to be dropped to the basket_1 or the basket_2
    'vision_sensor_1'   :	[integer]
            it is the object handle of the vision_sensor under consideration
    'exact_pos_temp'    :   [tuple]
            tells where the bot has to go to when calling this function
    'wheel_joints'		:	[list]
            the joint handles of the wheel joints 
    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 
    Return:
    ---
    nil
    
    Example call:
    pluckberry(client_id, coord, basket,vision_sensor_1,exact_pos_temp,wheel_joints,orientation)
    
    """
    emptybuff = bytearray()

    #first goes to a position above the berry so as to not knock it off or disturb other berries. It later moves down.
    coord_packed = sim.simxPackFloats((coord[0], coord[1], 0.6155739426612854))
    sim.simxCallScriptFunction(client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'input_TargetPositions', [
    ], [], ['berry_reach'], coord_packed, sim.simx_opmode_blocking)
    waitForMovementExecuted(client_id, 'berry_reach')
    

    coord_packed = sim.simxPackFloats((coord[0], coord[1], coord[2]))
    sim.simxCallScriptFunction(client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'input_TargetPositions', [
    ], [], ['berry_reach'], coord_packed, sim.simx_opmode_blocking)
    waitForMovementExecuted(client_id, 'berry_reach')


    call_open_close(client_id, "close")


    coord_packed = sim.simxPackFloats((coord[0], coord[1], 0.6155739426612854))
    sim.simxCallScriptFunction(client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'input_TargetPositions', [
    ], [], ['berry_reach'], coord_packed, sim.simx_opmode_blocking)
    waitForMovementExecuted(client_id, 'berry_reach')


    if basket == 2:
        joint_packed = sim.simxPackFloats((10*math.pi/180, 0, -7*math.pi/180))
    else:
        joint_packed = sim.simxPackFloats((0, -30*math.pi/180, 0*math.pi/180))
    sim.simxCallScriptFunction(client_id, 'robotic_arm', sim.sim_scripttype_childscript, 'input_JointPositions', [
    ], [], ['in_basket'], joint_packed, sim.simx_opmode_blocking)
    waitForMovementExecuted(client_id, 'in_basket')


    call_open_close(client_id, "open")

##############################################################


def pickberry(client_id, berry_details, requirement):
    """
    Purpose:
    ---
    This function ensures that the berries are collected in the manner where 
        the nearest to the gripper is collect first and so on.
    This is done so that we do not knock off other berries while plucking the 
        berries behind it first.

    Input arguments
    ---
    `client_id` 	:  [ integer ]
            the client_id generated from start connection remote API, it should be stored in a global variable
    'berry_details'  :	[list]
            Details about the berry's position
    Return
    ---
            Return a list of ordered_berry_details
    """
    ordered_berry_details = []
    minimum_dist = 1000
    for berry in berry_details:
        dist = task_3.dist((0, 0, 0), berry)
        if dist < minimum_dist:
            minimum_dist = dist
            ordered_berry_details.insert(0, berry)
        else:
            ordered_berry_details.append(berry)

    return ordered_berry_details[0]

##############################################################

def get_order_list(berry_positions_dictionary):
    """
    Purpose:
    ---
    This function returns a list containing three lists, each with two elements.
    The first element indicates the name of the plant{'strawberry','lemon','blueberry'} and the second element indicates 
        the approximate position of the plant with respect to the bot.

    Input arguments:
    'berry_positions_dictionary'    :   [dictionary]
        the dictionary with the details of the berries

    Return:
        'order_list': returns the ordered list, which is the order in which the plant is to be attended to.

    """

    order_list = []
    min_dist = 1000
    
    #determines the center plant
    for key in berry_positions_dictionary:
        if berry_positions_dictionary[key] != []:
            dist = task_3.dist((0, 0), (berry_positions_dictionary[key][0][0],berry_positions_dictionary[key][0][1]))
            if dist < min_dist:
                min_dist = dist
                order_list.insert(0,key)
            else:
                order_list.append(key)


    #this implies that the nearest berry (along xy plane) lies in the middle vertical band ie,the berries r either at the top or bottom of image
    if -0.15913319587708<=berry_positions_dictionary[order_list[0]][0][0]<=0.19086790084839: 
        order_list[0]=[order_list[0],'center']
        #this implies the second order_list element is to the left of the third    
        if berry_positions_dictionary[order_list[1]][0][0]<berry_positions_dictionary[order_list[2]][0][0]:
            order_list[1]=[order_list[1],'left']
            order_list[2]=[order_list[2],'right']
        else:
            order_list[1]=[order_list[1],'right']
            order_list[2]=[order_list[2],'left']
        

    else:
        order_list[0]=[order_list[0],'center']
        #this implies the second order_list element is below the third
        if berry_positions_dictionary[order_list[1]][0][1]<berry_positions_dictionary[order_list[2]][0][1]:
            order_list[1]=[order_list[1],'behind']
            order_list[2]=[order_list[2],'front']
        else:
            order_list[1]=[order_list[1],'front']
            order_list[2]=[order_list[2],'behind']
 
    return order_list

##############################################################


def task_4_primary(client_id, berry_requirements, current_pos, wheel_joints, exact_pos_temp, orientation):
    """
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot detects, plucks & deposits the required berries.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		    the client id of the communication thread returned by init_remote_api_server()

    `berry_requirements`    : {dictionary}
            dictionary with a list as the value for every key (berry).
            the first element of the list indicates the number of berries to be picked
            the second shows the collection box it needs to be deposited in

	`current_pos`        : (integer,integer)
            tuples indicating the qr code upon which the bot stands currently

    'wheel_joints`      :   [ list]
            list containing joint object handles of individual joints in the order forward right,forward left,rear right,rear left

    `exact_pos_temp`        : (integer, integer)
            tuple with two values which represent the current alignment of the bot with respect to the qr code.

    `orientation`       : integer
            can have two values- 0: indicates that the bot is aligned according to the x axis of the scene.
                                90: indicates that the bot is aligned according to the y axis of the scene.

	Returns:
	---
	`berry_requirements`
            updated berry requirements

	Example call:
	---
	berry_requirements=task_4_primary(client_id, berry_requirements, current_pos, wheel_joints, exact_pos_temp, orientation)
	
	"""

# INITIALISATION
    _, vision_sensor_1 = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    _, vision_sensor_2 = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)


    bot_position='center'
    berry_positions_dictionary = get_berry_dictionary(
        client_id, vision_sensor_2,bot_position)

    #a list with lists containing plant name and position, with the first list always containing the center berry
    order_list=get_order_list(berry_positions_dictionary) 
    
    
    #PLUCKING
    for element in order_list:
        
        bot_position=element[1]
        berry_positions_dictionary = get_berry_dictionary(
            client_id, vision_sensor_2,bot_position)
         
        while ((len(berry_positions_dictionary[element[0]]) != 0) and (berry_requirements[element[0][0]][0] != 0)):
            berry_positions_dictionary = get_berry_dictionary(
                client_id, vision_sensor_2,bot_position)
            
            # decides which berry to pick based on how close the berry is to the vision sensor
            coords = pickberry(client_id, berry_positions_dictionary[element[0]],berry_requirements[element[0][0]][0])
            send_identified_berry_data(client_id,element[0],coords[0],coords[1],coords[2])
            
            pluckberry(client_id, coords, berry_requirements[element[0][0]][1],vision_sensor_1,exact_pos_temp,wheel_joints,orientation)
            
            berry_requirements[element[0][0]][0] -= 1
            berry_positions_dictionary[element[0]].remove(coords)
            
    return berry_requirements

    ################################################################


if __name__ == "__main__":
    #OBSOLETE
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
        time.sleep(1)

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
