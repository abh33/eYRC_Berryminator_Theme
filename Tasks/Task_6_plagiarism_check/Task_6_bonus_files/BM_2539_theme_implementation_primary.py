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

# Team ID:			BM_2539
# Author List:		Amanullah Asad, Swastik Pal
# Filename:			theme_implementation.py
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
import os, sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode

import shared_resources
from task_2a import get_vision_sensor_image, get_vision_sensor_depth_image, transform_vision_sensor_depth_image, \
    detect_berries, detect_berry_positions, get_labeled_image
from task_3 import start_simulation, stop_simulation, init_remote_api_server, exit_remote_api_server, \
    get_bot_position, task_3_primary, view_image, nav_logic, init_setup, encoders, rotate
from task_4 import find_berries, target_berry, arm_stop, move_arm, tip_basket, get_arm_joint_handles, \
    sort_and_filter_dictionary, \
    clear_vision_sensor

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print(
        'sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def decode_json(file_location="Theme_Config.json"):
    """
    Decodes provided Json file. Gives output as {'CB1': {'B': 3, 'L': 0, 'S': 0}, 'CB2': {'B': 0, 'L': 2, 'S': 1}} 
    for input {"B": "3_CB1", "L": "2_CB2", "S": "1_CB2"} (example)
    """
    box_1 = {'B': 0, 'L': 0, 'S': 0}
    box_2 = {'B': 0, 'L': 0, 'S': 0}
    res_dict = {}
    with open(file_location) as file:
        data = json.load(file)
    for key, value in data.items():
        if value[4] == '1':
            box_1[key] = int(value[0])
        elif value[4] == '2':
            box_2[key] = int(value[0])
    res_dict['CB1'] = box_1
    res_dict['CB2'] = box_2
    # print(res_dict['CB1'][0])
    # print(data['B']) #debugging
    return res_dict


def go_to_room(client_id: int, room_number: int, rooms_entries: dict, wheel_joints):
    """
    Moves the bot to the centre of the room specified. The bot faces the rack at stop.
    :param client_id:
    :param room_number: the room number according to the rulebook
    :param rooms_entries: the list of room entrances (1-indexed)
    :param wheel_joints: a list containing joint object handles of individual joints

    R1  R2

    R4  R3
    """
    global current_room
    angle_to_rotate = angle_while_entering[room_number] - shared_resources.bot_front_angle
    rotate(client_id, wheel_joints, angle_to_rotate)
    shared_resources.bot_front_angle = angle_while_entering[room_number]

    diff = np.subtract(rooms_entries[room_number], bot_home)
    if math.fabs(diff[0]) > math.fabs(diff[1]):
        vert_first = False
    else:
        vert_first = True
    target_points = [rooms_entries[room_number]]
    nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=vert_first)
    target_points = [room_centres[room_number]]
    nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=not vert_first)

    angle_to_rotate = rack_angles[room_number] - shared_resources.bot_front_angle
    rotate(client_id, wheel_joints, angle_to_rotate)
    shared_resources.bot_front_angle = rack_angles[room_number]
    current_room = room_number


def go_home_from_room(client_id: int, rooms_entries: dict, wheel_joints):
    """
    Brings the bot home from the centre of the room specified.
    :param client_id:
    :param rooms_entries: the list of room entrances (1-indexed)
    :param wheel_joints: a list containing joint object handles of individual joints

    R1  R2

    R4  R3
    """

    global current_room
    angle_to_rotate = angle_while_entering[current_room] - shared_resources.bot_front_angle
    rotate(client_id, wheel_joints, angle_to_rotate)
    shared_resources.bot_front_angle = angle_while_entering[current_room]

    if current_room > 0:
        diff = np.subtract(rooms_entries[current_room], bot_home)
    else:
        diff = np.subtract(bot_home, shared_resources.bot_coord)
    if math.fabs(diff[0]) > math.fabs(diff[1]):
        vert_first = False
    else:
        vert_first = True
    if current_room > 0:
        target_points = [rooms_entries[current_room]]
        nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=vert_first)
    target_points = [bot_home]
    nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=not vert_first)
    # angle_to_rotate = rack_angles[current_room] - shared_resources.bot_front_angle
    # rotate(client_id, wheel_joints, angle_to_rotate)
    # shared_resources.bot_front_angle = rack_angles[current_room]
    current_room = 0


def go_to_collection_box(client_id: int, box_number: int, wheel_joints):
    """
    Positions the bot in front of the collection box specified. The bot faces the box at stop.
    The bot is considered to start from home.
    :param client_id:
    :param box_number: the room number according to the rulebook
    :param wheel_joints: a list containing joint object handles of individual joints

    R1  R2

    R4  R3
    """
    global current_room
    angle_to_rotate = 90 - shared_resources.bot_front_angle
    rotate(client_id, wheel_joints, angle_to_rotate)
    shared_resources.bot_front_angle = 90

    diff = np.subtract(CB_front_locations[box_number], bot_home)
    if math.fabs(diff[0]) > math.fabs(diff[1]):
        vert_first = False
    else:
        vert_first = True
    target_points = [CB_front_locations[box_number]]
    nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=vert_first)
    # angle_to_rotate = rack_angles[box_number] - shared_resources.bot_front_angle
    # rotate(client_id, wheel_joints, angle_to_rotate)
    # shared_resources.bot_front_angle = rack_angles[box_number]
    current_room = 0


def go_home_from_collection_box(client_id: int, wheel_joints):
    """
    Brings the bot home from the front of the collection box specified.
    :param client_id:
    :param wheel_joints: a list containing joint object handles of individual joints

    R1  R2

    R4  R3
    """

    global current_room
    # angle_to_rotate = 90 - shared_resources.bot_front_angle
    # rotate(client_id, wheel_joints, angle_to_rotate)
    # shared_resources.bot_front_angle = 90

    diff = np.subtract(bot_home, shared_resources.bot_coord)
    if math.fabs(diff[0]) > math.fabs(diff[1]):
        vert_first = False
    else:
        vert_first = True
    target_points = [bot_home]
    nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=not vert_first)
    # angle_to_rotate = rack_angles[current_room] - shared_resources.bot_front_angle
    # rotate(client_id, wheel_joints, angle_to_rotate)
    # shared_resources.bot_front_angle = rack_angles[current_room]
    current_room = 0


def align_with_qr(client_id):
    pass


def orient_towards_rack(client_id, wheel_joints):
    """
    Generic solution for orienting bot towards the rack in the room. (NOT USED)

    :param client_id:
    :param wheel_joints:
    :return:
    """
    max_berries = 0
    max_index = 0
    for i in range(4):
        berries_dictionary, berry_positions_dictionary = find_berries(client_id)
        berries_found = len(berry_positions_dictionary["Strawberry"]) > 0 + len(
            berry_positions_dictionary["Strawberry"]) > 0 + len(berry_positions_dictionary["Strawberry"]) > 0
        if berries_found > max_berries:
            max_berries = berries_found
            max_index = i
        rotate(client_id, wheel_joints, rotate_through=90)
    if max_berries > 0:
        rotate(client_id, wheel_joints, rotate_through=-90 * (3 - max_index))
        return 0
    else:
        return -1  # no rack or no berries on rack


def count_berries_in_room(berry_positions_dictionary):
    """
    Counts the number of berries in the room
    """
    berry_count = {}

    for name in shared_resources.berry_names:
        berry_count[name] = len(berry_positions_dictionary[name])

    return berry_count


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

    if (type(berry_name) != str):
        berry_name = str(berry_name)

    if (type(x_coor) != float):
        x_coor = float(x_coor)

    if (type(y_coor) != float):
        y_coor = float(y_coor)

    if (type(depth) != float):
        depth = float(depth)

    data_to_send = [berry_name, str(x_coor), str(y_coor), str(depth)]
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',
                                                                                      sim.sim_scripttype_childscript,
                                                                                      'detected_berry_by_team', [], [],
                                                                                      data_to_send, emptybuff,
                                                                                      sim.simx_opmode_blocking)
    return return_code


##################################################
##############################################################

# GLOBAL VARIABLES HERE
##############################################################

target_CB_1 = None
target_CB_2 = None
berries_in_room = {}
CB_front_locations = {1: (1, 10.05), 2: (6.8, 9.9)}
room_centres = {1: (1, 6.8), 2: (6.8, 7), 3: (7, 1.2), 4: (1.2, 1)}
bot_home = (4, 4)
rack_angles = {1: 90, 2: 0, 3: -90, 4: 180}
current_room = 0
angle_while_entering = {}


##############################################################

def theme_implementation_primary(client_id, rooms_entries):
    """
    Purpose:
    ---
    This is the only function that is called from the main function. Make sure to fill it
    properly, such that the bot completes the Theme Implementation.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()

    `rooms_entries`         :   [ list of tuples ]
        Room entry co-ordinate of each room in order.


    Returns:
    ---

    Example call:
    ---
    theme_implementation_primary(client_id, rooms_entries)

    """

    # initialize shared resources
    shared_resources.client_id = client_id
    _, shared_resources.ping_time = sim.simxGetPingTime(clientID=client_id)
    shared_resources.ping_time /= 1000  # convert to secs
    shared_resources.wheel_joints = init_setup(client_id)
    shared_resources.arm_joint_handles = get_arm_joint_handles(client_id)

    rooms_entries = {(i + 1): list(rooms_entries[i]) for i in range(len(rooms_entries))}  # convert list to dictionary
    for i in range(1, 5):
        # diff = np.subtract(rooms_entries[i], bot_home)
        # if math.fabs(diff[0]) > math.fabs(diff[1]):
        #     vert_first = False
        # else:
        #     vert_first = True
        # if rooms_entries[i][0] == rooms_entries[i][1]:
        #     if rooms_entries[i][0] * rooms_entries[i][1] > 0:  # same sign
        #         angle_while_entering[i] = 45
        #     else:
        #         angle_while_entering[i] = -45
        if rooms_entries[i][0] == 3 or rooms_entries[i][0] == 5:
            angle_while_entering[i] = 0
        elif rooms_entries[i][1] == 3 or rooms_entries[i][1] == 5 or rooms_entries[i][1] == 9:
            angle_while_entering[i] = 90

    shifting_factor = 0.3  # To ensure room entry goes smoothly
    for i in range(1, 5):
        if rooms_entries[i][0] == 0 or rooms_entries[i][0] == 6:
            rooms_entries[i][0] += shifting_factor
        elif rooms_entries[i][0] == 2 or rooms_entries[i][0] == 8:
            rooms_entries[i][0] -= shifting_factor
        if rooms_entries[i][1] == 0 or rooms_entries[i][1] == 6:
            rooms_entries[i][1] += shifting_factor
        elif rooms_entries[i][1] == 2 or rooms_entries[i][1] == 8:
            rooms_entries[i][1] -= shifting_factor

    # Using the decoded json file
    res_dict = decode_json()
    global target_CB_1, target_CB_2
    target_CB_1 = res_dict["CB1"]
    target_CB_2 = res_dict["CB2"]

    # print(res_dict)  # debugging only
    # print(target_CB_1)  # debugging only
    # print(target_CB_2)  # debugging only
    # print(get_bot_position(client_id))  # debugging only
    # align_with_qr(client_id)

    print("Filling Collection Box 1.")  # debugging only

    # Plucking CB1 berries
    for i in range(1, 2):
        go_to_room(client_id, i, rooms_entries, shared_resources.wheel_joints)

        # clear_vision_sensor(client_id)
        berries_dictionary, berry_positions_dictionary = find_berries(client_id)
        berries_in_room[i] = count_berries_in_room(berry_positions_dictionary)
        # print(berry_positions_dictionary)  # debugging only
        berry_positions_dictionary = sort_and_filter_dictionary(berry_positions_dictionary)

        while berries_in_room[i]["Blueberry"] > 0:
            if target_CB_1["B"] == 0:
                break
            target_berry(client_id, "Blueberry", berry_positions_dictionary)
            # target_berry(client_id, "Blueberry")
            berries_in_room[i]["Blueberry"] = berries_in_room[i]["Blueberry"] - 1
            target_CB_1["B"] = target_CB_1["B"] - 1
        while berries_in_room[i]["Lemon"] > 0:
            if target_CB_1["L"] == 0:
                break
            target_berry(client_id, "Lemon", berry_positions_dictionary)
            # target_berry(client_id, "Lemon")
            berries_in_room[i]["Lemon"] = berries_in_room[i]["Lemon"] - 1
            target_CB_1["L"] = target_CB_1["L"] - 1
        while berries_in_room[i]["Strawberry"] > 0:
            if target_CB_1["S"] == 0:
                break
            target_berry(client_id, "Strawberry", berry_positions_dictionary)
            # target_berry(client_id, "Strawberry")
            berries_in_room[i]["Strawberry"] = berries_in_room[i]["Strawberry"] - 1
            target_CB_1["S"] = target_CB_1["S"] - 1

        # print(berry_positions_dictionary)  # debugging only
        go_home_from_room(client_id, rooms_entries, shared_resources.wheel_joints)
        if target_CB_1["B"] + target_CB_1["L"] + target_CB_1["S"] == 0:
            break
    go_to_collection_box(client_id, 1, shared_resources.wheel_joints)
    #align_with_qr(client_id)  # self-correction
    tip_basket(client_id)
    #go_home_from_collection_box(client_id, shared_resources.wheel_joints)
    #align_with_qr(client_id)  # self-correction

    print("Filling Collection Box 2.")  # debugging only
    # Plucking CB2 berries
    for i in range(1, 2):
        break
        go_to_room(client_id, i, rooms_entries, shared_resources.wheel_joints)

        # clear_vision_sensor(client_id)
        berries_dictionary, berry_positions_dictionary = find_berries(client_id)
        berries_in_room[i] = count_berries_in_room(berry_positions_dictionary)
        # print(berry_positions_dictionary)  # debugging only
        berry_positions_dictionary = sort_and_filter_dictionary(berry_positions_dictionary)

        while berries_in_room[i]["Blueberry"] > 0:
            if target_CB_2["B"] == 0:
                break
            target_berry(client_id, "Blueberry", berry_positions_dictionary)
            # target_berry(client_id, "Blueberry")
            berries_in_room[i]["Blueberry"] = berries_in_room[i]["Blueberry"] - 1
            target_CB_2["B"] = target_CB_2["B"] - 1
        while berries_in_room[i]["Lemon"] > 0:
            if target_CB_2["L"] == 0:
                break
            target_berry(client_id, "Lemon", berry_positions_dictionary)
            # target_berry(client_id, "Lemon")
            berries_in_room[i]["Lemon"] = berries_in_room[i]["Lemon"] - 1
            target_CB_2["L"] = target_CB_2["L"] - 1
        while berries_in_room[i]["Strawberry"] > 0:
            if target_CB_2["S"] == 0:
                break
            target_berry(client_id, "Strawberry", berry_positions_dictionary)
            # target_berry(client_id, "Strawberry")
            berries_in_room[i]["Strawberry"] = berries_in_room[i]["Strawberry"] - 1
            target_CB_2["S"] = target_CB_2["S"] - 1

        # print(berry_positions_dictionary)  # debugging only
        go_home_from_room(client_id, rooms_entries, shared_resources.wheel_joints)
        if target_CB_2["B"] + target_CB_2["L"] + target_CB_2["S"] == 0:
            break
    #go_to_collection_box(client_id, 2, shared_resources.wheel_joints)
    #align_with_qr(client_id)  # self-correction
    #tip_basket(client_id)

    #print(berries_in_room)  # debugging only
    # time.sleep(3)  # time to observe

    return


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(2, 5), (5, 8), (5, 2), (3, 0)]  # example list of tuples

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
                    print(
                        '\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
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
