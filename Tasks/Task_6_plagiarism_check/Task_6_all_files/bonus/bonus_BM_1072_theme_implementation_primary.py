"""
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Theme Implementation of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
"""


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

import cv2
import numpy as np
import os, sys
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
    print("\n[ERROR] It seems the sim.py OR simConst.py files are not found!")
    print("\n[WARNING] Make sure to have following files in the directory:")
    print(
        "sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n"
    )
    sys.exit()

import task_1b
import task_2a
import task_3
import task_4

################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
##############################################################

def go_to_room(client_id, wheel_joints, current_pos, room, orientation):
    """
    Purpose:
    ---
    The bot reaches the axis of the room if it hasn't already and then proceeds to the center of the room.

    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints'		:	[list]
            the joint handles of the wheel joints 

    'current_pos'       :   [tuple]
            tuple which is the coordinate of the place where it is supposed to be in a given room, 
            that is the center of the room usually.

    'room'              :   [list]
            list containing three elements
            the first is the entry point to the room
            the second is the room number
            the third is the axis the entry of the room is facing

    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 


    Return:

    `exact_pos_temp`  :
                    which is the current alignment of the bot with respect to the qr code

    'current_pos'       :   [tuple]
            tuple which is the coordinate of the place where it is supposed to be in a given room, 
            that is the center of the room usually.

    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 

    ---
    Example call:
            exact_pos_temp,current_pos,orientation=set_bot_position(client_id, wheel_joints, current_pos, room, orientation)

    """

    exact_pos = (128, 128)
    exact_pos_temp=(128,128)

    if room[2] == "y":  # requires orientation 90
        # reaches the point on the axis closest to room entrance
        if (
            current_pos[0] != 4
        ):  # this means that the bot is on the x axis n so its orientation is 0
            task_3.nav_logic(
                client_id,
                [current_pos, (4, current_pos[1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(4, current_pos[1]), (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.set_bot_orientation(client_id, wheel_joints, 90)
            orientation = 90

        else:  # its already on the y axis so its orientation is 90
            task_3.nav_logic(
                client_id,
                [current_pos, (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )


        # goes to the center of the room
        if room[1] == 1:
            task_3.nav_logic(
                client_id,
                [(4, room[0][1]), (1, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(1, room[0][1]), (1, 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            current_pos=(1, 7)

        elif room[1] == 2:
            task_3.nav_logic(
                client_id,
                [(4, room[0][1]), (7, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(7, room[0][1]), (7, 7)],
                wheel_joints,
                (128,76),
                orientation,
            )
            exact_pos_temp=(128,76)
            current_pos=(7, 7)

        elif room[1] == 3:
            task_3.nav_logic(
                client_id,
                [(4, room[0][1]), (7, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(7, room[0][1]), (7, 1)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            current_pos=(7,1)

        else:
            task_3.nav_logic(
                client_id,
                [(4, room[0][1]), (1, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(1, room[0][1]), (1, 1)],
                wheel_joints,
                (128,180),
                orientation,
            )
            exact_pos_temp=(128,180)
            current_pos=(1,1)




    elif room[2] == "x":  # requires orientation 0
        # reaches the point on the axis closest to room entrance
        if current_pos[1] != 4:
            task_3.nav_logic(
                client_id,
                [current_pos, (current_pos[0], 4)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            current_pos = (current_pos[0], 4)
            task_3.set_bot_orientation(client_id, wheel_joints, 0)
            orientation = 0

        # special provision for entry to rooms 1 and 4 through the x axis because it isn't aligned correctly
        if room[0][0] == 2:
            task_3.nav_logic(
                client_id,
                [(current_pos[0], 4), (room[0][0], 4)],
                wheel_joints,
                (180, 128),
                orientation,
            )
        else:
            task_3.nav_logic(
                client_id,
                [current_pos, (room[0][0], 4)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        # goes to the center of the room
        if room[1] == 1:
            if room[0][0] == 2:
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 4), (room[0][0], 7)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 7), (1, 7)],
                    wheel_joints,
                    (128,100),
                    orientation,
                )
            else:
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 4), (room[0][0], 7)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 7), (1, 7)],
                    wheel_joints,
                    (128,100),
                    orientation,
                )
                
            exact_pos_temp=(128,100)
            current_pos=(1, 7)

        elif room[1] == 2:
            task_3.nav_logic(
                client_id,
                [(room[0][0], 4), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (7, 7)],
                wheel_joints,
                (100,128),
                orientation,
            )
            exact_pos_temp=(100,128)
            current_pos=(7, 7)

        elif room[1] == 3:
            task_3.nav_logic(
                client_id,
                [(room[0][0], 4), (room[0][0], 1)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 1), (7, 1)],
                wheel_joints,
                (128,160),
                orientation,
            )
            exact_pos_temp=(128,160)
            current_pos=(7,1)

        else:
            if room[0][0] == 2:
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 4), (room[0][0], 1)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 1), (1, 1)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
            else:
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 4), (room[0][0], 1)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 1), (1, 1)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
            current_pos=(1,1)


    # for top (it is obsolete)
    else:
        if orientation == 90:
            task_3.set_bot_orientation(client_id, wheel_joints, 0)
            orientation = 0

        # reaches the point on the axis closest to room entrance
        if current_pos[1] != 10:
            task_3.nav_logic(
                client_id,
                [current_pos, (4, current_pos[1])],
                wheel_joints,
                exact_pos,
                orientation,
            )  # reaches y axis
            task_3.nav_logic(
                client_id,
                [(4, current_pos[1]), (4, 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )  # reaches top axis
            task_3.nav_logic(
                client_id,
                [(4, 10), (room[0][0], 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        else:
            task_3.nav_logic(
                client_id,
                [current_pos, (room[0][0], 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        # goes to the center of the room
        if room[1] == 1:
            task_3.nav_logic(
                client_id,
                [(room[0][0], 10), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (1, 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        else:
            task_3.nav_logic(
                client_id,
                [(room[0][0], 10), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (7, 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )

    return exact_pos_temp,current_pos,orientation


##############################################################


def exit_room(client_id, wheel_joints, room, orientation):
    """
    Purpose:
    ---
    This makes the bot go back to the entrance from the centre and then to its respective axis
    ---
    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints'		:	[list]
            the joint handles of the wheel joints 

    'room'              :   [list]
            list containing three elements
            the first is the entry point to the room
            the second is the room number
            the third is the axis the entry of the room is facing

    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 


    Return:

    'current_pos'       :   [tuple]
            tuple which is the coordinate of the place where it is supposed to be in a given room, 
            that is the center of the room usually.


    ---
    Example call:
    current_pos=exit_room(client_id, wheel_joints, room, orientation)

    """
    exact_pos = (128, 128)

    #The logic of this function is the same as the go_to_room function

    if room[2] == "y":
        if room[1] == 1:
            task_3.nav_logic(
                client_id,
                [(1, 7), (1, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(1, room[0][1]), (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )

        elif room[1] == 2:
            task_3.nav_logic(
                client_id,
                [(7, 7), (7, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(7, room[0][1]), (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )

        elif room[1] == 3:
            task_3.nav_logic(
                client_id,
                [(7, 1), (7, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(7, room[0][1]), (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )

        else:
            task_3.nav_logic(
                client_id,
                [(1, 1), (1, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(1, room[0][1]), (4, room[0][1])],
                wheel_joints,
                exact_pos,
                orientation,
            )

        current_pos = (4, room[0][1])

    elif room[2] == "x":
        if room[1] == 1:
            if room[0][0] == 2:
                task_3.nav_logic(
                    client_id,
                    [(1, 7), (room[0][0], 7)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 7), (room[0][0], 4)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )
            else:
                task_3.nav_logic(
                    client_id,
                    [(1, 7), (room[0][0], 7)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 7), (room[0][0], 4)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )

        elif room[1] == 2:
            task_3.nav_logic(
                client_id,
                [(7, 7), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (room[0][0], 4)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        elif room[1] == 3:
            task_3.nav_logic(
                client_id,
                [(7, 1), (room[0][0], 1)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 1), (room[0][0], 4)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        else:
            if room[0][0] == 2:
                task_3.nav_logic(
                    client_id,
                    [(1, 1), (room[0][0], 1)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 1), (room[0][0], 4)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
            else:
                task_3.nav_logic(
                    client_id,
                    [(1, 1), (room[0][0], 1)],
                    wheel_joints,
                    exact_pos,
                    orientation,
                )
                task_3.nav_logic(
                    client_id,
                    [(room[0][0], 1), (room[0][0], 4)],
                    wheel_joints,
                    (180, 128),
                    orientation,
                )

        current_pos = (room[0][0], 4)

    else:
        if room[1] == 1:
            task_3.nav_logic(
                client_id,
                [(1, 7), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (room[0][0], 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        else:
            task_3.nav_logic(
                client_id,
                [(7, 7), (room[0][0], 7)],
                wheel_joints,
                exact_pos,
                orientation,
            )
            task_3.nav_logic(
                client_id,
                [(room[0][0], 7), (room[0][0], 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )

        current_pos = (room[0][0], 10)

    return current_pos


##############################################################


def go_to_cb1(client_id, wheel_joints, current_pos, orientation):
    """
    Purpose:
    ---
    This makes the bot go to the collection_box_1 and then it unloads the berries in the basket_1 to the collection_box_1
    ---
    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints'		:	[list]
            the joint handles of the wheel joints 

    'current_pos'       :   [tuple]
            tuple which is the coordinate of the place where it is supposed to be in a given room, 
            that is the center of the room usually.

    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 


    Return:
    nil
 
    ---
    Example call:
    go_to_cb1(client_id, wheel_joints, current_pos, orientation)

    """
    exact_pos = (128, 128)
    emptybuff = bytearray()

    if current_pos[0] != 4:
        task_3.nav_logic(
            client_id,
            [current_pos, (4, current_pos[1])],
            wheel_joints,
            exact_pos,
            orientation,
        )
    if current_pos[1] != 10:
        task_3.nav_logic(
            client_id,
            [(4, current_pos[1]), (4, 10)],
            wheel_joints,
            exact_pos,
            orientation,
        )
    if orientation != 0:
        task_3.set_bot_orientation(client_id, wheel_joints, 0)
        orientation = 0

    task_3.nav_logic(
        client_id, [(4, 10), (2, 11)], wheel_joints, (180, 39), orientation
    )

    #here the basket_1 childscript function is called which will open
    #  the lid and thus unload the berries
    sim.simxCallScriptFunction(
        client_id,
        "basket_1",
        sim.sim_scripttype_childscript,
        "unload",
        [],
        [],
        [],
        emptybuff,
        sim.simx_opmode_blocking,
    )
    time.sleep(3)#gives time till all the berries are unloaded
    #this ensures that the lid is closed back and the bot is ready 
    # to go to the next unloading site
    sim.simxCallScriptFunction(
        client_id,
        "basket_1",
        sim.sim_scripttype_childscript,
        "close_back",
        [],
        [],
        [],
        emptybuff,
        sim.simx_opmode_blocking,
    )


##############################################################


def go_to_cb2(client_id, wheel_joints, current_pos, orientation):
    """
    Purpose:
    ---
    This makes the bot go to the collection_box_2 and then it unloads the berries in the basket_2 to the collection_box_2
    ---
    Input arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    'wheel_joints'		:	[list]
            the joint handles of the wheel joints 

    'current_pos'       :   [tuple]
            tuple which is the coordinate of the place where it is supposed to be in a given room, 
            that is the center of the room usually.

    'orientation'       :   [integer]
            this makes the bot's longer side parallel to y-axis if 'orientation'=0 and
            parallel to x-axis if 'orientation'=90 


    Return:
    nil
 
    ---
    Example call:
    go_to_cb2(client_id, wheel_joints, current_pos, orientation)

    """
    emptybuff = bytearray()
    exact_pos = (128, 128)
    
    if current_pos!=(2,11):
        if current_pos[0] != 4:
            task_3.nav_logic(
                client_id,
                [current_pos, (4, current_pos[1])],
                wheel_joints,
                exact_pos,
                orientation,
                )
        if current_pos[1] != 10:
            task_3.nav_logic(
                client_id,
                [(4, current_pos[1]), (4, 10)],
                wheel_joints,
                exact_pos,
                orientation,
            )
        if orientation != 0:
            task_3.set_bot_orientation(client_id, wheel_joints, 0)
            orientation = 0

        task_3.nav_logic(
            client_id, [(4, 10), (6, 10)], wheel_joints, (76,128), 0
            )

    else:
        task_3.nav_logic(
            client_id, [(2, 11), (6, 10)], wheel_joints, (76,128), 0
            )

    wheel_positions=task_3.encoders(client_id)
    task_3.set_bot_movement(
        client_id,
        wheel_joints,
        800*math.pi/180,800*math.pi/180,800*math.pi/180,800*math.pi/180
    )
    flag=1
    while flag:
        wheel_stream_positions=task_3.encoders(client_id)
        if((wheel_stream_positions[0]-wheel_positions[0])>3.14):
            flag=0
    task_3.set_bot_movement(
        client_id,
        wheel_joints,
        0,0,0,0
    )
    #here the basket_2 childscript function is called which will open 
    # the lid and thus unload the berries
    sim.simxCallScriptFunction(
        client_id,
        "basket_2",
        sim.sim_scripttype_childscript,
        "unload",
        [],
        [],
        [],
        emptybuff,
        sim.simx_opmode_blocking,
    )
    time.sleep(3)#gives time till all the berries are unloaded
    #this ensures that the lid is closed back and the bot is ready 
    # to go to the next unloading site
    sim.simxCallScriptFunction(
        client_id,
        "basket_2",
        sim.sim_scripttype_childscript,
        "close_back",
        [],
        [],
        [],
        emptybuff,
        sim.simx_opmode_blocking,
    )

##############################################################


def get_berry_requirements():
    """
    Purpose:
    ---
    This function get the berry requirements from the 'Theme_Config.json'
    ---

    Return:
    berry_requirements :    [dictionary]
            dictionary that contains all the details of the berry requirements
 
    ---
    Example call:
    get_berry_requirements():

    """
    berry_requirements_file = open("Theme_Config.json")
    berry_requirements_data = json.load(berry_requirements_file)
    berry_requirements = {
        "B": [
            int(berry_requirements_data["B"][0]),
            int(berry_requirements_data["B"][4]),
        ],
        "L": [
            int(berry_requirements_data["L"][0]),
            int(berry_requirements_data["L"][4]),
        ],
        "S": [
            int(berry_requirements_data["S"][0]),
            int(berry_requirements_data["S"][4]),
        ],
    }

    return berry_requirements


##############################################################


def get_room_nav_details(rooms_entry):
    """
    Purpose:
    ---
    It takes the rooms_entry points and returns a list with lists containing the room entry points 
    along with their details.


    Input argument:
    'rooms_entry' :     [list]
        List containing the coordinates of the entry points of the room

    Return:
    'nav_order'              :   [list[list]]
            list containing lists which contain three elements
            the first is the entry point to the room
            the second is the room number
            the third is the axis the entry of the room is facing

    Example call:
    get_room_nav_details(rooms_entry)
    """
    nav_order = []

    #room 2
    if rooms_entry[1][1]==5:
        nav_order.append([rooms_entry[1], 2, "x"])
    else:
        nav_order.append([rooms_entry[1], 2, "y"])
    
    #room 3
    if rooms_entry[2][1]==3:
        nav_order.append([rooms_entry[2], 3, "x"])
    else:
        nav_order.append([rooms_entry[2], 3, "y"]) 

    #room 4
    if rooms_entry[3][1]==3:
        nav_order.append([rooms_entry[3], 4, "x"])
    else:
        nav_order.append([rooms_entry[3], 4, "y"])       
    
    #room 1
    if rooms_entry[0][1]==5:
        nav_order.append([rooms_entry[0], 1, "x"])
    else:
        nav_order.append([rooms_entry[0], 1, "y"])


    return nav_order


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
    """
	Assuming we have two collection baskets
	
	"""

    wheel_joints = task_3.init_setup(client_id)
    berry_requirements = (
        get_berry_requirements()
    )  # returns dictionary with berry letters as key and [no. of berries required, cb no.] as values
    nav_order = get_room_nav_details(
        rooms_entry
    )  # returns 4 element list with each element being [(x,y),room no.,closest centre axis]
    current_pos = (4, 4)
    orientation = 0
    
    if nav_order[0][2] == "y":
        task_3.set_bot_orientation(client_id, wheel_joints, 90)
        orientation = 90

    # open gripper here once because the function starts with close and ends with open
    task_4.call_open_close(client_id, "open")
    
    #Goes to the rooms
    for room in nav_order:

        exact_pos_temp,current_pos,orientation = go_to_room(
            client_id, wheel_joints, current_pos, room, orientation
        )
        berry_requirements = task_4.task_4_primary(
            client_id, berry_requirements, current_pos, wheel_joints, exact_pos_temp,orientation
        )  # this does the picking and returns the list of berries yet to be picked
        current_pos = exit_room(client_id, wheel_joints, room, orientation)

        if (
            (berry_requirements["B"][0] == 0)
            and (berry_requirements["L"][0] == 0)
            and (berry_requirements["S"][0] == 0)
        ):
            break

    # Dropping
    #Checks if it is necessary to go to both the collection boxes before heading there
    for berry in berry_requirements:
        if berry_requirements[berry][1]==1:
            go_to_cb1(client_id, wheel_joints, current_pos, orientation)
            current_pos=(2,11)
            break
    for berry in berry_requirements:
        if berry_requirements[berry][1]==2:
            go_to_cb2(client_id, wheel_joints)
            break


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(2, 5), (6, 5), (5, 2), (2, 3)]  # example list of tuples

    ###############################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print("\nConnection to CoppeliaSim Remote API Server initiated.")
    print("Trying to connect to Remote API Server...")

    try:
        client_id = task_1b.init_remote_api_server()
        if client_id != -1:
            print("\nConnected successfully to Remote API Server in CoppeliaSim!")

            # Starting the Simulation
            try:
                return_code = task_1b.start_simulation(client_id)

                if (return_code == sim.simx_return_novalue_flag) or (
                    return_code == sim.simx_return_ok
                ):
                    print("\nSimulation started correctly in CoppeliaSim.")

                else:
                    print("\n[ERROR] Failed starting the simulation in CoppeliaSim!")
                    print(
                        "start_simulation function is not configured correctly, check the code!"
                    )
                    print()
                    sys.exit()

            except Exception:
                print(
                    "\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!"
                )
                print("Stop the CoppeliaSim simulation manually.\n")
                traceback.print_exc(file=sys.stdout)
                print()
                sys.exit()

        else:
            print("\n[ERROR] Failed connecting to Remote API server!")
            print("[WARNING] Make sure the CoppeliaSim software is running and")
            print(
                "[WARNING] Make sure the Port number for Remote API Server is set to 19997."
            )
            print(
                "[ERROR] OR init_remote_api_server function is not configured correctly, check the code!"
            )
            print()
            sys.exit()

    except Exception:
        print(
            "\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!"
        )
        print("Stop the CoppeliaSim simulation manually if started.\n")
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        # Running student's logic
        theme_implementation_primary(client_id, rooms_entry)

        try:
            return_code = task_1b.stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (
                return_code == sim.simx_return_novalue_flag
            ):
                print("\nSimulation stopped correctly.")

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    task_1b.exit_remote_api_server(client_id)
                    if (
                        task_1b.start_simulation(client_id)
                        == sim.simx_return_initialize_error_flag
                    ):
                        print(
                            "\nDisconnected successfully from Remote API Server in CoppeliaSim!"
                        )

                    else:
                        print("\n[ERROR] Failed disconnecting from Remote API server!")
                        print(
                            "[ERROR] exit_remote_api_server function is not configured correctly, check the code!"
                        )

                except Exception:
                    print(
                        "\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!"
                    )
                    print("Stop the CoppeliaSim simulation manually.\n")
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print("\n[ERROR] Failed stopping the simulation in CoppeliaSim server!")
                print(
                    "[ERROR] stop_simulation function is not configured correctly, check the code!"
                )
                print("Stop the CoppeliaSim simulation manually.")

            print()
            sys.exit()

        except Exception:
            print(
                "\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!"
            )
            print("Stop the CoppeliaSim simulation manually.\n")
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print(
            "\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!"
        )
        print("Stop the CoppeliaSim simulation manually if started.\n")
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    except KeyboardInterrupt:
        print("\n[ERROR] Script interrupted by user!")
