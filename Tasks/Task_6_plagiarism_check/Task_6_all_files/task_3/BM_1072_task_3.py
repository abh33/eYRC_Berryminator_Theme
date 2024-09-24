"""
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
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
# Filename:			task_3.py
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


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


def dist(a, b):
    """
    Purpose:
    ---
    This function calculates the distance between two points, be it in either 2D or 3D space

    Input Arguments:
    ---
    `a`   :   (int,int..)
            tuple containing either two coordinate values or three

    `b`   :   (int,int..)
            tuple containing either two coordinate values or three

    Returns:
    ---
    `distance`  :   float
                calculated distance between a and b

    Example call:
    ---
    distance=dist(a, b)

    """
    if len(a) == 2:
        distance = math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    else:
        distance = math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )
    return distance


##############################################################


def return_ordered_polygon(polygon):
    """
    Purpose:
    ---
    This function returns the list of polygon values in the order bottom left, top left, top right, bottom right

    Input Arguments:
    ---
    `polygon`   :   [(int,int),(int,int),...]
                list of the coordinates of the four vertices of the qr code

    Returns:
    ---
    `ordered_polygon` 	:   [(int,int),(int,int),...]
            list of the coordinates of the four vertices of the qr code in the order bottom left, top left, top right, bottom right

    Example call:
    ---
    ordered_polygon=return_ordered_polygon(polygon)

    """
    # sorting wrt x
    x_ascending = sorted(polygon, key=lambda x: x[0])
    # sorting wrt y
    y_ascending = sorted(polygon, key=lambda x: x[1])

    ordered_polygon = [[], [], [], []]

    # arranging in bottom left, top left, top right, bottom right order
    for point in polygon:
        if ((point == x_ascending[0]) or (point == x_ascending[1])) and (
            (point == y_ascending[0]) or (point == y_ascending[1])
        ):
            ordered_polygon[0] = point
        elif ((point == x_ascending[0]) or (point == x_ascending[1])) and (
            (point == y_ascending[2]) or (point == y_ascending[3])
        ):
            ordered_polygon[1] = point
        elif ((point == x_ascending[2]) or (point == x_ascending[3])) and (
            (point == y_ascending[2]) or (point == y_ascending[3])
        ):
            ordered_polygon[2] = point
        else:
            ordered_polygon[3] = point

    return ordered_polygon


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
    sim.simxFinish(-1)  # just in case, close all opened connections
    client_id = sim.simxStart(
        "127.0.0.1", 19997, True, True, 5000, 5
    )  # Connect to CoppeliaSim

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
    sim.simxGetPingTime(client_id)
    return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

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

    return_code = 0

    ##############	ADD YOUR CODE HERE	##############
    _, objhandle = sim.simxGetObjectHandle(
        client_id, "vision_sensor_1", sim.simx_opmode_blocking
    )
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(
        client_id, objhandle, 0, sim.simx_opmode_blocking
    )

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
    np1d = np.array(vision_sensor_image).astype(np.uint8)
    x = int(image_resolution[0])
    y = int(image_resolution[1])
    np3d = np1d.reshape(x, y, 3)
    imgrgb = cv2.cvtColor(np3d, cv2.COLOR_BGR2RGB)
    transformed_image = np.flip(imgrgb, axis=0)

    ##################################################

    return transformed_image


def stop_simulation(client_id):
    """
    Purpose:
    ---
    This function should stop the running simulation in CoppeliaSim server.
    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
              It is already written in the main function.

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
    return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)

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
    sim.simxFinish(client_id)
    ##################################################


def detect_qr_codes(
    transformed_image, starting_tuple, target_tuple, operation, exact_pos, orientation
):

    """
    Purpose:
    ---
    This function receives the transformed image from the vision sensor and detects qr codes in the image, performing operations according
        to what is required.

    Input Arguments:
    ---
    `transformed_image` 	:  [ numpy array ]
            the transformed image array

    `starting_tuple`        :   (integer,integer)
            the tuple containing the coordinates of the starting point of the current path

    `target_tuple`        :   (integer,integer)
            the tuple containing the coordinates of the destination point of the current path

    `operation`         :   'string'
            the string which determines what function detect_qr_codes should perform. There are 5 possibilities-
            'basic detect' - checks if the target_tuple has entered the field of view of vision_sensor_1.
            'precision detect' - checks if the qr code has been aligned according to the specified exact_pos.
            'check rotation correction' - checks if the qr code is tilted beyond the permissible level indicated by exact_pos.
            'rotation correction' - this helps straighten the bot if it is tilted.
            'angle detect' - this calculates the angle between the centre of the bot and the centre of the qr code to help align them.

    `exact_pos`        : (integer, integer)
            tuple with two values which represent the alignment of the bot with respect to the qr code.
            Or this could take the value of an angle when calling 'check rotation correction'.

    `orientation`       : integer
            can have two values- 0: indicates that the bot is aligned according to the x axis of the scene.
                                90: indicates that the bot is aligned according to the y axis of the scene.

    Returns:
    ---
    True, False, None or theta depending on the operation

    Example call:
    ---
    value= detect_qr_codes(transformed_image, starting_tuple, target_tuple, operation, exact_pos, orientation)

    """

    ##############	ADD YOUR CODE HERE	##############

    detected_codes = decode(transformed_image)

    #Small correction because we upgraded the vision_sensor_1's resolution from 256 to 512
    if type(exact_pos)==tuple:
        exact_pos=(exact_pos[0]*2,exact_pos[1]*2)
        
    

    if operation == "basic detect":
        for code in detected_codes:
            if code.data.decode("utf-8") == str(target_tuple):
                return True
            else:
                return False



    elif operation == "precision detect":
        if len(detected_codes) >= 1:
            for code in detected_codes:
                x = code.rect[0] + code.rect[2] / 2
                y = code.rect[1] + code.rect[3] / 2

                if ((exact_pos[0] - 2) <= x <= (exact_pos[0] + 2)) and (
                    (exact_pos[1] - 2) < y < (exact_pos[1] + 2)
                ):
                    return True
                else:
                    return False



    elif operation == "check rotation correction":
        for code in detected_codes:
            ordered_polygon = return_ordered_polygon(code.polygon) #returns the coordinates of the vertices of the qr code in the order-
            # bottom left, top left, top right, bottom right
            if (
                abs(
                    (ordered_polygon[2][1] - ordered_polygon[1][1])
                    / (ordered_polygon[2][0] - ordered_polygon[1][0])
                )
                <= math.tan(exact_pos*math.pi/180)
            ):
                return False     # i.e. no correction needed
            else:
                return True



    elif operation == "rotation correction":
        for code in detected_codes:
            ordered_polygon = return_ordered_polygon(code.polygon)
            if (
                abs(
                    (ordered_polygon[2][1] - ordered_polygon[1][1])
                    / (ordered_polygon[2][0] - ordered_polygon[1][0])
                )
                <= 0.0524077793
            ):
                return False  # i.e. no correction needed

            elif (ordered_polygon[2][1] - ordered_polygon[1][1]) / (
                ordered_polygon[2][0] - ordered_polygon[1][0]
            ) > 0.0524077793:
                return "right"

            elif (ordered_polygon[2][1] - ordered_polygon[1][1]) / (
                ordered_polygon[2][0] - ordered_polygon[1][0]
            ) < -0.0524077793:
                return "left"
            else:
                return None



    #angle detect
    else:
        for code in detected_codes:
            x = code.rect[0] + code.rect[2] / 2
            y = code.rect[1] + code.rect[3] / 2
            theta = math.atan2(y - exact_pos[1], x - exact_pos[0])
            return theta

    ##################################################

def wheel_angle_calc(angle, orientation):

    """
    Purpose:
    ---
    This function takes the angle at which the bot needs to move and the current orientation of the bot
    and returns the required velocities of the wheels to achieve that.

    Input Arguments:
    ---
    `angle`         :   [ integer ]
            the angle of slope between the starting point and destination of the bot

    `orientation`      :   [ list]
            can have two values- 0: indicates that the bot is aligned according to the x axis of the scene.
                                90: indicates that the bot is aligned according to the y axis of the scene.


    Returns:
    ---

    `fr_rl'     :   [ float ]
            Desired velocity of the front right and rear left wheels

    `fl_rr'    :   [ float ]
            Desired velocity of the front right and rear right wheels

    Example call:
    ---
    fr_rl, fl_rr=wheel_angle_calc(angle, orientation)

    """

    fr_rl = math.sin(
        angle - (math.pi / 4) + (orientation * math.pi / 180)
    ) 
    fl_rr = math.sin(
        angle + (math.pi / 4) + (orientation * math.pi / 180)
    )

    return fr_rl, fl_rr

##################################################
def set_bot_movement(client_id, wheel_joints, fr, fl, rr, rl):

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

    `fr'     :   [ float ]
            Desired velocity of the front right wheel

    `fl'    :   [ float ]
            Desired velocity of the front right wheel

    `rr'    :   [ float ]
            Desired velocity of the rear right wheel

    `rl'    :   [ float ]
            Desired velocity of the rear left wheel

    Returns:
    ---
    None

    Example call:
    ---
    set_bot_movement(client_id, wheel_joints, 0.5, 0, 0, 0.5)

    """

    ##############	ADD YOUR CODE HERE	##############

    sim.simxPauseCommunication(client_id, True)

    sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[0], fr, sim.simx_opmode_oneshot
    )

    sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[1], fl, sim.simx_opmode_oneshot
    )

    sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[2], rr, sim.simx_opmode_oneshot
    )

    sim.simxSetJointTargetVelocity(
        client_id, wheel_joints[3], rl, sim.simx_opmode_oneshot
    )

    sim.simxPauseCommunication(client_id, False)
    ##################################################


def set_bot_orientation(client_id, wheel_joints, orientation):

    """
    Purpose:
    ---
    This function will take the required orientation of the bot and set it to that orientation.
    

    Input Arguments:
    ---
    `client_id`         :   integer
            the client id of the communication thread returned by init_remote_api_server()

    `wheel_joints`      : [integers]
            list containing the joint handles of the wheels in the order

    `orientation`       : integer
            can have two values- 0: indicates that the bot is aligned according to the x axis of the scene.
                                90: indicates that the bot is aligned according to the y axis of the scene.

    Returns:
    ---
    Example call:
    ---
    nav_logic(client_id, path, wheel_joints, exact_pos, orientation)
    
    """

    exact_pos = (128, 128)
    operation = "check rotation correction"
    vel = 150 * math.pi / 180
    _, vs1_handle = sim.simxGetObjectHandle(
        client_id, "vision_sensor_1", sim.simx_opmode_blocking
    )
    sim.simxGetVisionSensorImage(client_id, vs1_handle, 0, sim.simx_opmode_streaming)

    if orientation == 90:
        set_bot_movement(client_id, wheel_joints, -vel, vel, -vel, vel)
    else:
        set_bot_movement(client_id, wheel_joints, vel, -vel, vel, -vel)

    time.sleep(1)

    # loops infinitely and gets vision sensor image
    while True:

        (
            return_code,
            image_resolution,
            vision_sensor_image,
        ) = sim.simxGetVisionSensorImage(
            client_id, vs1_handle, 0, sim.simx_opmode_buffer
        )
        if (
            (return_code == sim.simx_return_ok)
            and (len(image_resolution) == 2)
            and (len(vision_sensor_image) > 0)
        ):
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution
            )
        else:
            continue

        if operation == "check rotation correction":
            correction = detect_qr_codes(
                transformed_image, _, _, operation, 7, orientation
            )
            if correction == False:
                operation = "rotation correction"
            else:
                continue

        # for making the bot straight
        elif operation == "rotation correction":

            vel = 50 * math.pi / 180
            if orientation == 90:
                set_bot_movement(client_id, wheel_joints, -vel, vel, -vel, vel)
            else:
                set_bot_movement(client_id, wheel_joints, vel, -vel, vel, -vel)

            correction = detect_qr_codes(
                transformed_image, _, _, operation, exact_pos, orientation
            )
            if correction == False:
                break
            elif correction == "right":
                set_bot_movement(client_id, wheel_joints, -vel, vel, -vel, vel)
            elif correction == "left":
                set_bot_movement(client_id, wheel_joints, vel, -vel, vel, -vel)
            else:
                continue

        else:
            continue

    set_bot_movement(client_id, wheel_joints, 0, 0, 0, 0)
    sim.simxGetVisionSensorImage(client_id, vs1_handle, 0, sim.simx_opmode_discontinue)

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
            list containing joint object handles of individual joints in the order forward right,forward left,rear right,rear left

    Example call:
    ---
    wheel_joints=init setup(client_id)

    """

    ##############	ADD YOUR CODE HERE	##############
    wheel_joints = []

    _, frhandle = sim.simxGetObjectHandle(
        client_id, "rollingJoint_fr", sim.simx_opmode_blocking
    )
    wheel_joints.append(frhandle)
    _, flhandle = sim.simxGetObjectHandle(
        client_id, "rollingJoint_fl", sim.simx_opmode_blocking
    )
    wheel_joints.append(flhandle)
    _, brhandle = sim.simxGetObjectHandle(
        client_id, "rollingJoint_rr", sim.simx_opmode_blocking
    )
    wheel_joints.append(brhandle)
    _, blhandle = sim.simxGetObjectHandle(
        client_id, "rollingJoint_rl", sim.simx_opmode_blocking
    )
    wheel_joints.append(blhandle)

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
        client_id, "combined_joint_position", sim.simx_opmode_blocking
    )
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")

    for index, joint_val in enumerate(joints_position):
        joints_position[index] = float(joint_val)

    return joints_position


def nav_logic(client_id, path, wheel_joints, exact_pos, orientation):

    """
    Purpose:
    ---
    This function will take the path, the required exact_pos value and the orientation and attempt to make the bot
        go to the destination with the right orientation and stop at the exact_pos.
    

    Input Arguments:
    ---
    `client_id`         :   integer
            the client id of the communication thread returned by init_remote_api_server()
            
    `path`        : [(integer,integer),(integer,integer)]
            a list with two tuples- the starting point and the destination point

    `exact_pos`        : (integer, integer)
            tuple with two values which represent the alignment of the bot with respect to the qr code.

    `orientation`       : integer
            can have two values- 0: indicates that the bot is aligned according to the x axis of the scene.
                                90: indicates that the bot is aligned according to the y axis of the scene.

    Returns:
    ---
    Example call:
    ---
    nav_logic(client_id, path, wheel_joints, exact_pos, orientation)
    
    """
    #basic initiation
    ang = math.atan2((path[1][1] - path[0][1]), (path[1][0] - path[0][0]))
    fvel = 800 * math.pi / 180  
    svel = 70 * math.pi / 180 
    fr_rl_original, fl_rr_original = wheel_angle_calc(
        ang, orientation
    )
    operation = "basic detect"


    set_bot_movement(
        client_id,
        wheel_joints,
        fr_rl_original * fvel,
        fl_rr_original * fvel,
        fl_rr_original * fvel,
        fr_rl_original * fvel,
    )
    _, vs1_handle = sim.simxGetObjectHandle(
        client_id, "vision_sensor_1", sim.simx_opmode_blocking
    )


    sim.simxGetVisionSensorImage(client_id, vs1_handle, 0, sim.simx_opmode_streaming)
    while True:

        (
            return_code,
            image_resolution,
            vision_sensor_image,
        ) = sim.simxGetVisionSensorImage(
            client_id, vs1_handle, 0, sim.simx_opmode_buffer
        )
        if (
            (return_code == sim.simx_return_ok)
            and (len(image_resolution) == 2)
            and (len(vision_sensor_image) > 0)
        ):
            transformed_image = transform_vision_sensor_image(
                vision_sensor_image, image_resolution
            )
        else:
            continue

        #if true means that the qr code has entered the vision sensor's field of view
        if (operation == "basic detect") and (
            detect_qr_codes(
                transformed_image, path[0], path[1], operation, exact_pos, orientation
            )
            == True
        ): 
            set_bot_movement(
                client_id,
                wheel_joints,
                fr_rl_original * svel,
                fl_rr_original * svel,
                fl_rr_original * svel,
                fr_rl_original * svel,
            )
            operation = "precision detect"

        #if true means that the centre of the bot is above the centre of the qr code
        elif operation == "precision detect":
            theta = detect_qr_codes(
                transformed_image,
                path[0],
                path[1],
                "angle detect",
                exact_pos,
                orientation,
            )
            if theta != None:
                fr_rl = math.sin(
                    -theta - (math.pi / 4)
                )
                fl_rr = math.sin(
                    -theta + (math.pi / 4)
                )
                set_bot_movement(
                    client_id,
                    wheel_joints,
                    fr_rl * svel,
                    fl_rr * svel,
                    fl_rr * svel,
                    fr_rl * svel,
                )

            if (
                detect_qr_codes(
                    transformed_image,
                    path[0],
                    path[1],
                    operation,
                    exact_pos,
                    orientation,
                )
                == True
            ):
                rotation_correction = detect_qr_codes(
                    transformed_image,
                    path[0],
                    path[1],
                    "check rotation correction",
                    4.5,
                    orientation,
                )
                if rotation_correction == False:
                    break
                else:
                    operation = "rotation correction"


        # ensures that the bot isn't tilted too much
        elif operation == "rotation correction":
            correction = detect_qr_codes(
                transformed_image, path[0], path[1], operation, exact_pos, orientation
            )
            if correction == False:
                break

            elif correction == "right":
                set_bot_movement(
                    client_id,
                    wheel_joints,
                    -20 * math.pi / 180,
                    20 * math.pi / 180,
                    -20 * math.pi / 180,
                    20 * math.pi / 180,
                )
            elif correction == "left":
                set_bot_movement(
                    client_id,
                    wheel_joints,
                    20 * math.pi / 180,
                    -20 * math.pi / 180,
                    20 * math.pi / 180,
                    -20 * math.pi / 180,
                )
            else:
                continue

        else:
            continue
    sim.simxGetVisionSensorImage(client_id, vs1_handle, 0, sim.simx_opmode_discontinue)
    set_bot_movement(client_id, wheel_joints, 0, 0, 0, 0)



def shortest_path(target_points):
    #OBSOLETE
    """
    Purpose:
    ---
    This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
    """

    source = (0, 0)
    target_points.insert(0, source)
    return target_points


def task_3_primary(client_id, target_points):
    #OBSOLETE
    """
    Purpose:
    ---

    # NOTE:This is the only function that is called from the main function and from the executable.

    Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic.
    The bot should traverses all the target navigational co-ordinates.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    `target_points`     : [ list ]
            List of tuples where tuples are the target navigational co-ordinates.

    Returns:
    ---

    Example call:
    ---
    target_points(client_id, target_points)

    """

    # gets joint handles
    wheel_joints = init_setup(client_id)

    # since this is supposed to be the euclidean path, just uses target points. But we need to add (0,0) at the beginning
    path = shortest_path(target_points)

    # looping through all the target points
    for i in range(len(path) - 2):
        nav_logic(client_id, [path[i], path[i + 1]], wheel_joints, False)

    nav_logic(
        client_id, [path[len(path) - 2], path[len(path) - 1]], wheel_joints, True
    )  # this is for the last point

    # after all the points have been reached the bot stops
    set_bot_movement(client_id, wheel_joints, 0, 0, 0, 0)


if __name__ == "__main__":
    #OBSOLETE
    ##################################################
    # target_points is a list of tuples. These tuples are the target navigational co-ordinates
    # target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
    # example:
    # target_points = [(2,3),(3,6),(11,11),(0,0)]    # You can give any number of different co-ordinates
    target_points = [(8, 11), (4, 5), (1, 4), (3, 1), (6, 1)]

    ##################################################
    ## NOTE: You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print("\nConnection to CoppeliaSim Remote API Server initiated.")
    print("Trying to connect to Remote API Server...")

    try:
        client_id = init_remote_api_server()
        if client_id != -1:
            print("\nConnected successfully to Remote API Server in CoppeliaSim!")

            # Starting the Simulation
            try:
                return_code = start_simulation(client_id)

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

        task_3_primary(client_id, target_points)
        time.sleep(1)

        try:
            return_code = stop_simulation(client_id)
            if (return_code == sim.simx_return_ok) or (
                return_code == sim.simx_return_novalue_flag
            ):
                print("\nSimulation stopped correctly.")

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    exit_remote_api_server(client_id)
                    if (
                        start_simulation(client_id)
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
            "\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!"
        )
        print("Stop the CoppeliaSim simulation manually if started.\n")
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
