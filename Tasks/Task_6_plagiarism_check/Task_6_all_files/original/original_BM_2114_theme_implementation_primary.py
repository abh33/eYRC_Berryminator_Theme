'''
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
'''


# Team ID:			BM_2114
# Author List:		Aathan A, Hakash MP, Lakshmipriya R, Lakshmi Sruthi K
# Filename:			theme_implementation.py
# Theme:            Berryminator
# Functions:		coordinates(), encoders(), set_bot_movement(), shortest_path(),
#                   collection_box(), room1(), room2(), room3(), room4(),
#                   balsumcal(), dropberries(), get_angle(), rotation(),
#                   newStop(), set_velocity(), nav_logic(), theme_implementation_primary()

# Global variables:	bal, theta, enc_count, cb_flag, prev_encoder, prev_stop_enc, prev_vel


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
from task_4 import *
from i import *
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


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

# global variables:
balance_berries = {}           # this has the number and type of berries not yet collected
theta = 0                      # angle of the bot with respect to home position.
enc_count = 0                  # count variable used in set_velocity().
prev_vel = [0, 0, 0, 0]        # has the previous velocity values set on calling set_velocity() each time.
prev_encoder = [0, 0]          # has the previous encoder values of front_left and front_right wheels on calling set_velocity() each time.
prev_stop_enc = [0, 0]         # has the encoder values of front_left and front_right wheels recorded while stopping on previous destination.
cb_flag = 0                    # flag variable used in theme_implementation_primary() to indicate which collection box to go to.


def coordinates(client_id):
    """
    Function name: `coordinates`

	Input Arguments:
	---
	`client_id`       :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Output:
	---
	`result`          :   [ tuple ]
		Python tuple containing the coordinates in the detected qr code.

    Logic:
	---
	This function will get the image from vision sensor, transform it, detect the qr code 
	in that image and change the retrieved tuple from string to integer and then return it.  
    If there's no qr code detected, the function will return (-1,-1)
	
	Example call:
	---
	coordinates(client_id)
	
	"""
    result = (-1, -1)

    # obtaining and transforming vision sensor image to detect qr code from it
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    code = detect_qr_codes(transformed_image)

    # extracting the co-ordinates from the string 'code'
    if(code != None):
        result = tuple(int(num) for num in code.replace('(', '').replace(')', '').replace('...', '').split(', '))
    
    return result


def encoders(client_id):
    """
    Function name: `Encoders`  

    Input Arguments:
    ---
    `client_id`           :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server()

    Output:
    ---
    'joints_position`     :   [ list]
            Python list containing the total distance travelled by all wheels.

    Logic:
    ---
    This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
    and return a list which contains the total distance travelled by all four wheels.  

    Example call:
    ---
    joints_position = encoders(client_id)

    """

    return_code, signal_value = sim.simxGetStringSignal(client_id, 'combined_joint_position', sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")
    if joints_position[0] != '':
        for index, joint_val in enumerate(joints_position):
            joints_position[index] = float(joint_val)
    else:
        return [0, 0, 0, 0]

    for i in range(len(joints_position)):
        joints_position[i] = joints_position[i] * 0.1/2  # distance travelled by wheels

    return joints_position


def set_bot_movement(client_id, w1, w2, w3, w4):
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

    `w1`                :   [ float ]
            Velocity for Front_Left wheel

    `w2`                :   [ float ]
        Velocity for Rear_Left wheel

    `w3`                :   [ float ]
        Velocity for Rear_Right wheel

    `w4`                :   [ float ]
        Velocity for Front_Right wheel
    
    Returns:
    ---
    None

    Example call:
    ---
    set_bot_movement(client_id, 1, 1, 1, 1)

    """

    wheel_joints = init_setup(client_id)
    rcrfl1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[0], w1, sim.simx_opmode_streaming)
    rcrrl1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[1], w2, sim.simx_opmode_streaming)
    rcrrr1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[2], w3, sim.simx_opmode_streaming)
    rcrfr1 = sim.simxSetJointTargetVelocity(client_id, wheel_joints[3], w4, sim.simx_opmode_streaming)


def shortest_path(x1, y1, x2, y2):
    """
    Purpose:
    ---
    This function finds the shortest path on the given floor between the destination and source co-ordinates.
    
    Input Arguments:
    ---
    `x1`         :   [ integer ]
            x-coordinate of source co-ordinates

    `y1`         :   [ integer ]
            y-coordinate of source co-ordinates

    `x2`         :   [ integer ]
            x-coordinate of destination co-ordinates

    `y2`         :   [ integer ]
            y-coordinate of destination co-ordinates

    Returns:
    ---
    `vx`         :   [ float ]
            forw_back velocity for wheels.

    `vy`         :   [ float ]
            left_right velocity for wheels.

    `r`          :   [ float ]
            distance between source and destination co-ordinates.  

    Example call:
    ---
    fb_vel, lr_vel, dist = shortest_path(x1, y1, x2, y2)

    """

    V = 6.5  # base bot velocity
    x = x2-x1
    y = y2-y1
    r = math.sqrt(x*x + y*y)
    s = x/r
    c = y/r
    Vx = V * s
    Vy = V * c

    # correction after rotation
    vx = Vx*math.cos(theta)-Vy*math.sin(theta)
    vy = Vx*math.sin(theta)+Vy*math.cos(theta)

    # setting the value to zero if it is negligible
    if abs(vx) < 0.0000001:
        vx = 0
    if abs(vy) < 0.0000001:
        vy = 0

    return vx, vy, r  # fb_vel, rl_vel, distance


def collection_box(num):
    """
    Purpose:
    ---
    This function returns the co-ordinates to be travelled to reach the collection boxes and return back.
    
    Input Arguments:
    ---
    `num`                   :   [ integer ]
            the number of the collection box (1 or 2)

    Returns:
    ---
    `target_points`         :   [ list ]
            the points to be travelled to.
 
    Example call:
    ---
    points = cb(num)

    """

    # collection_box_1
    if num == 1:
        target_points = [(4, 10), (1, 10), (4, 10), (4, 4)]

    # collection_box_2
    elif num == 2:
        target_points = [(4, 10), (7, 10)]

    return target_points


def room1(x2, y2):
    """
    Purpose:
    ---
    This function returns the co-ordinates to be travelled to reach the plant in room1 and return back.
    
    Input Arguments:
    ---
    `x2`         :   [ integer ]
            x-coordinate of room entry point

    `y2`         :   [ integer ]
            y-coordinate of room entry point

    Returns:
    ---
    [ list ]
            the points to be travelled to.

    `pos`        :   [ char ]
            the position in which bot should stop while entering.

    Example call:
    ---
    points, pos = room1(x2, y2)
    
    """

    pos = 'g'
    # (2, 5)
    if (x2, y2) == (2, 5):
        pos = 'l'
        l = [(x2, 4), (x2, 6)]
    # (3, 6)
    elif x2 == 3:
        pos = 'u'
        l = [(4, y2), (2, y2)]
    # (0, 5)
    else:
        l = [(x2, 4), (x2, 6)]

    return l+[(1, 7)]+l[::-1]+[(4, 4)], pos


def room2(x2, y2):
    """
    Purpose:
    ---
    This function returns the co-ordinates to be travelled to reach the plant in room2 and return back.
    
    Input Arguments:
    ---
    `x2`         :   [ integer ]
            x-coordinate of room entry point

    `y2`         :   [ integer ]
            y-coordinate of room entry point

    Returns:
    ---
    [ list ]
            the points to be travelled to.

    `pos`        :   [ char ]
            the position in which bot should stop while entering.

    Example call:
    ---
    points, pos = room2(x2, y2)
    
    """

    pos = 'g'
    # (6, 5)
    if y2 == 5:
        pos = 'u'
        l = [(x2, 4), (x2, 6)]
    # (5, 8)
    elif (x2, y2) == (5, 8):
        pos = 'g'  # should be r
        l = [(4, y2), (6, y2)]
    # (5, 6)
    else:
        pos = 'g'
        l = [(4, y2), (6, y2)]

    return l+[(7, 7)]+l[::-1]+[(4, 4)], pos


def room3(x2, y2):
    """
    Purpose:
    ---
    This function returns the co-ordinates to be travelled to reach the plant in room3 and return back.
    
    Input Arguments:
    ---
    `x2`         :   [ integer ]
            x-coordinate of room entry point

    `y2`         :   [ integer ]
            y-coordinate of room entry point

    Returns:
    ---
    [ list ]
            the points to be travelled to.

    `pos`        :   [ char ]
            the position in which bot should stop while entering.

    Example call:
    ---
    points, pos = room3(x2, y2)
    
    """

    pos = 'g'
    # (6, 3), (8, 3)
    if y2 == 3:
        l = [(x2, 4), (x2, 2)]
    # (5, 2)
    elif x2 == 5:
        pos = 'u'
        l = [(4, y2), (6, y2)]

    return l+[(7, 1)]+l[::-1]+[(4, 4)], pos


def room4(x2, y2):
    """
    Purpose:
    ---
    This function returns the co-ordinates to be travelled to reach the plant in room4 and return back.
    
    Input Arguments:
    ---
    `x2`         :   [ integer ]
            x-coordinate of room entry point

    `y2`         :   [ integer ]
            y-coordinate of room entry point

    Returns:
    ---
    [ list ]
            the points to be travelled to.

    `pos`       :   [ char ]
            the position in which bot should stop while entering.

    Example call:
    ---
    points, pos = room4(x2, y2)
    
    """

    pos = 'g'
    # (2, 3)
    if y2 == 3:
        pos = 'u'
        l = [(x2, 4), (x2, 2)]
    # (3, 2)
    elif (x2, y2) == (3, 2):
        pos = 'l'
        l = [(4, y2), (2, y2)]
    # (3, 0)
    else:
        l = [(4, y2), (2, y2)]

    return l+[(1, 1)]+l[::-1]+[(4, 4)], pos


def balsumcal(value):
    """
    Purpose:
    ---
    This function returns the number of berries not yet collected for the given collection box 
    from 'balance_berries' dictionary which contains the number and type of berries to be collected for each collection box.
    
    Input Arguments:
    ---
    `value`       :   [ string ]
            name of the collection box.

    Returns:
    ---
    `sum`         :   [ integer ]
            the number of berries not yet collected for the given collection box

    Example call:
    ---
    sum = balsumcal('CB1')
    
    """

    sum = 0
    for val in balance_berries[value]:
        sum += balance_berries[value][val]
    return sum


def dropberries(client_id):
    """
    Purpose:
    ---
    This function is called to drop all the collected berries in the collection box.
    
    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().

    Returns:
    ---
    None

    Example call:
    ---
    dropberries(client_id)
    
    """

    sim.simxSetStringSignal(client_id, 'drop', '1', sim.simx_opmode_oneshot)
    rc = sim.simxClearStringSignal(client_id, 'dropcom', sim.simx_opmode_oneshot)
    
    while True:
        return_code, signal_value = sim.simxGetStringSignal(client_id, 'dropcom', sim.simx_opmode_blocking)
        signal_value = signal_value.decode()

        if(signal_value == '1'):
            time.sleep(3)
            rc = sim.simxSetStringSignal(client_id, 'drop', '2', sim.simx_opmode_oneshot_wait)
        elif(signal_value == '2'):
            time.sleep(6)
            sim.simxSetStringSignal(client_id, 'drop', '3', sim.simx_opmode_oneshot)
        elif(signal_value == '3'):
            time.sleep(3)
            sim.simxSetStringSignal(client_id, 'drop', '4', sim.simx_opmode_oneshot)
        elif(signal_value == '4'):
            rc = sim.simxClearStringSignal(client_id, 'dropcom', sim.simx_opmode_oneshot_wait)
            rc = sim.simxClearStringSignal(client_id, 'drop', sim.simx_opmode_oneshot_wait)
            break


def get_angle(qrcode):
    """
    Purpose:
    ---
    This function is to calculate the angle of the qrcode read by the vision sensor.

    Input Arguments:
    ---
    `qrcode`         :   [ object ]
            the 'qrcode' is the object containing info about detected qr code 

    Returns:
    ---
    [ float ]
            it returns the angle of qr code with respect to it's original position 

    Example call:
    ---
    angle = get_angle(qrcode)

    """

    poly = qrcode.polygon  # retriving the 4 corner coodrinates of the qrcode
    angle = math.atan2(poly[1].y - poly[0].y, poly[1].x - poly[0].x)
    return angle - math.pi/2


def rotation(client_id, dir):
    """
    Purpose:
    ---
    This function is called to rotate the bot to 90 degrees in the direction specified.
    
    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().

    `dir`               :   [ string ]
            the direction in which the bot needs to be rotated (clockwise or anticlockwise).

    Returns:
    ---
    None

    Example call:
    ---
    rotation(client_id, 'c')
    
    """

    global theta
    #time.sleep(2)

    # this calculates the value of theta to be achieved and the velocity to be given
    if dir == 'c':
        theta += math.pi/2
        w = x = 1
        z = y = -1
    elif dir == 'ac':
        theta -= math.pi/2
        w = x = -1
        z = y = 1

    print('rotation\n')
    set_bot_movement(client_id, w, x, y, z)   #setting velocity for rotation
    flag = 0  # loop count variable

    while True:
        flag+=1
        # retriving the qr code image at each instance
        vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
        transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
        gray_img = cv2.cvtColor(transformed_image, 0)
        barcode = decode(gray_img)

        for obj in barcode:
            a = get_angle(obj)
            if flag>5:  # after 5 loops
                # stopping the bot after rotation
                if(dir == 'c' and math.degrees(abs(a)) > 87):
                    print('clockwise rotation stopped')
                    r = set_velocity(client_id, 0, 90, 0, 0, 0, 'r', 0, 0)
                    #time.sleep(1)
                    return None
                if(dir == 'ac' and math.degrees(abs(a)) < 4):
                    print('anticlockwise rotation stopped')
                    r = set_velocity(client_id, 0, 90, 0, 0, 0, 'r', 0, 0)
                    #time.sleep(1)
                    return None


def newStop(client_id, X, Y, pos='g'):
    """
    Purpose:
    ---
    This function is to stop the bot at the required position after reaching the destination co-ordinates.
    
    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().

    `X`                 :   [ integer ]
            x-coordinate of destination coordinates.

    `Y`                 :   [ integer ]
            y-coordinate of destination coordinates.

    `pos`               :   [ char ]
            the position where the bot needs to be stopped with respect to the vision sensor image.
            (l-left, r-right, c-centre, u-up, d-down)   

    Returns:
    ---
    [integer]
            0 - bot has not stopped; 1 - bot has stopped.

    Example call:
    ---
    rotation(client_id, 'c')
    
    """    
    
    # retriving the qr code image at each instance
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    gray_img = cv2.cvtColor(transformed_image, 0)
    barcode = decode(gray_img)

    for obj in barcode:
        (x, y, w, h) = obj.rect
        # calculating centroid of qr code at each instance
        a = (x + w//2)
        b = (y + h//2)
        code = coordinates(client_id)

        if code == (X, Y):
            if(pos == 'c' and a > 226-25 and a < 284+25 and b > 226-25 and b < 284+25):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('centre')
                return 1
            elif(pos == 'u' and b > 284+20):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('up')
                return 1
            elif(pos == 'd' and b < 226-50):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('down')
                return 1
            elif(pos == 'l' and a > 284+50 ):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('left')
                return 1
            elif(pos == 'r' and a < 226-50 ):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('right')
                return 1
            elif(pos == 'g'):
                set_bot_movement(client_id, 0, 0, 0, 0)
                print('general')
                return 1
    return 0


def set_velocity(client_id, flag, dist, forw_back_vel, left_right_vel, rot_vel, pos, x2, y2):
    """
    Purpose:
    ---
    This function determines the velocity to be set.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().

    `flag`              :   [ integer ]
            flag is 0 for the first time, 1 otherwise.

    `dist`              :   [ float ]
            the distance the bot needs to travel which is returned by shortest_path()

    `forw_back_vel'     :   [ float ]
            Desired forward/backward velocity of the bot

    `left_right_vel'    :   [ float ]
            Desired left/back velocity of the bot

    `rot_vel'           :   [ float ]
            Desired rotational velocity of the bot

    `pos`               :   [ char ]
            the position where the bot needs to be stopped with respect to the vision sensor image.
            (l-left, r-right, c-centre, u-up, d-down)

    `x2`                :   [ integer ]
            x-coordinate of destination co-ordinates

    `y2`                :   [ integer ]
            y-coordinate of destination co-ordinates

    Returns:
    ---
    [integer]
            0 - bot has not stopped; 1 - bot has stopped.

    Example call:
    ---
    set_velocity(client_id,wheel_joints,flag,dist,forw_back_vel,left_right_vel,k)
    
    """

    global prev_vel
    global prev_encoder
    global prev_stop_enc
    global enc_count
    i = 1
    
    # initialising when starting from a coordinate
    if flag == 0:
        prev_encoder[0] = 0
        prev_encoder[1] = 0
        enc_count = 0

    # getting encoder values for front two wheels
    joint_pos = encoders(client_id)
    fl = joint_pos[0]
    fr = joint_pos[3]

    abs_fl = abs(fl)
    abs_fr = abs(fr)

    # when the encoder values goes from positive to zero to negative or vice versa,
    # the prev value needs to be added to account for the distance travelled
    # enc_count = 0 is for adding it for the first time
    if ((prev_encoder[0] > 0 and fl < 0) or (prev_encoder[0] < 0 and fl > 0)) and enc_count == 0:
        abs_fl += prev_stop_enc[0]
        enc_count = 1
    if ((prev_encoder[1] > 0 and fr < 0) or (prev_encoder[1] < 0 and fr > 0)) and enc_count == 0:
        abs_fr += prev_stop_enc[1]
        enc_count = 2

    if enc_count == 1:
        abs_fl += prev_stop_enc[0]
    if enc_count == 2:
        abs_fr += prev_stop_enc[1]

    # the main part
    # after cover% of distance, we slow down by 1.5 times each time and call newStop to stop at the required pos
    # after stopping we update the prev values to use it next time and we return 1 to nav_logic function
    if dist>2:
        cover = 0.8
    else:
        cover = 0.65

    if enc_count == 0:
        condition = abs(abs_fl-prev_stop_enc[0]) > dist*cover or abs(abs_fr-prev_stop_enc[1]) > dist*cover
    else:
        condition = abs_fl > dist*cover or abs_fr > dist*cover

    if condition:
        #print('here', prev_vel[0], prev_vel[3])
        i = 1.5             
        stop = newStop(client_id, x2, y2, pos)

        if stop == 1:
            print('stopped\n')
            # these are the encoder values while stopping
            prev_stop_enc[0] = abs_fl
            prev_stop_enc[1] = abs_fr
            return 1
    ####################

    # this is for setting vel for the first time when starting from any coordinate
    if flag == 0:
        if pos == 'r':  # this part is to update prev values after rotation
            prev_stop_enc[0] = abs_fl
            prev_stop_enc[1] = abs_fr
        w1 = (left_right_vel + forw_back_vel + rot_vel)
        w2 = (left_right_vel - forw_back_vel + rot_vel)
        w3 = (left_right_vel + forw_back_vel - rot_vel)
        w4 = (left_right_vel - forw_back_vel - rot_vel)

    # this is the part where we slow down,
    # if i = 1 it doesnt slow down
    else:
        if abs(prev_vel[0]) < 1 and abs(prev_vel[3]) < 1:  # min velocity
            i = 1
        w1 = prev_vel[0]/i
        w2 = prev_vel[1]/i
        w3 = prev_vel[2]/i
        w4 = prev_vel[3]/i

    # these are the variables which has the previous encoder values each time we call set_velocity
    prev_encoder[0] = fl
    prev_encoder[1] = fr

    set_bot_movement(client_id, w1, w2, w3, w4)

    # these are the previous velocity values
    prev_vel[0] = w1
    prev_vel[1] = w2
    prev_vel[2] = w3
    prev_vel[3] = w4
    
    return 0


def nav_logic(client_id, target_points, p, pos):
    """
    Purpose:
    ---
    This function implements the navigation logic. It gets the distance and required velocities 
    from shortest_path() function and sets the velocity using set_velocity() function inside a while loop.
    Then it calls task_4_primary() when it reaches the plant racks to pluck berries and calls dropberries()
    when it reaches the collection boxes to deposit the collected berries.

    Input Arguments:
    ---
    `client_id`             :   [ integer ]
            the client id of the communication thread returned by init_remote_api_server().

    `target_points`         :   [ list ]
            List of tuples where tuples are the target navigational co-ordinates.

    `p`                     :   [ integer ]
            current list index of the target_points

    `pos`                   :   [ char ]
            the position where the bot needs to be stopped with respect to the vision sensor image.
            (l-left, r-right, c-centre, u-up, d-down)   

    Returns:
    ---
    None.

    Example call:
    ---
    nav_logic(client_id, target_points, p, pos)
    
    """

    global balance_berries

    # (x1, y1) is the source co-ordinates and (x2, y2) is the destination co-ordinates.
    (x1, y1) = (4, 4)
    if p > 0:
        (x1, y1) = target_points[p-1]
    (x2, y2) = target_points[p]

    fb_vel, lr_vel, dist = shortest_path(x1, y1, x2, y2)

    # initialising variables.
    flag = 0
    dist *= 0.53
    stopped = 0

    while True:
        # when we call set_velocity for the first time flag = 0 then flag = 1
        stopped = set_velocity(client_id, flag, dist, fb_vel, lr_vel, 0, pos, x2, y2)
        flag = 1

        if stopped == 1:  # if stoped == 1 the bot has already stopped
            # at the plant
            # centre() is to make the bot centered with respect to the vision_sensor image.
            # bal is the number of berries we havent collected yet
            if (x2, y2) == (1, 7):
                # centre(client_id)
                balance_berries = task_4_primary(client_id, '1')
            if (x2, y2) == (7, 7):
                # centre(client_id)
                balance_berries = task_4_primary(client_id, '2')
            if (x2, y2) == (7, 1):
                # centre(client_id)
                balance_berries = task_4_primary(client_id, '3')
            if (x2, y2) == (1, 1):
                # centre(client_id)
                balance_berries = task_4_primary(client_id, '4')

            # at the collection box
            if (x2, y2) == (1, 10) or (x2, y2) == (7, 10):
                # this drops the berries in the collection box
                centre(client_id)
                dropberries(client_id)
            if (x2, y2) == (4, 4) or (x2, y2) == (4, 10):
                centre(client_id)

            break


def theme_implementation_primary(client_id, rooms_entry):
    '''	
    Purpose:
	---
    This function is to navigate to all 4 rooms and the 2 collection boxes as required.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`rooms_entry`         :   [ list of tuples ]
		Room entry co-ordinate of each room in order.

	
	Returns:
	---
    None
	
	Example call:
	---
	theme_implementation_primary(client_id, rooms_entry)

    '''

    global cb_flag
    points = []

    # for traversing to all 4 rooms
    for i in range(len(rooms_entry)):
        (x2, y2) = rooms_entry[i]

        if i == 0:
            points, pos = room1(x2, y2)
        elif i == 1:
            while (theta) != math.pi/2:
                # rotating until required theta value is achieved
                rotation(client_id, 'c')
            points, pos = room2(x2, y2)
        elif i == 2:
            while (theta) != math.pi:
                rotation(client_id, 'c')
            points, pos = room3(x2, y2)
        elif i == 3:
            print('\n\nroom4')
            while (theta) != 3*math.pi/2:
                rotation(client_id, 'c')
            points, pos = room4(x2, y2)

        for j in range(len(points)):
            if j != 0:
                pos = 'g'
            nav_logic(client_id, points, j, pos)

    # for going to collection box
        points.clear()  # this is so that we dont go back to any of the rooms in case we dont go to collection box
        
        # collection box 1
        if cb_flag == 0:
            if balsumcal('CB1') == 0:  # this calculates if there are any berries to be collected
                while (theta)%(math.pi*2) != 0:
                    rotation(client_id, 'ac')
                points = collection_box(1)
                cb_flag = 1
        # collection box 2
        elif cb_flag == 1:
            if balsumcal('CB2') == 0:
                while (theta)%(math.pi*2) != 0:
                    if theta == 3*math.pi/2:
                        rotation(client_id, 'c')
                    else:
                        rotation(client_id, 'ac')
                points = collection_box(2)
                cb_flag = 2

        if len(points) != 0:
            pos = 'g'
            for j in range(len(points)):
                nav_logic(client_id, points, j, pos)
            # the end
            if cb_flag == 2:
                print('end')
                set_bot_movement(client_id, 0, 0, 0, 0)
                #time.sleep(3)
                break


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(0, 5), (6, 5), (5, 2), (3, 0)]     # example list of tuples

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
