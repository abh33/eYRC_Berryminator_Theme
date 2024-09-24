'''
*****************************************************************************************
*
*                ===============================================
*                   Berryminator (BM) Theme (eYRC 2021-22)
*                ===============================================
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
'''


# Team ID:            [ 1010 ]
# Author List:        [ P TEJAS KRISHNA, AVVARU YASWANTH, G BHNAU CHANDANA, D KARTHIK SAINADH REDDY ]
# Filename:            task_3.py
# Functions:        
# Global variables:    
#                     [ x_coordinate, y_coordinate, v, x_centroid, y_centroid, client_id ]

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
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()



################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

# global variables
x_coordinate, y_coordinate = 4, 4
v = 3     # velocity = 3
client_id = 0
x_centroid, y_centroid = 255, 255

##############################################################

def get_bot_orientation(client_id):
    
    """
    Purpose: 
    ---
    Scans the qrcode and calculates the slope of vertical edge of qrcode to give the orientation of bot.
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        remote api connection 

    Returns:
    ---
    `angle`  :   [ float ] 
        returns the slope of the edge
    
    """
    
    ################################# OUR CODE ###############################
    
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image  = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    mask = cv2.inRange(transformed_image,(0,0,0),(200,200,200))
    thresholded = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    inverted = 255-thresholded 
    barcodes = decode(inverted)
    barcodes = decode(transformed_image)

    if(len(barcodes) == 0):
        angle = get_bot_orientation(client_id)

    for ele in barcodes:
        x,X = ele[3][3][0],ele[3][2][0]
        Y,y = ele[3][2][1],ele[3][3][1]
        # print(x,X,y,Y)

        if(X-x != 0):
            angle = (math.atan((Y-y)/(X-x)))*180/math.pi
        else:
            angle = 90
    # print(angle)
    
    ###########################################################################
    
    return angle
   
def get_qr_diagonal_slope(client_id):

    """ 
    
    Purpose:
    ---
    Scans the qr code and calculates the slope of diagnol points of qr code.
    
    Arguments:
    ---
    `client_id`    :   [ integer ]
        remote api connection 

    Returns:
    ---
    `angle`  :   [ float ] 
        returns the slope of the edge
        
    """
    
    ############################### OUR CODE ##################################
  
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image  = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    mask = cv2.inRange(transformed_image,(0,0,0),(200,200,200))
    thresholded = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    inverted = 255-thresholded 
    barcodes = decode(inverted)
    barcodes = decode(transformed_image)
    
    if(len(barcodes) == 0):
        angle = get_qr_diagonal_slope(client_id)

    for ele in barcodes:
        x,X = ele[3][3][0],ele[3][1][0]
        Y,y = ele[3][1][1],ele[3][3][1]
        # print(x,X,y,Y)
        angle = abs((math.atan((Y-y)/(X-x)))*180/math.pi)

    ##########################################################################
    
    return angle
 
def getCentroid(client_id):
    
    """
    Purpose:
    ---
    Reads the qr code and decides the centroid values of qr code for adjusting the position of bot whenever necessary
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        remote api connection
        
    Returns:
    ---
    `centroids`    :    [ list ] 
        centroid coordinates of the centroid.
        
    """
    ####################### GLOBAL VARIABLES #######################
    
    global x_centroid, y_centroid
        
    ############################ OUR CODE ##########################
    
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image  = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    mask = cv2.inRange(transformed_image,(0,0,0),(200,200,200))
    thresholded = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    inverted = 255-thresholded 
    barcodes = decode(inverted)
    barcodes = decode(transformed_image)
    centroids = (255,255)

    for ele in barcodes:
        cX = (ele[3][0][0] + ele[3][1][0] + ele[3][2][0] + ele[3][3][0])/4
        cY = (ele[3][0][1] + ele[3][1][1] + ele[3][2][1] + ele[3][3][1])/4
        x_centroid = cX
        y_centroid = cY
        centroids =(cX,cY)
    
    if(centroids == (255,255)):
        return [x_centroid, y_centroid]
    
    ##########################################################################
    
    return centroids

def getPos(client_id):
    
    """
    Purpose:
    ---
    Scans the qr code and gets the positon of the bot
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        remote api connection
        
    Returns:
    ---
    `qr_code`    :    [ list ] 
        qr code value after scanning and processing the data.
    
    """
    
    ############################ GLOBAL VARIABLES #######################

    global x_coordinate,y_coordinate
    
    ############################# OUR CODE ##############################
    
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image  = transform_vision_sensor_image(vision_sensor_image, image_resolution)

    qr_codes = detect_qr_codes(client_id,transformed_image)
    qr_codes = str(qr_codes)
    qr_codes = qr_codes[1:-1]
    qr_codes = qr_codes.replace(" ","")
    qr_codes = qr_codes.split(",")
    qr_code = []
    
    for ele in qr_codes:
        if(ele != "" or '' ):
            qr_code.append(int(ele))
    
    if(len(qr_code)!=0):
        x_coordinate,y_coordinate = qr_code[0],qr_code[1]
   
    qr_code=[x_coordinate,y_coordinate]           # Updates the data globally.
    
    #####################################################################
    
    return qr_code

def wheel_velocity_setting(client_id,v1,v2,v3,v4):
    
    """
    Purpose:
    ---
    Sets the individual velocities of bot wheels.
    
    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        remote api connection
        
    `v1`   :   [ float ] 
        velocity for wheel 1
        
    `v2`   :   [ float ] 
        velocity for wheel 2
        
    `v3`   :   [ float ] 
        velocity for wheel 3
        
    `v4`   :   [ float ] 
        velocity for wheel 4
    
    Returns:
    ---
    None
    
    """
    
    ################################ OUR CODE ###############################
    
    # Getting the joint handles of all wheels
    wheel_joints = init_setup(client_id)
    
    # Setting individual wheel velocities
    sim.simxSetJointTargetVelocity(client_id, wheel_joints[0], v1, sim.simx_opmode_streaming) # For front left wheel 
    sim.simxSetJointTargetVelocity(client_id, wheel_joints[1], v2, sim.simx_opmode_streaming) # For front right wheel
    sim.simxSetJointTargetVelocity(client_id, wheel_joints[2], v3, sim.simx_opmode_streaming) # For back left wheel
    sim.simxSetJointTargetVelocity(client_id, wheel_joints[3], v4, sim.simx_opmode_streaming) # For back right wheel
    
    ##########################################################################
    
    return None

def get_dist(client_id,cpos,tpos):
    
    """
    Purpose: 
    ---
    To get distance between two set of coordinates.

    Input Arguments:
    ---
    `cpos`   :    [ tuple ] 
        Current position
        
    `tpos`   :    [ tuple ]
        Target position

    Returns:
    ---
        float: Distance between two coordinates.
        
    """
    
    ########################## OUR CODE #########################
    
    return ((((tpos[0] - cpos[0] )**2) + ((tpos[1]-cpos[1])**2) )**0.5)

    #############################################################

def min_max(client_id,axis, min, max):
    
    """
    Purpose:
    ---
    Adjust the position of bot within the given pixel values of coordinate.
    
    Input Arguments:
    --- 
    `client_id`   :   [ integer ] 
        client_id after connection using remote api
        
    `axis`    :     [ integer ] 
        0 -> x-axis  and  1 -> y-axis 
        
    `min`     :     [ integer ]
        Minimum pixel value at which the respective centroid coordinate should fall in. 
        
    `max`     :     [ integer ]
        Maximum pixel value at which the respective centroid coordinate should fall in.
    
    Returns: 
    ---
    None
    
    """

    ########################### OUR CODE ########################
    
    # Getting the centroid pixel of qr code
    centroid = getCentroid(client_id)
    
    # ---------------- x axis position adjustment --------------- #
    if(axis == 0):
        
        while True:
            
            if(centroid[0] > 0 and centroid[0] < min):
                wheel_velocity_setting(client_id,-0.95,0.95,0.95,-0.95)
            
            if(centroid[0] > max  and centroid[0] < 512):
                wheel_velocity_setting(client_id,0.95,-0.95,-0.95,0.95)
            
            if(centroid[0] >= min and centroid [0] <= max ):
                wheel_velocity_setting(client_id,0,0,0,0)
                break
            # print(centroid)
            centroid = getCentroid(client_id)
    
    # ---------------- y axis position adjustment --------------- #
    if(axis == 1):
        
        while True:
           
            if(centroid[1] > 0 and centroid[1] < min):
                wheel_velocity_setting(client_id,1,1,1,1)
            
            if(centroid[1] > max  and centroid[1] < 512):
                wheel_velocity_setting(client_id,-1,-1,-1,-1)
           
            if(centroid[1] >= min and centroid [1] <= max ):
                wheel_velocity_setting(client_id,0,0,0,0)
                break
            # print(centroid)
            centroid = getCentroid(client_id)   

###################################################################

    return None

def adjPos(client_id,coord):
     
    """
    Purpose : 
    ---
    Adjust the position of bot depending on the entry coordinate of the room
    
    Input Arguments :
    --- 
    `client_id`    :     [ integer ] 
        remote api connection
        
    `coord`    :     [ tuple ] 
        Entry coordinates of the room in which bot is going to enter.
    
    Returns : 
    ---
    None
    
    """   

    ########################### OUR CODE ############################
    
    # ---------------- x axis position adjustment --------------- #
    
    if(coord == (0,5) or coord == (6,5) or coord == (6,3) ): 
        min_max(client_id,0, 171, 185) 
    
    if(coord == (2,5) or coord == (8,3) or coord == (2,3) ):
        min_max(client_id,0, 325, 340)
        
    # ---------------- y axis position adjustment --------------- #
    
    if( coord == (3,6) or coord == (3,0) ): 
        min_max(client_id, 0, 375,395)
        min_max(client_id,1, 375, 395)
        
    if( coord == (5,6)): 
        min_max(client_id,1, 385,395)
    
    if( coord == (5,8) ): 
        min_max(client_id,1, 150, 160)
    
    if( coord == (5,2) or coord == (3,2) ): 
        min_max(client_id,1, 150, 160)
    
    ##################################################################
    
    return None

def setVelocity(client_id, velocity):
    
    """
    Purpose:
    ---
    Set the velocity of the bot globally 
    
    Input Arguments:
    `client_id`    :      [ integer ] 
        remote api connection 

    `velocity`     :      [ float ]
    
    Returns: 
    ---
    None
    
    """
    
    ########################## OUR CODE ########################
    
    global v
    v = velocity
    
    ############################################################
    
    return None

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
    `client_id`     :  [ integer ]
        the client_id generated from start connection remote API, it should be stored in a global variable
    
    Example call:
    ---
    client_id = init_remote_api_server()
    
    """

    global client_id
    
    ##############    ADD YOUR CODE HERE    ##############
    
    # to disconnect the previous connections
    sim.simxFinish(-1) 
    
    # start fresh connections
    client_id = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
    
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
    `return_code`     :  [ integer ]
        the return code generated from the start running simulation remote API
    
    Example call:
    ---
    return_code = start_simulation()
    
    """
    return_code = -2

    ##############    ADD YOUR CODE HERE    ##############
    
    # start simulation
    return_code = sim.simxStartSimulation(client_id,sim.simx_opmode_oneshot)
    
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
    `vision_sensor_image`     :  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution`         :  [ list ]
        the image resolution returned from the get vision sensor image remote API
    `return_code`             :  [ integer ]
        the return code generated from the remote API
    
    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """


    return_code = 0

    ################### ADD YOUR CODE HERE    ##############
    
    _,cam_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, cam_handle, 0, sim.simx_opmode_blocking)
    
    ##################################################
    
    # print(vision_sensor_image, image_resolution, return_code)
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
    `vision_sensor_image`     :  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution`         :  [ list ]
        the image resolution returned from the get vision sensor image remote API
    
    Returns:
    ---
    `transformed_image`     :  [ numpy array ]
        the resultant transformed image array after performing above 4 steps
    
    Example call:
    ---
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    
    """

    transformed_image = None

    ##############    ADD YOUR CODE HERE    ##############
    
    # transforms vision sensor image to 3d array and flips it.
    arr = np.array(vision_sensor_image, dtype=np.uint8)
    arr = np.resize(arr, (512, 512, 3))
    transformed_image = arr[...,::-1]
    transformed_image = np.flip(transformed_image,0)
    
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
    `return_code`     :  [ integer ]
        the return code generated from the stop running simulation remote API
    
    Example call:
    ---
    return_code = stop_simulation()
    
    """

    return_code = -2

    ##############    ADD YOUR CODE HERE    ##############
    
    # to stop simulation
    return_code = sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)
    
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

    ##############    ADD YOUR CODE HERE    ##############
    
    # closes the connections
    sim.simxFinish(client_id)
    
    ##################################################

def detect_qr_codes(client_id,transformed_image):
    
    """
    Purpose:
    ---
    This function receives the transformed image from the vision sensor and detects qr codes in the image

    Input Arguments:
    ---
    
    `client_id`      :      [ integer ]
        remote api connection
        
    `transformed_image`     :  [ numpy array ]
        the transformed image array
    
    Returns:
    ---
    None
    
    Example call:
    ---
    detect_qr_codes()
    
    """

    ##############    ADD YOUR CODE HERE    ##############

    # does image processing and gets the data inside the qrcode and returns the qrcodes data

    qr_codes = []
    
    mask = cv2.inRange(transformed_image,(0,0,0),(155,155,155))
    thresholded = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    inverted = 255-thresholded 
    barcodes = decode(inverted)
    
    templist=[]
    
    for ele in barcodes:
        cX = (ele[3][0][0] + ele[3][1][0] + ele[3][2][0] + ele[3][3][0])/4
        cY = (ele[3][0][1] + ele[3][1][1] + ele[3][2][1] + ele[3][3][1])/4
        centroids =(cX,cY)
        templist = ele[0].decode()
        qr_codes = templist
        templist = []

    ##################################################

    return qr_codes

def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):

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

    ##############    ADD YOUR CODE HERE    ##############

    # depending on the parameters, this function will direct the bot , and decide it to move  whether to move 
    # forward, backward, right, left, forward diagonal, backward diagonal, clockwise and anticlockwise rotation 
    
        
    if(forw_back_vel != 0 and left_right_vel == 0 and rot_vel == 0): # + for forward , - for backward 
        wheel_velocity_setting(client_id,forw_back_vel,forw_back_vel,forw_back_vel,forw_back_vel)
    
    elif(left_right_vel != 0 and forw_back_vel == 0 and rot_vel == 0 ): # + for right side, - for left side
        wheel_velocity_setting(client_id,left_right_vel,-left_right_vel,-left_right_vel,left_right_vel)

    elif(rot_vel != 0 and left_right_vel ==0 and forw_back_vel == 0 ): # + for clock wise rotation, - for anti clock wise rotation
        wheel_velocity_setting(client_id,rot_vel,-rot_vel,rot_vel,-rot_vel)

    elif(rot_vel == 0 and left_right_vel ==0 and forw_back_vel == 0 ): # To stop the bot
        wheel_velocity_setting(client_id,0,0,0,0)

    elif(rot_vel == 0 and left_right_vel != 0 and forw_back_vel != 0  ): 
        
        if(left_right_vel<0 and forw_back_vel<0): # move to  Q3 
            wheel_velocity_setting(client_id, -(abs(left_right_vel)+abs(forw_back_vel)/2), 0, 0, -(abs(left_right_vel)+abs(forw_back_vel)/2))
        
        elif(left_right_vel<0 and forw_back_vel>0): # move to  Q2
            wheel_velocity_setting(client_id,0,  abs(left_right_vel)+abs(forw_back_vel)/2, abs(left_right_vel)+abs(forw_back_vel)/2, 0)
        
        elif(left_right_vel>0 and forw_back_vel > 0): # move to  Q1
            wheel_velocity_setting(client_id, (abs(left_right_vel)+abs(forw_back_vel)/2), 0, 0, (abs(left_right_vel)+abs(forw_back_vel)/2))
        
        elif(left_right_vel>0 and forw_back_vel<0): # move to  Q4
            wheel_velocity_setting(client_id,0, -(abs(left_right_vel)+abs(forw_back_vel)/2), -(abs(left_right_vel)+abs(forw_back_vel)/2), 0)

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

    ##############    ADD YOUR CODE HERE    ##############
    wheel_joints = []
    
    _,fl = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_oneshot)
    _,fr = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_oneshot)
    _,rl = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_oneshot)
    _,rr = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_oneshot)
    
    wheel_joints.append(fl)  #0
    wheel_joints.append(fr)  #1
    wheel_joints.append(rl)  #2
    wheel_joints.append(rr)  #3

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
    
    ########################## OUR CODE #########################
    
    return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")

    for index,joint_val in enumerate(joints_position):
        joints_position[index]=float(joint_val)

    #############################################################
    
    return joints_position
  
def nav_logic(targetPos, wheel_joints, client_id):
    
    """
    Purpose:
    ---
    This function should implement your navigation logic.
    
    Input Arguments:
    ---
    `targetPos`     :      [ list ]
        List containing the coordinates of the targert position.
        
    'wheel_joints`      :   [ list]
        Python list containing joint object handles of individual joints.
        
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ######################## OUR CODE ########################
    
    global v            # ------------ Accessing the global variable velocity `v`----------- #
    
    # Setting the bot velocity to 0 to stop the bot
    wheel_velocity_setting( client_id,0, 0, 0, 0)
    
    # Getting current position by scanning the qr code
    currPos = getPos(client_id)
    
    a = currPos[0] 
    b = currPos[1] 
    c = targetPos[0]
    d = targetPos[1]
    
    if(a == c and b > d): #1
        set_bot_movement(client_id, wheel_joints, -v ,0,0)
    
    elif(a == c and b < d): #2
        set_bot_movement(client_id, wheel_joints, v ,0,0)
    
    elif(b == d and a > c): #3
        set_bot_movement(client_id, wheel_joints, 0, -v, 0)
    
    elif(b == d and c > a): #4
        set_bot_movement(client_id, wheel_joints, 0, v, 0)
    
    elif(a < c and b < d ): # q1  #5
        set_bot_movement(client_id, wheel_joints, v, v, 0)
    
    elif(a < c and b > d ): # q4  #6
        set_bot_movement(client_id, wheel_joints, -v, v, 0)
    
    elif(a > c and b < d ): # q2  #7
        set_bot_movement(client_id, wheel_joints, v, -v, 0)
    
    elif(a > c and b > d ): # q3  #8
        set_bot_movement(client_id, wheel_joints, -v, -v, 0)
   
    else:  # For all other cases
        set_bot_movement(client_id, wheel_joints, v, v, 0)
    
    # To reduce velocity almost to zero of the bot after reaching the target coordinate
    while True:
        
        currPos = getPos(client_id)
        
        if(len(currPos) > 0):
            a = currPos[0] 
            b = currPos[1] 
            c = targetPos[0]
            d = targetPos[1]
            
            if(a == c and b == d ):
                set_bot_movement(client_id, wheel_joints, 0,0,0) 
                break
    
    ############################################################
    
    return None
    
def shortest_path(currPos, targetPos, wheel_joints, client_id):
    
    """
    Purpose:
    ---
    To find the shortest path on the given floor between the destination and source co-ordinates.
    
    Input Arguments:
    ---
    `currPos`     :      [ list ]
        List containing the coordinates of the current position.
        
    `targetPos`     :      [ list ]
        List containing the coordinates of the targert position.
        
    'wheel_joints`      :   [ list]
        Python list containing joint object handles of individual joints.
        
    `client_id`     :      [ integer ]
        remote api connection
        
    Returns:
    ---
    None
    
    """
    
    ##################### OUR CODE #####################
    
    """
    
    This function decides the path to go. 
    If the bot needs to move diagonal and forward, then this function will detect and call the nav_logic() function inorder to make the bot traverse.
    It calls the nav_logic() function twice if it needs to take a turn or call once, if it has to move straight.
    
    """

    a = currPos[0] 
    b = currPos[1] 
    c = targetPos[0]
    d = targetPos[1]

    if( a == c or b == d or abs(b-d) == abs(a-c) ): 
        targetPos = [c, d]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a > c and b < d and (math.atan((b-d)/(c-a)) * 180) / math.pi > 45 ):
        p= c
        q = a + b - p
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a > c and b < d and (math.atan((b-d)/(c-a)) * 180) / math.pi < 45 ):
        q = d
        p = a + b - q
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a < c and b > d and (math.atan((b-d)/(c-a)) * 180) / math.pi > 45 ):
        p = c
        q = a + b - p
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a < c and b > d and (math.atan((b-d)/(c-a)) * 180) / math.pi < 45 ):
        q = d
        p = a + b - q
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a > c and b > d and math.atan((d-b)/(c-a)) * 180 / math.pi > 45 ):
        p = c
        q = -a + b + p
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [ c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a > c and b > d and math.atan((d-b)/(c-a)) * 180 / math.pi < 45 ): 
        q = d
        p = a - b + q
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id)
        targetPos = [ c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a < c and b < d and math.atan((d-b)/(c-a)) * 180 / math.pi > 45 ):
        p = c
        q = -a + b + p
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id) 
        targetPos = [ c, d ]
        nav_logic( targetPos, wheel_joints, client_id)

    elif( a < c and b < d and math.atan((d-b)/(c-a)) * 180 / math.pi < 45 ): 
        q = d
        p = a - b + q		
        targetPos = [p, q]
        nav_logic( targetPos, wheel_joints, client_id) 
        targetPos = [c, d]
        nav_logic( targetPos, wheel_joints, client_id)

    ####################################################

def task_3_primary(client_id, target_points):
    
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
    
    ####################### OUR CODE ######################
    
    wheel_joints = init_setup(client_id)
    wheel_velocity_setting(client_id, 0, 0, 0, 0)
    
    # Going through all the target points
    for coord in target_points:
        cpos = getPos(client_id)
        tpos = list(coord)
        shortest_path(cpos,tpos,wheel_joints,client_id)
        
    #######################################################
    
if __name__ == "__main__":

    ##################################################
    # target_points is a list of tuples. These tuples are the target navigational co-ordinates
    # target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
    # example:
    # target_points = [(2,0),(3,6),(2,2),(5,8)]    # You can give any number of different co-ordinates
    target_points = [(2,0)]
    ##################################################
    ## NOTE: You are NOT allowed to make any changes in the code below ##

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

        task_3_primary(client_id, target_points)
        time.sleep(1)        

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
        print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
        
