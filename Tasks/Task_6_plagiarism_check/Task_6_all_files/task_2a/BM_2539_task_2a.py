'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 2A of Berryminator(BM) Theme (eYRC 2021-22).
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
# Filename:			task_2a.py
# Functions:		identify_berry, get_relative_position
# Global variables:
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os, sys
import traceback

##############################################################

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

def identify_berry(r, g, b):
    if r > 150 and g < 50:
        return 'Strawberry'
    elif r > 200 and g > 200 and b < 50:
        return 'Lemon'
    elif b > 150 and r < 50:
        return 'Blueberry'


def get_relative_position(x, y, d):
    h: float = (x + 1) / 512 - 0.5
    v: float = (y + 1) / 512 - 0.5
    far_clipping_plane = 2.00
    near_clipping_plane = 0.01
    # dis: float = near_clipping_plane + d * (far_clipping_plane - near_clipping_plane)
    dis: float = 0.02 + d * 2

    return h, v, dis


def get_distance(x1: float, y1: float, x2: float, y2: float):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


##############################################################

def get_vision_sensor_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's image array from the CoppeliaSim scene.

    Input Arguments:
    ---
    `client_id`    :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
        the handle of the vision sensor scene object

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

    sim_img_colour_rgb = 0
    sim_img_colour_grey = 1

    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(clientID=client_id,
                                                                                      sensorHandle=vision_sensor_handle,
                                                                                      options=sim_img_colour_rgb,
                                                                                      operationMode=sim.simx_opmode_blocking)

    ##################################################

    return vision_sensor_image, image_resolution, return_code


def get_vision_sensor_depth_image(client_id, vision_sensor_handle):
    """
    Purpose:
    ---
    This function takes the client id and handle of the vision sensor scene object as input
    arguments and returns the vision sensor's depth buffer array from the CoppeliaSim scene.
    Input Arguments:
    ---
    `client_id`               :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()
    `vision_sensor_handle`    :   [ integer ]
        the handle of the vision sensor scene object

    Returns:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
        the depth buffer array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
        the image resolution returned from the get vision sensor image remote API
    `return_code` 			:  [ integer ]
        the return code generated from the remote API

    Example call:
    ---
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
    """

    vision_sensor_depth_image = []
    image_resolution = []
    return_code = 0

    ##############	ADD YOUR CODE HERE	##############

    return_code, image_resolution, vision_sensor_depth_image = sim.simxGetVisionSensorDepthBuffer(clientID=client_id,
                                                                                                  sensorHandle=vision_sensor_handle,
                                                                                                  operationMode=sim.simx_opmode_blocking)

    ##################################################

    return vision_sensor_depth_image, image_resolution, return_code


def transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution):
    """
    Purpose:
    ---
    This function converts the depth buffer array received from vision sensor and converts it into
    a numpy array that can be processed by OpenCV
    This function should:
    1. First convert the vision_sensor_depth_image list to a NumPy array with data-type as float32.
    2. Since the depth image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
    the new NumPy array should then be resized to a 2-D (two dimensional) NumPy array.
    3. Flip the resultant image array about the appropriate axis. The resultant image NumPy array should be returned.

    Input Arguments:
    ---
    `vision_sensor_depth_image` 	:  [ list ]
        the image array returned from the get vision sensor image remote API
    `image_resolution` 		:  [ list ]
        the image resolution returned from the get_vision_sensor_depth_image() function

    Returns:
    ---
    `transformed_depth_image` 	:  [ numpy array ]
        the resultant transformed image array after performing above 3 steps

    Example call:
    ---
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)

    """

    transformed_depth_image = None

    ##############	ADD YOUR CODE HERE	##############

    # Converting list to Numpy array
    transformed_depth_image = np.array(vision_sensor_depth_image, dtype=np.float32)

    # Reshaping 1-D float32 array to 2-D (length, breadth) array (image)
    transformed_depth_image.shape = (image_resolution[1], image_resolution[0])
    # task_1b.view_image(transformed_depth_image)

    # Flipping the image about the Y-axis
    transformed_depth_image = np.fliplr(transformed_depth_image)
    # task_1b.view_image(transformed_depth_image)

    ##################################################

    return transformed_depth_image


def detect_berries(transformed_image, transformed_depth_image):
    """
    Purpose:
    ---
    This function takes the transformed image and transformed depth image as input arguments and returns
    the pixel coordinates and depth values in form of a dictionary.

    Input Arguments:
    ---
     `transformed_image` 	:  [ numpy array ]
         the transformed image array
     `transformed_depth_image` 	:  [ numpy array ]
         the transformed depth image array

    Returns:
    ---
    `berries_dictionary` 	:  [ dictionary ]
        the resultant dictionary with details of all the berries

    Example call:
    ---
    berries_dictionary = detect_berries(transformed_image, transformed_depth_image)

    """
    berries_dictionary = {}

    berries = ["Strawberry", "Blueberry", "Lemon"]

    res_strawberry = []  # list of strawberry positions
    res_lemon = []  # List of lemon positions
    res_blueberry = []  # list of blueberry positions

    res = []
    ##############	ADD YOUR CODE HERE	##############

    gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)

    ret, threshold_image = cv2.threshold(gray, 45, 255,
                                         cv2.THRESH_BINARY_INV)  # Checks for value above 45 and replaces it with 0 (black)

    # cv2.imshow("threshold_image", threshold_image)
    # cv2.imshow('gray image', gray)

    # using the findContours() function
    contours, hierarchy = cv2.findContours(threshold_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

    i = 0

    # loop for storing data of shapes
    for contour in contours:
        if i == 0:
            i = 1
            continue

        # finding centroid of berry
        # M = cv2.moments(contour)
        # x = None
        # y = None
        # if M['m00'] != 0.0:
        # 	x = int(M['m10'] / M['m00'])
        # 	y = int(M['m01'] / M['m00'])
        # x1, y1 = x, y
        x, y, w, h = cv2.boundingRect(contour)
        x += w // 2
        y += h // 2
        #
        # # comparing both centroid finding algorithms
        # # result: the values differ by 1
        # if x != x1:
        # 	print("Not matching x", x, x1)
        # if y != y1:
        # 	print("Not matching y", y, y1)

        (b, g, r) = transformed_image[y, x]
        berry_name = identify_berry(r, g, b)
        z = transformed_depth_image[y, x]

        if berry_name == "Strawberry":
            res_strawberry.append((x, y, z))
        elif berry_name == "Blueberry":
            res_blueberry.append((x, y, z))
        else:
            res_lemon.append((x, y, z))

    # Creating list of lists for each berry
    res.append(res_strawberry)
    res.append(res_blueberry)
    res.append(res_lemon)

    # Appending it into the dictionary
    for i in range(3):
        berries_dictionary[berries[i]] = res[i]

    # print(berries_dictionary)
    ##################################################
    return berries_dictionary


def detect_berry_positions(berries_dictionary):
    """
    Purpose:
    ---
    This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
    berries with respect to vision sensor. The final output is returned in another dictionary.

    Input Arguments:
    ---
    `berries_dictionary` 	:  [ dictionary ]
        the dictionary returned by detect_berries() function

    Returns:
    ---
    `berry_positions_dictionary` 	:  [ dictionary ]
        the resultant dictionary with details of 3D positions of all the berries

    Example call:
    ---
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)

    """
    berry_positions_dictionary = {}

    ##############	ADD YOUR CODE HERE	##############

    for berry_type in berries_dictionary.keys():
        berry_details_list = berries_dictionary[berry_type]
        berry_positions_list = []
        for index in range(len(berry_details_list)):
            x = berry_details_list[index][0]
            y = berry_details_list[index][1]
            d = berry_details_list[index][2]
            berry_positions_list.append(get_relative_position(x, y, d))
        berry_positions_dictionary[berry_type] = berry_positions_list

    ##################################################

    return berry_positions_dictionary


def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
    """
    Purpose:
    ---
    This function takes the transformed_image and the dictionaries returned by detect_berries()
    and  detect_berry_positions() functions. This function is already completed for your reference
    and will be helpful for debugging purposes.

    Input Arguments:
    ---
    `transformed_image` :	[ numpy array ]
            numpy array of image returned by cv2 library

    `berries_dictionary` 	:  [ dictionary ]
        the resultant dictionary with details of all the berries

    `berry_positions_dictionary` 	:  [ dictionary ]
        the resultant dictionary with details of 3D positions of all the berries

    Returns:
    ---
    `labelled_image` :	[ numpy array ]
            labelled image

    Example call:
    ---
    transformed_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
    """
    labelled_image = np.array(transformed_image)
    ######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########

    for berry_type in berries_dictionary.keys():
        berry_details_list = berries_dictionary[berry_type]
        berry_positions_list = berry_positions_dictionary[berry_type]
        for index in range(len(berry_details_list)):
            pixel_x, pixel_y, depth_val = berry_details_list[index]
            coordinates = (pixel_x, pixel_y)
            horizontal_displacement, vertical_displacement, distance_from_sensor = berry_positions_list[index]
            horizontal_displacement, vertical_displacement, distance_from_sensor = round(horizontal_displacement,
                                                                                         2), round(
                vertical_displacement, 2), round(distance_from_sensor, 2)
            cv2.putText(labelled_image, str((horizontal_displacement, vertical_displacement, distance_from_sensor)),
                        coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    return labelled_image
