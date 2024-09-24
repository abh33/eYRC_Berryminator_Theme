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


# Team ID:			[ BM-1489 ]
# Author List:		[ Shaik, Gurkirat, Kaushik, Harika ]
# Filename:			theme_implementation.py
# Functions:
# Global variables:
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################


from http import client
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
import task_1b
import task_2a
import task_3
import task_4

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
plant_pos = [(1, 7), (7, 7), (7, 1), (1, 1)]

##################################################################################


def rotate_theta(client_id, angle, use_vision_sensor):
    wheel_joints = task_3.init_setup(client_id)

    initial_theta = task_3.encoders(client_id)
    separation_width = 0.3170/2
    separation_length = 0.4760/2
    radius = 0.05
    omega_bot = 0.3
    vel_fl = (1/radius)*(0+0 -
                         (separation_width+separation_length)*omega_bot)
    angle_of_wheel = (angle/omega_bot)*vel_fl

    error = abs(angle_of_wheel)
    sum = 0

    kp = -0.33
    ki = 0
    kd = -0.035
    if use_vision_sensor:
        kp = 0.06
        ki = 0
        kd = 0
    start = time.time()
    prev_error = abs(initial_theta[0]+angle_of_wheel) - \
        abs(task_3.encoders(client_id)[0])
    max_blue = 0
    max_straw = 0
    max_lem = 0
    return_code, vision_sensor2_handle = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    while True:
        # start = time.time()

        if use_vision_sensor:
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
            # print(berry_positions_dictionary)
            max_blue = max(
                len(berry_positions_dictionary["Blueberry"]), max_blue)
            max_lem = max(len(berry_positions_dictionary["Lemon"]), max_lem)
            max_straw = max(
                len(berry_positions_dictionary["Strawberry"]), max_straw)
        final_theta = task_3.encoders(client_id)
        error = abs(initial_theta[0]+angle_of_wheel)-abs(final_theta[0])
        step = time.time()-start
        sum += error*step
        differential = (error - prev_error)/(step)
        prev_error = error
        omega_bot = kp*error + kd*differential
        start = time.time()
        # print(error, sum, differential)
        task_3.set_bot_movement(client_id, wheel_joints, 0, 0, omega_bot)
        if abs(error) < 0.06:
            break
    task_3.set_bot_movement(client_id, wheel_joints, 0, 0, 0)
    if use_vision_sensor:
        return [max_straw, max_lem, max_straw]


def scan360(client_id):
    t = True
    r1 = rotate_theta(client_id, math.pi/2, t)
    r2 = rotate_theta(client_id, math.pi/2, t)
    r3 = rotate_theta(client_id, math.pi/2, t)
    r4 = rotate_theta(client_id, math.pi/2, t)

    fruit_rooms = [r1, r4, r3, r2]
    # print(fruit_rooms)
    return fruit_rooms
##############################################################


def open_gates(client_id, berry):
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'basket', sim.sim_scripttype_childscript, 'open_gates', [], [], berry, emptybuff, sim.simx_opmode_blocking)
    # print(berry)


def close_gates(client_id, berry):
    emptybuff = bytearray()
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(
        client_id, 'basket', sim.sim_scripttype_childscript, 'close_gates', [], [], berry, emptybuff, sim.simx_opmode_blocking)
    # print(berry)


def to_gate(client_id, qr_code, wheel_joints, start, target_points, transformed_image1, array1, array):
    # print("target_points")
    # print(target_points)
    for point in target_points:
        vision_sensor_image, image_resolution, return_code = task_3.get_vision_sensor_image(
            client_id)
        transformed_image = task_3.transform_vision_sensor_image(
            vision_sensor_image, image_resolution)
        start = task_3.detect_qr_codes(transformed_image)
        start = (int(start[0]), int(start[1]))
        if (point in plant_pos) or (start in plant_pos):
            # print("in")
            # print("start")
            # print(start)
            # print(point)
            # point = [point]
            route = task_3.nav_logic_in_room(client_id, qr_code, wheel_joints, start,
                                             [point], transformed_image1, array1, False)
        else:
            # print("out")
            # print("start")
            # print(start)
            # print(point)
            # start = (start[0], start[1])
            route = task_3.nav_logic(client_id, qr_code, wheel_joints, start,
                                     [point], transformed_image1, array, True)
        # print(route)
        start = point


def calculate_distance(start, goal, rooms_entry, array1, array):
    dist = 0
    plant_pos = [(1, 7), (7, 7), (7, 1), (1, 1)]

    for point in goal:
        if (point in plant_pos) or (start in plant_pos):
            # print("in")
            # print(array1)
            route = task_3.shortest_path(start, point, array1, True)
        else:
            # print("out")
            # print(array)
            route = task_3.shortest_path(start, point, array, True)
        dist = dist+len(route)
        start = point
    return dist


def combinations(lst, k):

    if k == 0:
        return [[]]

    l = []
    for i in range(0, len(lst)):

        m = lst[i]
        remLst = lst[i + 1:]

        for p in combinations(remLst, k-1):
            l.append([m]+p)

    return l


'''
Following function returns all possible room combinations based of priority order
'''


def distance_order(total_cases, box_coor, array1, array, rooms_entry):
    # global rooms_entry
    dis = []
    for case in total_cases:
        path = []
        case.reverse()
        for room in case:
            path.append(rooms_entry[room-1])
            path.append(plant_pos[room-1])
            path.append(rooms_entry[room-1])
        path.append(box_coor)

        dis.append(calculate_distance(
            (4, 4), path, rooms_entry, array1, array))
    return dis


def berry_cases(no_of_rooms, box_coor, array1, array, rooms_entry):
    plant_pos = [(1, 7), (7, 7), (7, 1), (1, 1)]
    berry_set = 1
    berry_list = []
    total_cases = []
    for i in range(no_of_rooms):
        berry_list.append(i+1)
    for i in range(no_of_rooms):
        all_cases = combinations(berry_list, i+1)
        total_cases = total_cases+all_cases

    room_pref = total_cases[0:4]
    pref_order = distance_order(
        room_pref, box_coor, array1, array, rooms_entry)
    z = zip(pref_order, room_pref)
    z = list(z)
    sorted_z = sorted(z, key=lambda x: x[0])
    room_pref = []

    for case in sorted_z:
        room_pref.append(case[1][0])
    # room_pref2=[]
    some_list = [0]*len(room_pref)
    for i in range(len(room_pref)):
        some_list[room_pref[i]-1] = i+1
    # room_pref2.append(room_pref[])
    room_pref = some_list

    dis = distance_order(total_cases, box_coor, array1, array, rooms_entry)
    zipped = zip(dis, total_cases)
    zipped = list(zipped)
    sorted_zip = sorted(zipped, key=lambda x: x[0])
    total_cases = []
    for case in sorted_zip:
        total_cases.append(case[1])
    for i in range(len(total_cases)):
        for j in range(len(total_cases[i])):
            total_cases[i][j] = room_pref[total_cases[i][j]-1]
        total_cases[i].sort(reverse=True)

    return room_pref, total_cases


'''
Following function selects all room combinations which satisfy required berry count
'''


def select_rooms(data, order, full_order, count):
    possibilities = []
    for i in range(len(full_order)):
        sum = 0
        for j in range(len(full_order[i])):
            pos = order.index(full_order[i][j])
            temp = data[pos]
            sum = sum+temp[1]
        if sum >= count:
            possibilities.append(full_order[i])

    rooms = []
    for i in range(len(possibilities)):
        k = []
        for j in range(len(possibilities[i])):
            pos = order.index(possibilities[i][j])
            temp = data[pos]
            k.append(temp[0])
        rooms.append(k)

    for i in range(len(rooms)):
        rooms[i].sort(reverse=True)

    return rooms


'''
Following recursive function finalises the rooms which the bot should enter
'''


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


def final_room_set(l1, l2, l3):
    k = []
    for x in l1:
        if x in l2:
            k.append(x)
    v = []
    for x in k:
        if x in l3:
            v.append(x)
    final_set = []
    for i in range(len(v[0])):
        final_set.append(int(v[0][i][1:]))
    return final_set

##############################################################


def open_gates_final(client_id, box_no, zip_with_type_and_box):
    # print(zi)
    if zip_with_type_and_box['S'] == box_no:
        open_gates(client_id, 'S')
    if zip_with_type_and_box['L'] == box_no:
        open_gates(client_id, 'L')
    if zip_with_type_and_box['B'] == box_no:
        open_gates(client_id, 'B')


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
    # wheel_joints = task_3.init_setup(client_id)
    # task_3.get_closer(client_id, wheel_joints, 1, True)
    # time.sleep(5)
    # open_gates(client_id, 'L')
    ############################DECODING JSON FILE#########################################
    obj = open("Theme_Config.json")
    data = json.load(obj)
    # print(type(data))
    berry_quantity = []
    box_data = []
    total_blueberries = int(data["B"][0])
    total_lemons = int(data["L"][0])
    total_strawberries = int(data["S"][0])
    berry_quantity.append(total_strawberries)
    berry_quantity.append(total_lemons)
    berry_quantity.append(total_blueberries)
    index1 = (data.keys())
    index = []
    if 'S' in index1:
        index.append('S')
    if 'L' in index1:
        index.append('L')
    if 'B' in index1:
        index.append('B')

    zip_with_type = zip(index, berry_quantity)
    box_data.append(int(data["S"][4]))
    box_data.append(int(data["L"][4]))
    box_data.append(int(data["B"][4]))
    zip_with_box = zip(box_data, berry_quantity)
    zip_with_type_and_box = zip(index, box_data)
    zip_with_type_and_box = {index[i]: box_data[i] for i in range(len(index))}
    zip_with_type = {index[i]: berry_quantity[i] for i in range(len(index))}
    # print(zip_with_type_and_box)
    #######################################################################################
    array = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1],
        [1, 1, 1, 1, 0, 1, 1, 1, 1]])
    az = np.ones((9, 3))

    array1 = np.array([
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 0, 0, 0]])
    array = np.flip(array, axis=0)
    array1 = np.flip(array1, axis=0)
    # array = np.flip(array, axis=1)
    array = np.transpose(array)
    array1 = np.transpose(array1)
    array = np.concatenate((array, az), axis=1)
    array1 = np.concatenate((array1, az), axis=1)
    # array1 = np.flip(array1, axis=0)
    array = np.flip(array, axis=0)
    # array = np.zeros((12, 12))
    for point in rooms_entry:
        array1[point[0]][point[1]] = 0
        array[point[0]][point[1]] = 0
        if point[0]+1 < array1.shape[0] and point[1]+1 < array1.shape[1]:
            array1[point[0]+1][point[1]+1] = 1
        if point[0]+1 < array1.shape[0] and point[1]-1 > 0:
            array1[point[0]+1][point[1]-1] = 1
        if point[0]-1 > 0 and point[1]+1 < array1.shape[1]:
            array1[point[0]-1][point[1]+1] = 1
        if point[0]-1 > 0 and point[1]-1 > 0:
            array1[point[0]-1][point[1]-1] = 1
    list = []
    list.append(rooms_entry[0])
    wheel_joints = task_3.init_setup(client_id)
    task_3.set_bot_movement(client_id, wheel_joints, 0, 0, 0)
    # task_3.get_closer(client_id, wheel_joints, 1, True)
    vision_sensor_image1, image_resolution1, return_code1 = task_3.get_vision_sensor_image(
        client_id)
    transformed_image1 = task_3.transform_vision_sensor_image(
        vision_sensor_image1, image_resolution1)
    qr_code = task_3.detect_qr_codes(transformed_image1)

    return_code, handle_prox_sensor = sim.simxGetObjectHandle(
        client_id, 'RG2_attachProxSensor', sim.simx_opmode_blocking)
    # open_gates(client_id, 'L')
    wheel_joints = task_3.init_setup(client_id)
    vision_sensor_image1, image_resolution1, return_code1 = task_3.get_vision_sensor_image(
        client_id)
    transformed_image1 = task_3.transform_vision_sensor_image(
        vision_sensor_image1, image_resolution1)
    qr_code = task_3.detect_qr_codes(transformed_image1)
    room_no = 0

    b1 = [(1, 10)]
    b2 = [(7, 10)]

    r1_to_b = [rooms_entry[0], plant_pos[0], rooms_entry[0], b1]
    no_of_rooms = 4
    types_of_berries = 3
    no_of_boxes = 2
    total_berries = scan360(client_id)
    # total_berries = [[2, 2, 2], [2, 2, 2], [2, 2, 2], [2, 2, 2]]

    berry_data = []
    box_coor = []
    box_coor.append((2, 11))  # dispose coordinate for box 1
    box_coor.append((6, 11))  # dispose coordinate for box 2
    orders = []
    full_orders = []

    for i in range(no_of_boxes):
        a1, a2 = berry_cases(
            no_of_rooms, box_coor[i], array1, array, rooms_entry)
        orders.append(a1)
        full_orders.append(a2)

    for i in range(types_of_berries):
        temp = []
        for j in range(no_of_rooms):
            temp.append([('r'+str(j+1)), total_berries[j][i]])
        berry_data.append(temp)

    all_possibilities = []
    for i in range(types_of_berries):
        box_number = box_data[i]
        all_possibilities.append(select_rooms(
            berry_data[i], orders[box_number-1], full_orders[box_number-1], berry_quantity[i]))

    l1 = all_possibilities[0]
    l2 = all_possibilities[1]
    l3 = all_possibilities[2]
    x = final_room_set(l1, l2, l3)
    # print(x)
    #######################################################################
    return_code, vision_sensor2_handle = sim.simxGetObjectHandle(
        client_id, 'vision_sensor_2', sim.simx_opmode_blocking)

    # print(berry_positions_dictionary)
    box_x = 0.1917
    box_y = -0.23302
    # slb
    box = [[box_x, box_y, -0.35],
           [box_x, box_y, -0.2403], [box_x, box_y, -0.1383]]
    default_pos = [0, -1.2, -0.12]
    coor = []
    # 0 strawberry
    # 1 Lemon
    # 2 Blueberry
    fruit_list = ["Strawberry", "Lemon", "Blueberry"]
    task_4.call_open_close(client_id, "open")

    start = (4, 4)
    target_points = []
    req_berries = [zip_with_type['S'], zip_with_type['L'], zip_with_type['B']]
    for room in x:
        target_points.append(rooms_entry[room-1])
        # target_points.append(plant_pos[room-1])
        to_gate(client_id, qr_code, wheel_joints, start,
                target_points, transformed_image1, array1, array)
        target_points = []

        target_points.append(plant_pos[room-1])
        to_gate(client_id, qr_code, wheel_joints, start,
                target_points, transformed_image1, array1, array)
    ###################berry picking######################
        task_3.qr_code_pid(client_id, wheel_joints, 0)
        early_orien = task_3.get_curr_orientation(client_id)
        while True:
            task_3.qr_code_pid(client_id, wheel_joints, 0)
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
            fruit_list = ["Strawberry", "Lemon", "Blueberry"]
            found = False
            if len(berry_positions_dictionary["Blueberry"]) + len(berry_positions_dictionary["Strawberry"]) + len(berry_positions_dictionary["Lemon"]) == sum(total_berries[room-1]):
                if berry_positions_dictionary["Blueberry"][0][2] < 0.6:
                    found = True
            # print(found)
            # print("F")
            if found:
                break
            task_3.rotate_theta1(client_id, math.pi/2, False)

        task_3.qr_code_pid(client_id, wheel_joints, 0)

        ######################################################
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
        # print(berry_positions_dictionary)
        zip_with_type = {index[i]: berry_quantity[i]
                         for i in range(len(index))}
        # print(zip_with_type)
        for i in range(len(zip_with_type)):
            if 'S' in zip_with_type.keys() and i == 0:
                type2 = 'Strawberry'
                typ = 'S'
            elif 'L' in zip_with_type.keys() and i == 1:
                type2 = 'Lemon'
                typ = 'L'
            elif 'B' in zip_with_type.keys() and i == 2:
                type2 = 'Blueberry'
                typ = 'B'

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
            # print(berry_positions_dictionary)

            x, y, z = berry_positions_dictionary[type2][0]
            flag = 0
            curr_orien = task_3.get_curr_orientation(client_id)
            # east
            # fosho
            # done
            curr_north_south = False
            if curr_orien == 0:
                # print("east")
                if x > 0.2:
                    # print("right")
                    flag = -1
                    curr_north_south = True
                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)

                if x < -0.2:
                    # print("left")
                    flag = 1
                    curr_north_south = True
                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)
            # north
            if curr_orien == 90:
                # print("north")
                curr_north_south = True
                if x > 0.2:
                    # print("right")
                    flag = -1

                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)

                if x < -0.2:
                    # print("left")
                    flag = 1

                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)
            # west
            # fosho
            # done
            if curr_orien == 180:
                # print("south")
                curr_north_south = True
                if x > 0.2:
                    # print("right")
                    flag = -1

                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)

                if x < -0.2:
                    # print("left")
                    flag = 1
                    curr_north_south = True
                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)
            # south
            if curr_orien == 270:
                curr_north_south = True
                # print("south")
                if x > 0.2:
                    # print("right")
                    flag = -1

                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)

                if x < -0.2:
                    # print("left")
                    flag = 1
                    # curr_north_south = False
                    task_3.get_closer(client_id, wheel_joints,
                                      flag, curr_north_south)

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

            j = 0
            while req_berries[i] > 0 and total_berries[room - 1][i] > 0:

                x, y, z = berry_positions_dictionary[type2][j]
                send_identified_berry_data(client_id, type2, x, y, z)
                coor = [x, y, z]
                # task_3.qr_code_pid(client_id, wheel_joints, 0)
                task_3.set_bot_movement(client_id, wheel_joints, 0, 0, 0)
                task_4.reachTarget(client_id, coor)
                task_4.call_open_close(client_id, "close")
                task_4.reachTarget(client_id, default_pos)
                task_4.reachTarget(client_id, box[fruit_list.index(type2)])
                task_4.call_open_close(client_id, "open")
                task_4.reachTarget(client_id, default_pos)
                total_berries[room-1][i] -= 1
                req_berries[i] -= 1
                j += 1

            # if x > 0.2:
            #     task_3.get_closer(client_id, wheel_joints, 1)

            # if x < -0.2:
            #     task_3.get_closer(client_id, wheel_joints, -1)
            if flag > 0:
                task_3.get_closer(
                    client_id, wheel_joints, -1, curr_north_south)
            elif flag < 0:
                task_3.get_closer(client_id, wheel_joints, 1, curr_north_south)

        ########################################################

        while True:
            if task_3.get_curr_orientation(client_id) == early_orien:
                break
            task_3.rotate_theta1(client_id, math.pi/2, False)
        
        task_3.qr_code_pid(client_id, wheel_joints, 0)

        ########################################################
        start = target_points[-1]
        # print(start)
        target_points = []
        target_points.append(rooms_entry[room-1])
        # print(target_points)
        to_gate(client_id, qr_code, wheel_joints, start,
                target_points, transformed_image1, array1, array)
        start = target_points[0]
        target_points = []
    vision_sensor_image, image_resolution, return_code = task_3.get_vision_sensor_image(
        client_id)
    transformed_image = task_3.transform_vision_sensor_image(
        vision_sensor_image, image_resolution)
    start = task_3.detect_qr_codes(transformed_image)
    start = (int(start[0]), int(start[1]))
    # print(start)
    # print(b1, b2)
    d_to_b1 = calculate_distance(start, b1, rooms_entry, array1, array)
    d_to_b2 = calculate_distance(start, b2, rooms_entry, array1, array)
    if (1 in box_data) and (2 in box_data):
        if d_to_b1 > d_to_b2:
            target_points.append(b2[0])
            # target_points.append(b1)
        else:
            target_points.append(b1[0])
            # target_points.append(b2)
    elif (1 in box_data):
        target_points.append(b1[0])
    else:
        target_points.append(b2[0])

    to_gate(client_id, qr_code, wheel_joints, start,
            target_points, transformed_image1, array1, array)
    while True:
        if task_3.get_curr_orientation(client_id) == 0:
            break
        task_3.rotate_theta1(client_id, math.pi/2, False)
    task_3.get_closer(client_id, wheel_joints, 1, True)

    box_no = 2
    if target_points[0] == b1[0]:
        box_no = 1

    open_gates_final(client_id, box_no, zip_with_type_and_box)
    time.sleep(10)

    task_3.get_closer(client_id, wheel_joints, -1, True)
    target_points = []
    if (1 in box_data) and (2 in box_data):
        if d_to_b1 > d_to_b2:
            target_points.append(b1[0])
        else:
            target_points.append(b2[0])
        to_gate(client_id, qr_code, wheel_joints, start,
                target_points, transformed_image1, array1, array)
        while True:
            if task_3.get_curr_orientation(client_id) == 0:
                break
            task_3.rotate_theta1(client_id, math.pi/2, False)
        task_3.get_closer(client_id, wheel_joints, 1, True)
        if box_no == 1:
            box_no = 2
        else:
            box_no = 1
        open_gates_final(client_id, box_no, zip_with_type_and_box)
        time.sleep(10)

#######################################################################


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

    sim.simxFinish(-1)
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

    if client_id != -1:
        return_code = sim.simxStartSimulation(
            client_id, sim.simx_opmode_oneshot)

    ##################################################

    return return_code


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

    if return_code != -1:
        return_code = sim.simxStopSimulation(
            client_id, sim.simx_opmode_oneshot_wait)

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

    sim.simxGetPingTime(client_id)
    sim.simxFinish(client_id)

    ##################################################


if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry = [(2, 5), (5, 8), (8, 3), (3, 0)]    # example list of tuples

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
