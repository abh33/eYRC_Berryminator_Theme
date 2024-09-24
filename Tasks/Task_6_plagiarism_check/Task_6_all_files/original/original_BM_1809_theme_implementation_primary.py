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


# Team ID:			[ 1809 ]
# Author List:		[ Treshan, Muditha, Shivanka, Ashen ]
# Filename:			theme_implementation.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from audioop import reverse
from importlib.resources import path
from mimetypes import init
from pydoc import cli
from re import L
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
from pyzbar.pyzbar import decode


# ##############################################################
import task_1b
import task_2a
import task_3


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

# function for decoding berry datas from Theme_Config.json file
def decode_config():
    # read and get datas
    file_open = open('Theme_Config.json')
    file_read = file_open.read()
    file_read = file_read.split('\n')
    file_read = file_read[1:-1]
    file_read = [i.strip(' ,') for i in file_read]
    file_open.close()

    # create a dictionary including each berry type with number of berries and collection box
    berryToPluck = {}
    for item in file_read:
        item_split= item.split('"')
        berry, number, CB = item_split[1], int(item_split[3][0]), int(item_split[3][-1])
        berryToPluck[berry] = [number, CB]
    return berryToPluck

#Function to open and close the gripper
def call_open_close(ci, command, object):

    command = [command]
    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(ci, object ,sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)

#setting the dummy position wrt to the vision sensor handle
def set_dummy_pos(ci, position):
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(ci,'robotic_arm',sim.sim_scripttype_childscript,'set_target', [] , position , [] , '' ,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        return retInts,retFloats,retStrings,retBuffer
        
#setting the dummy position wrt to the world frame
def set_dummy_pos_box(ci , box):
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(ci,'robotic_arm',sim.sim_scripttype_childscript,'set_target_box', [box] , [] , [] , '' ,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        return retInts,retFloats,retStrings,retBuffer

##setting up orientation of the gripper
def set_orientaion(ci, val):
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(ci,'robotic_arm',sim.sim_scripttype_childscript,'set_orientation', [val] , [] , [] , '' ,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        return retInts,retFloats,retStrings,retBuffer

#putting berries into the box
def put_to_box(ci, box_num):
     
    #collection_box_cord = [0, -0.10, 0]
    retInts3,retFloats3,retStrings3,retBuffer3 = set_dummy_pos_box(ci, box_num)
    check_moved(ci)
    #time.sleep(2)
    call_open_close(ci, "open" , "gripper")
    time.sleep(1)
    call_open_close(ci, "close", "gripper")
    time.sleep(1)

# function for sending data to Eyantra servers 
def send_identified_berry_data(ci,berry_name,x_coor,y_coor,depth):
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
	`ci` 	:  [ integer ]
		the ci generated from start connection remote API, it should be stored in a global variable

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

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(ci,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code
	
	##################################################

# function for setting a quarter number for each entry
def setQuartersToEntries(room_entry):
    entries = {}        # dictionary with entry and the quarter
    room = 0
    for entry in room_entry:
        room += 1
        entry_row, entry_col = entry[0], entry[1]
        if entry_row == 3 :
            quarter = 3                         # left side facing
            pre_entries.append((4, entry_col))
        elif entry_row == 5:
            quarter = 1                          # right side facing
            pre_entries.append((4, entry_col))
        elif entry_col == 5:
            quarter = 0                          # front side facing
            pre_entries.append((entry_row, 4))
        else:
            quarter = 2                         # back side facing
            pre_entries.append((entry_row, 4))
        
        entries[room] = [entry, quarter]
    return entries

# fucntion for setting Bot parallel to a side of a QR image
def iden_shape(ci, transformed_image, parallel_state):
    img = transformed_image
    img = np.ascontiguousarray(img, dtype=np.uint8)
    code = decode(img)
    
    # checking for parallis between the Bot and a side of a QR image
    for barcode in code:
        pts = np.array([barcode.polygon], np.float32)
        pts = pts.reshape((-1, 1, 2))
        pts = [i[0] for i in pts]
        pts = sorted(pts, key = lambda x: x[0])
        thresh = 40
        if pts[1][0] - pts[0][0] < thresh and pts[-1][0] - pts[-2][0] < thresh :
            if parallel_state == 0:
                return pts
            else:
                pts = sorted(pts, key = lambda y: y[1])
                angle = np.arctan((pts[1][1] - pts[0][1])/(pts[1][0] - pts[0][0])) * 360 / (2 * np.pi)

                # calculation and suitable rotating 
                angle_thresh = 1
                rot_vel_scale = 0.3
                k,_,_ = PID_center(angle - angle_thresh, error_sum_para, last_error_para)
                if abs(angle) <= angle_thresh:
                    task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
                    return pts
                elif angle > 0:
                    task_3.set_bot_movement(ci, wheel_joints, 0, 0, -(k)*rot_vel_scale )
                else:
                    task_3.set_bot_movement(ci, wheel_joints, 0, 0, -(k)*rot_vel_scale)
    return []
# PID for setQRmid function
def PID_mid(diff,error_sum_mid,last_error_mid):
    Kp = 1
    Ki = 0
    Kd = 0.05
    P = Kp*diff
    I = (error_sum_para+diff)*Ki
    D = (diff-last_error_mid)*Kd

    correction = P+I+D
    error_sum_mid += diff
    last_error_mid = diff
    return correction,error_sum_mid,last_error_mid

#PID for idenshape function
def PID_center(dangle,error_sum_para,last_error_para):
    Kp = 0.1
    Ki = 0
    Kd = 0.05
    P = Kp*dangle
    I = (error_sum_para+dangle)*Ki
    D = (dangle-last_error_para)*Kd

    correction = P+I+D
    error_sum_para += dangle
    last_error_para = dangle
    return correction,error_sum_para,last_error_para

# function for identifing berry types and number of berries at first
def identify_berries(ci, vision_sensor_handle_1, vision_sensor_handle_2, wheel_joints):
    room_berries_list = []

    quarter_count = 0
    pre_enc = task_3.encoders(ci)
    max_s, max_b, max_l = 0, 0, 0       # maximum number of each berry type in a room

    task_3.set_bot_movement(ci, wheel_joints, 0, 0, -0.5)       # robot rotating
    while True:

        #Get image array and depth buffer from vision sensor in CoppeliaSim scene
        try:
            # vision sensor 2 images
            vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(ci,vision_sensor_handle_2)
            vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(ci, vision_sensor_handle_2)

            # vision sensor 1 qr code image
            vision_sensor_image_1, image_resolution_1, return_code_1 = task_2a.get_vision_sensor_image(ci,vision_sensor_handle_1)

            if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):

                # Get the transformed vision sensor image captured in correct format
                try:
                    # vision sensor 2 transformed image
                    transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
                    transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)

                    # vision sensor 2 qr codes transformed image
                    transformed_image_1 = task_1b.transform_vision_sensor_image(vision_sensor_image_1, image_resolution_1)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                            break

                    if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):
                        # berry positions
                        berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
                        berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
                        
                        # count berries in each room
                        if len(berry_positions_dictionary["Strawberry"]) == len(berry_positions_dictionary["Blueberry"]) == len(berry_positions_dictionary["Lemon"]) == 0:
                            if max_b !=0 and max_l != 0 and max_s != 0 :
                                room_berries_list.append([max_b, max_l, max_s])
                                max_b, max_l, max_s = 0, 0, 0                          


                        elif len(berry_positions_dictionary["Strawberry"]) >= max_s and len(berry_positions_dictionary["Blueberry"]) >= max_b and len(berry_positions_dictionary["Lemon"]) >= max_l:
                            max_s = len(berry_positions_dictionary["Strawberry"])
                            max_b = len(berry_positions_dictionary["Blueberry"])
                            max_l = len(berry_positions_dictionary["Lemon"])
                        
                        # count quarters that has rotated
                        enc = if_quarter(pre_enc, ci)
                        if enc != []:
                            quarter_count += 1
                            pre_enc = enc
                        
                        # stop rotating if the Bot has rotated a full round and been parallel
                        if quarter_count == 4:                            
                            points = iden_shape(ci, transformed_image_1, 1)
                            if points != []:
                                break

                        else:
                            points = iden_shape(ci, transformed_image_1, 0)
                    
                    else:
                        print('\n[ERROR] transform_vision_sensor_image function is not configured correctly, check the code.')
                        print('Stop the CoppeliaSim simulation manually.')
                        print()
                        sys.exit()

                except Exception:
                    print('\n[ERROR] Your transform_vision_sensor_image function throwed an Exception, kindly debug your code!')
                    print('Stop the CoppeliaSim simulation manually.\n')
                    traceback.print_exc(file=sys.stdout)
                    print()
                    sys.exit()

            else:
                print('\n[ERROR] get_vision_sensor function is not configured correctly, check the code.')
                print('Stop the CoppeliaSim simulation manually.')
                print()
                sys.exit()

        except Exception:
            print('\n[ERROR] Your get_vision_sensor_image function throwed an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    # craete a dictionary including berry numbers in each room
    room_berries = {}
    for index in range(len(room_berries_list)):
        if index == 3:
            room_berries[1] = room_berries_list[index]
        else:
            room_berries[index+2] = room_berries_list[index]

    return room_berries

# function for getting every possible navigation
def room_order(least_rooms_list, berries = [], room = 0):
    if room != 0:
        least_rooms_list = least_rooms_list + [room]
        berries = berries + [room_berries[room]]
        berry_sum = [np.array(i) for i in berries]
        B_sum, L_sum, S_sum = sum(berry_sum)
    else:
        B_sum, L_sum, S_sum = 0, 0, 0
    if B_sum >= berryToPluck['B'][0] and L_sum >= berryToPluck['L'][0] and S_sum >= berryToPluck['S'][0]:
        roomOrder.append(sorted(least_rooms_list))
        return
    
    else:
        for room in range(1, 5):
            if room not in least_rooms_list:
                room_order(least_rooms_list, berries, room)

# function for getting every possible navigation with least number of rooms
def possible_room_orders(least_room_list):
    sorted_len_room_orders = sorted(least_room_list, key = len)
    min_len = len(sorted_len_room_orders[0])        # minimum number of rooms

    possible_orders = []
    for order in sorted_len_room_orders:
        if len(order) == min_len:
            possible_orders.append(order)
        else:
            break
    return possible_orders

# function for calculating distances and turns in paths
def distanceAndTurns(points_order):
    turns = 0
    distance = 0
    pre_pos = iniPos
    for item in points_order:
        if item[0] == 't':      # turns
            turns += int(item[1])
        else:           # distances
            points = sum(item, [])
            for index in range(len(points)):
                distance += (abs(points[index][0] - pre_pos[0]) + abs(points[index][1] - pre_pos[1]))
                pre_pos = points[index]

    return distance, turns

# functions for getting all permutations of paths (room lists)
def path_permutations(path, paths = []):
    if len(path) == 1:  # one room  
        return [path]
    elif len(path) == 2:        # two rooms
        paths = [path, list(reversed(path))]
        return paths
    elif len(path) == 3:        # three rooms
        for i in range(3):
            path1 = [path[i%3], path[(i+1)%3], path[(i+2)%3]]
            paths.extend([path1, list(reversed(path1))])
        return paths
    elif len(path) == 4:            # four rooms
        for i in range(4):
            path1 = [path[(i+1)%4], path[(i+2)%4], path[(i+3)%4]]
            path2 = path_permutations(path1, [])
            path3 = [[path[i]] + path1 for path1 in path2]
            paths.extend(path3)
        return paths

# fucntion for finding a best path with minimum distances and turns
def best_path(rooms_entry, possible_orders_list):
    path_list = []
    path_dis_turns = []

    for non_order_path in possible_orders_list:
        paths = path_permutations(non_order_path)
        for path in paths:
            points_order = pointsOrder(path, rooms_entry)
            distance, turns = distanceAndTurns(points_order)

            for room in range(1, 5):
                if room in path and rooms_entry[room][1] != rack_quarter[room]:     # if racks are not in front of entries
                    turns += 2      # two turns inside rooms
            path_dis_turns.append([distance, turns])
        path_list.extend(paths)

    path_dis_turns_sorted = path_dis_turns.copy()
    path_dis_turns_sorted = sorted(path_dis_turns_sorted, key = lambda x : (x[0], x[1]))
    min_dis_turns = path_dis_turns_sorted[0]
    best_room_order = path_list[path_dis_turns.index(min_dis_turns)]        # best path
    return best_room_order

# function for identifing a rotation about a quarter using encoder values
def if_quarter(pre_enc, ci):
    enc = task_3.encoders(ci)
    cur_rot = [enc[j] - pre_enc[j] for j in range (len(enc))]

    for e in cur_rot:
        if e >= 10.5 :
            return enc            
    return []

# function for Bot rotation
def rotate(ci, quarter, direction):
    # rotation
    rot_velocity = 1
    task_3.set_bot_movement(ci, wheel_joints, 0, 0, -rot_velocity * direction)

    qua = 0
    pre_enc = task_3.encoders(ci)
    while True:
        vision_sensor_image_1, image_resolution_1, return_code_1 = task_2a.get_vision_sensor_image(ci, vision_sensor_handle_1)
        if ((return_code_1 == sim.simx_return_ok) and (len(image_resolution_1) == 2)  and (len(vision_sensor_image_1) > 0)):
            transformed_image_1 = task_1b.transform_vision_sensor_image(vision_sensor_image_1, image_resolution_1)

            # complete quarters and stop
            enc = if_quarter(pre_enc, ci)
            if enc != [] :
                pre_enc = enc
                qua += 1
            if qua == quarter:
                points = iden_shape(ci, transformed_image_1, 1)
                if len(points):
                    break

# function for changing cordinates according to quarter values (changing the cordinate system according to the facing side of the Bot)
def change_rel_cord(points, quarter, cur_pos):
    quarter %= 4
    if quarter == 3:
        cur_pos = (cur_pos[1], 8 - cur_pos[0])
        points = [(point[1], 8 - point[0]) for point in points]
    elif quarter == 1:
        cur_pos = (11 - cur_pos[1], cur_pos[0])
        points = [(11 - point[1], point[0]) for point in points]
    elif quarter == 2:
        cur_pos = (8 - cur_pos[0], 11 - cur_pos[1])
        points = [(8 - point[0], 11 - point[1]) for point in points]
    return points

# function for getting cordinate of points that the Bot has to traversed on best path
def pointsOrder(roomOrder, room_entry):
    point_order = []
    pre_quarter = 0
    pre_pos = iniPos
    pre_room = 0
    for room in roomOrder:
        entry = room_entry[room][0]
        entry_row, entry_col = entry[0], entry[1]
        cur_quarter = room_entry[room][1]       # current quarter
 
        quarter_diff = abs(cur_quarter - pre_quarter)       # number of quarters that has to rotate next
        if quarter_diff == 1:
            if cur_quarter - pre_quarter > 0:
                point_order.append('t1'+'d+1')
            else:
                point_order.append('t1'+'d-1')
        elif quarter_diff == 2:
            point_order.append('t'+str(quarter_diff)+'d+1')     # clockwise rotation
        elif quarter_diff == 3:
            point_order.append('t1'+'d-1')          # anticlockwise rotation

        if pre_room == 3 or pre_room == 4:
            if room == 1 or room == 2:
                point_order.append([[iniPos]])
        pre_room = room

        # getting points
        if entry_row == 3 :
            point_order.append([[(4, entry_col),entry], [(4, entry_col)]])
        elif entry_row == 5:
            point_order.append([[(4, entry_col),entry], [(4, entry_col)]])
        elif entry_col == 5:
            point_order.append([[(entry_row, 4),entry], [(entry_row, 4)]])
        else:
            if entry_col == 3:
                point_order.append([[(entry_row, 4),entry], [(entry_row, 4)]])
            elif entry_col == 9:
                point_order.append([[(4,10), (entry_row, 10),entry], [(entry_row, 10), (4,10)]])
        
        cur_pos = point_order[-1][-1][0]
        # if pre_pos == cur_pos:
        #     del point_order[-1][0][0]
        if pre_pos[0] != cur_pos[0] and pre_pos[1] != cur_pos[1]:
            point_order.insert(-2, [[iniPos]])
        pre_pos = cur_pos
        pre_quarter = cur_quarter
    if cur_pos[1] != 9:
        if cur_pos[1] < 5:
            point_order.append([[iniPos]])     
        point_order.append([[(4, 10)]])
    return point_order

# function for setting the Bot parallel to a side of a QR code
def setQRParallel():
    while True:
        #print("setqrparallel")
        count=0
        while count<50:
            #print(count)
            vision_sensor_image_1, image_resolution_1, return_code_1 = task_2a.get_vision_sensor_image(ci, vision_sensor_handle_1)
            
            if ((return_code_1 == sim.simx_return_ok) and (len(image_resolution_1) == 2)  and (len(vision_sensor_image_1) > 0)):
                
                transformed_image_1 = task_1b.transform_vision_sensor_image(vision_sensor_image_1, image_resolution_1)
                if iden_shape(ci, transformed_image_1, 1):
                    
                    break
            count += 1
        else:       # when not detecting QR codes
            task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0.02)
        
        if count<50:
            break

# function for moving the robot inside rooms 
def robot_move_in_room(ci, point_order , cur_pos , cur_quarter, state):
    #cur_quarter = 0
    dir = 1
    for item in point_order[0]:
            if item[0] == 't':      # rotations
                quarter, dir = int(item[1]), int(item[-2:])
                cur_quarter += dir * quarter
                rotate(ci, quarter, dir)
            else:
                # navigation
                for point in item:                    
                    rel_points = change_rel_cord([point], cur_quarter, cur_pos)
                    rel_pos = change_rel_cord([cur_pos], cur_quarter, cur_pos)
                    targets = [rel_pos, rel_points, [point]]
                    
                    task_3.task_3_primary(ci, targets)
                    if point == (2, 2) or point == (2, 6):
                        if cur_quarter == 0:
                            if state == 'enter':
                                setQRmid(0, 1)
                            else:
                                setQRmid(1, 0, 120)

                        elif cur_quarter == 2:
                            if state == 'enter':
                                setQRmid(0, 1)
                            else:
                                setQRmid(1, 0, 392)
                    
                    elif point in entries:
                        pass

                    elif point == point_order[0][0][-1]:
                        if point == (2, 5):
                            if cur_quarter == 0:
                                setQRmid(1, 0, 120)
                            elif cur_quarter == 2:
                                setQRmid(1, 0, 392)
                        else:
                            if len(point_order[0][0]) == 2:
                                setQRmid(1, 1)
                            else:
                                setQRmid(1, 0)

                    else:
                        print(point)
                        setQRmid(1, 1)
                    cur_pos = point
                    setQRParallel()
    return cur_pos, cur_quarter

# function for setting the path inside the room
def path_in_room(entry_cord , room_num):

    if room_num == 1:
        if entry_cord == (4,6) or entry_cord ==(4,7):
            return 'rc'
    if room_num == 2:
        if entry_cord == (6,4) or entry_cord ==(7,4):
            return 'rc'
        if entry_cord == (6,10) or entry_cord ==(7,10):
            return 'ra'
    if room_num == 3:
        if entry_cord == (4,2) or entry_cord == (4,1):
            return 'rc'
    if room_num == 4:
        if entry_cord == (2,4) or entry_cord == (1,4):
            return 'rc'

# function for Bot moving outside rooms
def robot_move(ci, point_order , rooms_entry):
    cur_quarter = 0
    cur_pos = iniPos
    dir = 1
    for item in point_order:
            if item[0] == 't':          # rotations
                quarter, dir = int(item[1]), int(item[-2:])
                cur_quarter += dir * quarter
                rotate(ci, quarter, dir)
                setQRParallel()
                setQRmid(1, 1)
            else:
                # navigation through points
                for point_list in item:
                    for point in point_list:
                        if point not in entries:
                        # get relative cordinates
                            rel_points = change_rel_cord([point], cur_quarter, cur_pos)
                            rel_pos = change_rel_cord([cur_pos], cur_quarter, cur_pos)
                            targets = [rel_pos, rel_points, [point]]                            
                            # moving and using setQRmid and setQRParallel functions
                            task_3.task_3_primary(ci, targets)
                        else:
                            continue    
                        if point == (2, 4):
                            for room_num in range(1, 5):
                                if pre_entries[room_num-1] == point and cur_quarter == rooms_entry[room_num][1]:
                                    room = room_num
                                    break
                            # print("enter :", room)
                            if not room_visited[room]:
                               
                                if cur_quarter == 0:
                                    setQRmid(1, 0, 120)
                                elif cur_quarter == 2:
                                    setQRmid(1, 0, 392)
                                # room_visited[room] = True
                            else:   
                                if cur_quarter == 0:
                                    setQRmid(0, 1)
                                elif cur_quarter == 2:
                                    setQRmid(0, 1)
                        elif point == (4, 10):
                            setQRmid(1, 0)
                        if point in pre_entries:
                            if point!=(2, 4):
                                for room_num in range(1, 5):
                                    
                                    qua = cur_quarter%4
                                    # if cur_quarter==-1:
                                    #     qua = 3
                                    if pre_entries[room_num-1] == point and qua == rooms_entry[room_num][1]:
                                        room = room_num
                                        break                
                            if not room_visited[room]:  # enter a room  
                                
                                if point!=(2, 4):                              
                                    setQRmid(1, 0)
                                room_visited[room] = True
                        # entry
                        #elif point in entries:
                                # room_num = pre_entries.index(point) + 1                            
                                room_num = room
                                room_points = get_room_points(room_num, pre_entries)
                                cur_pos, cur_quarter = robot_move_in_room(ci, room_points, point, cur_quarter, 'enter')

                                #setQRmid(1, 1)
                                #path in room
                                rot = path_in_room(point, room_num)
                                if rot == 'rc':
                                    rotate(ci, 1, 1)   ##rotate 90degrees right
                                elif rot == 'ra':
                                    rotate(ci, 1, -1)   ## rotate 90 degrees left
                                
                                setQRmid(1, 1)
                                task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
                                pluck_berries_in_room(ci, berry_data)       # plucking berries
                                
                                if rot == 'rc':
                                    rotate(ci, 1, -1)   ##rotate 90degrees LEFT
                                elif rot == 'ra':
                                    rotate(ci, 1, 1)   ## rotate 90 degrees RIGHT
                                
                                setQRmid(1, 1)
                                
                                cur_pos, cur_quarter = robot_move_in_room(ci, [[[room_points[0][0][0]]]] , room_points[0][0][1], cur_quarter, "leave")        
                            else:             # leave a room  
                                if point != (2, 4):           
                                    setQRmid(0, 1)
                                cur_pos = point
                        
                        elif point not in pre_entries:
                            cur_pos = point                          
                            setQRmid(1, 1)
                        elif point!=(2, 4):         
                            if point in pre_entries:                                            
                                setQRmid(1, 1)
                    setQRParallel()
    return cur_quarter

# function for aligning the Bot according to the QR code view in vision sensor image
def setQRmid(x_state, y_state, x_thresh = 256, y_thresh = 256):
    qr_R = 20
    vel_scale = 0.005
    while True:
        count = 0
        # setting the Bot
        while count< 100:
            vision_sensor_image_1, image_resolution_1, return_code_1 = task_2a.get_vision_sensor_image(ci, vision_sensor_handle_1)
            if ((return_code_1 == sim.simx_return_ok) and (len(image_resolution_1) == 2)  and (len(vision_sensor_image_1) > 0)):
                transformed_image_1 = task_1b.transform_vision_sensor_image(vision_sensor_image_1, image_resolution_1)
                if (type(transformed_image_1) is np.ndarray):
                    qr_codes_list = task_3.detect_qr_codes(transformed_image_1)
                    if len(qr_codes_list) != 0:
                        cur_qr_x, cur_qr_y = qr_codes_list[0][1][0], qr_codes_list[0][1][1]

                        # calculation part
                        x_diff, y_diff = cur_qr_x - x_thresh, cur_qr_y - y_thresh
                        kx,_,_ = PID_mid(x_diff,error_sum_mid_x,last_error_mid_x)
                        ky,_,_ = PID_mid(y_diff,error_sum_mid_y,last_error_mid_y)

                        if x_state == 1:
                            if abs(x_diff) <= qr_R:
                                task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
                                x_state = 0
                            else:
                                task_3.set_bot_movement(ci, wheel_joints, 0, -kx*vel_scale, 0)

                        elif y_state == 1:
                            if abs(y_diff) <= qr_R:
                                task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
                                y_state = 0
                            else:
                                task_3.set_bot_movement(ci, wheel_joints, ky*vel_scale, 0, 0)
                        
                        if x_state == y_state == 0:
                            
                            break
            else:           
                count += 1

        else:       # when not detecting the QR code
            task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0.01)
        
        if count< 100:
            break

# function for path planning for rooms
def get_room_points(room_num, rooms_entry):
    room_qr_codes = {1:[[[(1,7)]]], 2:[[[(7,7)]]], 3:[[[(7,1)]]], 4:[[[(1,1)]]] }
    c = 0
    for i in range(4):

        if rooms_entry[i][1] == 4 and (room_num == 1 or room_num == 2):
            room_qr_codes[c+1][0][0].insert(0, (rooms_entry[i][0], rooms_entry[i][1]+2))

        if  (rooms_entry[i][1] == 10) and (room_num == 2):
            room_qr_codes[c+1][0][0].insert(0, (rooms_entry[i][0], rooms_entry[i][1]-2))

        if  rooms_entry[i][0] == 4 and (room_num == 1 or room_num == 4):
            room_qr_codes[c+1][0][0].insert(0, (rooms_entry[i][0] - 2, rooms_entry[i][1]))

        if  rooms_entry[i][0] == 4 and (room_num == 2 or room_num == 3):
            room_qr_codes[c+1][0][0].insert(0, (rooms_entry[i][0] + 2, rooms_entry[i][1]))

        if  rooms_entry[i][1] == 4 and (room_num == 4 or room_num == 3):
            room_qr_codes[c+1][0][0].insert(0, (rooms_entry[i][0], rooms_entry[i][1]-2))
        c += 1
    
    return room_qr_codes[room_num]


# function for getting the berry details
def get_berry_position(ci):
    berries_dictionary = {}
    berry_positions_dictionary = {}
    return_code, vision_sensor_handle = sim.simxGetObjectHandle(ci, 'vision_sensor_2', sim.simx_opmode_blocking)
       
    vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(ci,vision_sensor_handle)
    vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(ci, vision_sensor_handle)

    if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
        transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
        transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
        

        

        if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

            berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
            berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)

            labelled_image = task_2a.get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
            labelled_depth_image = task_2a.get_labeled_image(transformed_depth_image, berries_dictionary, berry_positions_dictionary)
                        
    return berry_positions_dictionary

#Taking velocities of handles to check whether they have moved
def get_velocities(ci):
    
    handles = []
    joint = "robotic_arm_rj_r1"
    rec,handle =  sim.simxGetObjectHandle(ci, joint , sim.simx_opmode_blocking)
    handles.append(handle)
    for i in range(5):
        joint =  'robotic_arm_rj_'+str(i+1)+str(i+2)
        rec,handle =  sim.simxGetObjectHandle(ci, joint , sim.simx_opmode_blocking)
        
        handles.append(handle)
    
    vel_list = []
    for i in range(len(handles)):
        res, velocity =sim.simxGetObjectFloatParameter(ci, handles[i] , 2012, sim.simx_opmode_oneshot)
        vel_list.append(abs(round(velocity,2)))    

    return vel_list

#function to check whether the arm has moved to the correct position  
def check_moved(ci):
    c = 0
    while True:
        vel_list = get_velocities(ci)
        if c == 0:
            pass
        else:  
            if sum(vel_list) < 0.3:
                break
        c += 1


#function called for plucking berries
def pluck_berry(ci, position, box_num):

    retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.2, 0.15])
    check_moved(ci)
    

    if (position[0] < -0.21 and position[0] > -0.265) or (position[0] > 0.21 and position[0] < 0.265):
        
        retInts1,retFloats1,retStrings1,retBuffer1 = set_orientaion(ci, 1)
        
        check_moved(ci)
        #time.sleep(2)

        if position[0] < 0:
            retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0] - 0.05  , position[1]-0.1   , position[2]])
        else:
            retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0] + 0.05  , position[1]-0.1   , position[2]])

        #time.sleep(2)
        call_open_close(ci, "open", "gripper")
        check_moved(ci)
        #retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1]+0.01 , position[2]+0.01 ])
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , position[2] ])
        check_moved(ci)
        #time.sleep(3)     
        call_open_close(ci, "close", "gripper")    
        time.sleep(1)

        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , 0.15])
        check_moved(ci)
        #time.sleep(2)
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.09, 0.15])
        check_moved(ci)
        #time.sleep(2)
        retInts1,retFloats1,retStrings1,retBuffer1 = set_orientaion(ci, 0)
        check_moved(ci)
        put_to_box(ci, box_num)

    elif position[0] < -0.21:
        
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1]   ,position[2]])
        call_open_close(ci, "open", "gripper")
        check_moved(ci)
        #retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , position[2] ])
        #check_moved(ci)
        #time.sleep(3)     #3
        call_open_close(ci, "close", "gripper")    
        time.sleep(1)
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , 0.15])
        check_moved(ci)
        #time.sleep(2) #2

        if box_num == 2:
            retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [ -0.2  , -0.1 , 0])
            check_moved(ci)
            put_to_box(ci, box_num)
        else:

            retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.09, 0.15])
            check_moved(ci)
            put_to_box(ci, box_num)
        #time.sleep(1.5)

    elif position[0] > 0.265:
        
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1]   ,position[2]])
        call_open_close(ci, "open", "gripper")
        check_moved(ci)
        #retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , position[2] ])
        #check_moved(ci)
        #time.sleep(3)     #3
        call_open_close(ci, "close", "gripper")    
        time.sleep(1)
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , 0.15])
        check_moved(ci)
        #time.sleep(2) #2

        if box_num == 1:
            put_to_box(ci, box_num)
        else:

            retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.09, 0.15])
            check_moved(ci)
            put_to_box(ci, box_num)
            
        #time.sleep(1.5)
    else:
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1]   ,position[2]])
        call_open_close(ci, "open", "gripper")
        check_moved(ci)
        #retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , position[2] ])
        #check_moved(ci)
        #time.sleep(3)     #3
        call_open_close(ci, "close", "gripper")    
        time.sleep(1)
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [position[0]  , position[1] , 0.15])
        check_moved(ci)
        #time.sleep(2) #
        retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.09, 0.15])
        check_moved(ci)

        put_to_box(ci, box_num)


def pluck_berries_in_room(ci, berry_data):
    berry_pos = get_berry_position(ci)
    berries = []

    for berry in berry_pos:
        if berry_pos[berry][0][0] < 0.1362 and berry_pos[berry][0][0] > -0.1727:
           berries.insert(0, berry)
        else:
            berries.append(berry)

    num_berries = [len(berry_pos[i]) for i in berries]
    
    for i in range(3):
        berry_pos_sorted = sorted(berry_pos[berries[i]] , key = lambda x:x[2])

        if not berry_data[berries[i][0]][0] == 0:
            if num_berries[i] >= berry_data[berries[i][0]][0]:
                for j in range(berry_data[berries[i][0]][0]):
                    
                    
                    print(berries[i] , berry_pos_sorted[j][0], berry_pos_sorted[j][1], berry_pos_sorted[j][2])
                    send_identified_berry_data(ci, berries[i] , berry_pos_sorted[j][0], berry_pos_sorted[j][1], berry_pos_sorted[j][2])
                    pluck_berry(ci, berry_pos_sorted[j], berry_data[berries[i][0]][1])
                    
                    berry_data[berries[i][0]][0] -= 1

                    #rotate when put to box
                   
                    #put_to_box(ci,  berry_data[berries[i][0]][1])
                    retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.05, 0.15])
                    check_moved(ci)

            else:
                for j in range(num_berries[i]):
                    print(berries[i] , berry_pos_sorted[j][0], berry_pos_sorted[j][1], berry_pos_sorted[j][2])
                    send_identified_berry_data(ci, berries[i] , berry_pos_sorted[j][0], berry_pos_sorted[j][1], berry_pos_sorted[j][2])
                    #time.sleep(0.5)

                   
                    pluck_berry(ci, berry_pos_sorted[j] , berry_data[berries[i][0]][1])
                    
                    berry_data[berries[i][0]][0] -= 1
                    
                    #put_to_box(ci,  berry_data[berries[i][0]][1])
                    retInts1,retFloats1,retStrings1,retBuffer1 = set_dummy_pos(ci, [0, -0.05, 0.15])
                    check_moved(ci)
               
#function to align robot in a room
def align_robot(ci):
    wheel_joints = task_3.init_setup(ci)
    while True:
        berry_pos = get_berry_position(ci)
        mins = []
        for i in berry_pos:
            if len(berry_pos[i]) != 0:
                mins.append(sorted(berry_pos[i] , key = lambda x:x[2])[0][2])
        if min(mins) < 0.5:
            task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
            
            break
        else:
            task_3.set_bot_movement(ci, wheel_joints, -1, 0, 0)
    #else:

#############################################################
#depositing in collection boxes
def call_open_close_box(ci, command,lr):
    
    if lr == 1:
        y = 2
    else:
        y = 1

    command = [command]
    emptybuff = bytearray()
    return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(ci, 'basket' ,sim.sim_scripttype_childscript,'open_close',[y],[],command,emptybuff,sim.simx_opmode_blocking)

def closerToBox(k, box_num):
    pos1 = task_3.encoders(ci)
    #print(pos1)
    while True:
        task_3.set_bot_movement(ci, wheel_joints, 0, k*4, 0)
        x = task_3.encoders(ci)
        #print(x)
        enc1 = x[0]
        if abs(enc1 - pos1[0]) > 3.5:
            break

    task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)
    call_open_close_box(ci, 'open', -k)
    time.sleep(5)
    call_open_close_box(ci, 'close',-k)
    if box_num != 2:        
        pos1 = task_3.encoders(ci)
        while True:
            task_3.set_bot_movement(ci, wheel_joints, 0, k*(-4), 0)
            x = task_3.encoders(ci)
            #print(x)
            enc1 = x[0]
            if abs(enc1 - pos1[0]) > 3.5:
                break

def berryPlace(quar,lr,procee):            #left_first==> lr=-1 , right_first==> lr=1
    
    dest = ['(7, 10)','(1, 10)']
    if lr == -1 :
        dest = dest[::-1]
    if quar == 0:
        rotate(ci,1,lr*1)
        quar += 1
    elif quar%4 == 1:
        if lr == -1:
            rotate(ci,2,lr*1)
            quar += 2
    elif quar%4 == 2:
        rotate(ci, 1, lr * -1)
        quar -= 1
    elif quar%4 == 3:
        if lr == 1 and procee == 0:
            rotate(ci,2,lr*1)
            quar += 2
        else:
            dest = dest[::-1]
            lr=-lr
    quar %= 4
    setQRmid(1, 0)
    # while True:
    point = eval(dest[0])
    rel_points = change_rel_cord([point], quar, point)
    rel_pos = change_rel_cord([(4, 10)], quar, (4, 10))
    targets = [rel_pos, rel_points, [point]]
    
    
    # moving and using setQRmid and setQRAPrallel functions
    task_3.task_3_primary(ci, targets)
    setQRParallel()
    setQRmid(1,1)
    #collection_box_weight(-1*lr,0)
    closerToBox(-1*lr, 1)
    setQRmid(1,1)
    if procee == 1:
        rotate(ci,2,1)
        quar += 2
        setQRmid(1, 1)
        rel_points = change_rel_cord([(4, 10)], quar, (4, 10))
        rel_pos = change_rel_cord([point], quar, point)
        targets = [rel_pos, rel_points, [(4, 10)]]
       
        task_3.task_3_primary(ci, targets)

        setQRmid(1, 0)

        point = eval(dest[1])
        rel_points = change_rel_cord([point], quar, point)
        rel_pos = change_rel_cord([(4, 10)], quar, (4, 10))
        targets = [rel_pos, rel_points, [point]]
        task_3.task_3_primary(ci, targets)
        setQRmid(1, 1)
        closerToBox(1*lr, 2)
        task_3.set_bot_movement(ci, wheel_joints, 0, 0, 0)


def collection_box_plan(curr_quar,box_berries):
    if (len(box_berries[1]) != 0) and (len(box_berries[2]) != 0) :
            berryPlace(curr_quar,1,1)
    elif (len(box_berries[1])==0):
            berryPlace(curr_quar, 1,0)
    elif (len(box_berries[2])==0):
            berryPlace(curr_quar, -1,0)


##############################################################



def theme_implementation_primary(client_id, rooms_entry):
    """
    Purpose:
    ---
    This is the only function that is called from the main function. Make sure to fill it
    properly, such that the bot completes the Theme Implementation.

    Input Arguments:
    ---
    `ci`         :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()

    `rooms_entry`         :   [ list of tuples ]
        Room entry co-ordinate of each room in order.

    
    Returns:
    ---
    
    Example call:
    ---
    theme_implementation_primary(ci, rooms_entry)
    
    """
    #################################################################
    #global variables
    global wheel_joints
    global vision_sensor_handle_1
    global vision_sensor_handle_2
    global enc
    global entries
    global rack_quarter
    global room_berries
    global berryToPluck
    global roomOrder
    global iniPos
    global berry_data
    global ci
    global pre_entries
    global room_visited
    global error_sum_para 
    global last_error_para
    global error_sum_mid_x,error_sum_mid_y
    global last_error_mid_x,last_error_mid_y

    #initialization of varibales related PID for setQRmid and idenshape functions
    last_error_mid,last_error_mid_x,last_error_mid_y = 0,0,0
    error_sum_mid,error_sum_mid_x,error_sum_mid_y =0,0,0
    ci = client_id
    error_sum_para = 0
    last_error_para = 0

    wheel_joints = task_3.init_setup(ci)
    iniPos = (4,4)
    return_code, vision_sensor_handle_1 = sim.simxGetObjectHandle(ci, 'vision_sensor_1', sim.simx_opmode_blocking)
    return_code, vision_sensor_handle_2 = sim.simxGetObjectHandle(ci, 'vision_sensor_2', sim.simx_opmode_blocking)

    rack_quarter = {1 : 0, 2 : 1, 3 : 2, 4 : 3}
    pre_entries = []
    room_visited = {1:False, 2:False, 3:False, 4:False}
    entries = rooms_entry

    #decoding the json config file
    berryToPluck = decode_config()
    berry_data = berryToPluck  

    #deciding the rooms to enter 
    rooms_entry = setQuartersToEntries(rooms_entry)

    #identify berries in each plant and going to room with most berries
    #This is been commented for this task because each plant consists of two berries
    # #room_berries = identify_berries(ci, vision_sensor_handle_1, vision_sensor_handle_2, wheel_joints)
    room_berries = {2: [2, 2, 2], 3: [2, 2, 2], 4: [2, 2, 2], 1: [2, 2, 2]} 

    #order of going to rooms
    roomOrder = []
    room_order([])
    least_room_list = list(map(list,set(map(tuple,roomOrder))))
    possible_orders_list = possible_room_orders(least_room_list)
    roomOrder = best_path(rooms_entry, possible_orders_list)
    point_order = pointsOrder(roomOrder, rooms_entry) 

    #moving the robot to each room and plucking berries  
    curr_quar = robot_move(ci, point_order, rooms_entry)

    box_berries = {1: [], 2: []}
    for i in berry_data:
        box_berries[berry_data[i][1]].append(berry_data[i])

    #putting plucked berries to collection boxes
    collection_box_plan(curr_quar,box_berries)
    #################################################################



if __name__ == "__main__":

    # Room entry co-ordinate
    rooms_entry =  [(3, 6), (6, 5), (6, 3), (3, 2)] # example list of tuples

    ###############################################################
    ## You are NOT allowed to make any changes in the code below ##

    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.')
    print('Trying to connect to Remote API Server...')

    try:
        ci = task_1b.init_remote_api_server()
        if (ci != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!')

            # Starting the Simulation
            try:
                return_code = task_1b.start_simulation(ci)

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
        theme_implementation_primary(ci, rooms_entry)

        try:
            return_code = task_1b.stop_simulation(ci)                            
            if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
                print('\nSimulation stopped correctly.')

                # Stop the Remote API connection with CoppeliaSim server
                try:
                    task_1b.exit_remote_api_server(ci)
                    if (task_1b.start_simulation(ci) == sim.simx_return_initialize_error_flag):
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
        print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    except KeyboardInterrupt:
        print('\n[ERROR] Script interrupted by user!')