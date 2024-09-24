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


# Team ID:			BM-1067
# Author List:		Zaid
# Filename:			theme_implementation.py
# Functions:		
# Global variables:	NONE
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
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()




################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
import task_1b
import task_2a
import task_3

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
	global client_id
	client_id = -1

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(-1) # just in case, close all opened connections
	client_id = sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim


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

	return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
	sim.simxGetPingTime(client_id)
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
	# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	sim.simxGetPingTime(client_id)

    # Now close the connection to CoppeliaSim:
	sim.simxFinish(client_id)

	##################################################

def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):
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

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code
	
	##################################################

def call_open_close(client_id, command):
	"""
	Purpose: Open/Close Gripper
	---
	"""
	time.sleep(0.3)
	return_code,_,_,_,_ = sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],[command],bytearray(),sim.simx_opmode_blocking)
	time.sleep(0.3)

def moveArm(client_id,pos,acuurate=True):
	"""
	Purpose: Move Arm to specified Position
	---
	"""
	# if it is needed to check if arm has reached
	if acuurate:
		# do it twice for better accuracy
		a=0.02
		for _ in range(1):
			_,_,_,_,_ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setTargetPos',[],pos,[''],bytearray(),sim.simx_opmode_blocking)
			_,joint1 = sim.simxGetObjectHandle(client_id,'robotic_arm_rj_r1',sim.simx_opmode_blocking)
			# _,joint2 = sim.simxGetObjectHandle(client_id,'robotic_arm_rj_12',sim.simx_opmode_blocking)
			# _,joint3 = sim.simxGetObjectHandle(client_id,'robotic_arm_rj_23',sim.simx_opmode_blocking)
			# _,joint4 = sim.simxGetObjectHandle(client_id,'robotic_arm_rj_34',sim.simx_opmode_blocking)
			joint1PosOld = -1
			# joint2PosOld = -1
			# joint3PosOld = -1
			# joint4PosOld = -1
			# loop until arm has stopped and then break
			while True:
				for _ in range(2):
					_,joint1Pos = sim.simxGetJointPosition(client_id,joint1,sim.simx_opmode_streaming)
					_,joint1Pos = sim.simxGetJointPosition(client_id,joint1,sim.simx_opmode_buffer)
				# for _ in range(2):
				# 	_,joint2Pos = sim.simxGetJointPosition(client_id,joint2,sim.simx_opmode_streaming)
				# 	_,joint2Pos = sim.simxGetJointPosition(client_id,joint2,sim.simx_opmode_buffer)
				# for _ in range(2):
				# 	_,joint3Pos = sim.simxGetJointPosition(client_id,joint3,sim.simx_opmode_streaming)
				# 	_,joint3Pos = sim.simxGetJointPosition(client_id,joint3,sim.simx_opmode_buffer)
				# for _ in range(2):
				# 	_,joint4Pos = sim.simxGetJointPosition(client_id,joint4,sim.simx_opmode_streaming)
				# 	_,joint4Pos = sim.simxGetJointPosition(client_id,joint4,sim.simx_opmode_buffer)
				error = joint1Pos-joint1PosOld
				#print(error)
				# joint1Pos=round(joint1Pos,3)
				# joint2Pos=round(joint1Pos,3)
				# joint3Pos=round(joint1Pos,3)
				# joint4Pos=round(joint1Pos,3)
				if not (abs(error)<0.01):
					_,_,_,_,_ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setTargetPos',[],pos,[''],bytearray(),sim.simx_opmode_blocking)
					time.sleep(0.01)
				else:
					#print('breaking at',abs(error))
					break
				joint1PosOld = joint1Pos
				# joint2PosOld = joint2Pos
				# joint3PosOld = joint3Pos
				# joint4PosOld = joint4Pos
	else:
		# set target position
		_,_,_,_,_ = sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setTargetPos',[],pos,[''],bytearray(),sim.simx_opmode_blocking)

def get_berries_position(client_id,camPos,plant_direction):
	"""
	Purpose: Detect the position of berries
	---
	"""
	#get vision sensor data
	_,vision_sensor_handle = sim.simxGetObjectHandle(client_id,"vision_sensor_2",sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id,vision_sensor_handle)
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id,vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	#detect berries from images
	berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	#set X and Y addition variable's sign based on plant_direction
	if plant_direction == 'forward':
		x=1
		y=1
	elif plant_direction == 'back':
		x=-1
		y=-1
	elif plant_direction == 'right':
		x=1
		y=-1
	elif plant_direction == 'left':
		x=-1
		y=1
	berry_positions={}
	# loop through all keys in berry_positions_dictionary
	for i in berry_positions_dictionary.keys():
		# set 3d pos of berries based on plant direction
		coord=berry_positions_dictionary[i]
		new_coord=[]
		for j in coord:
			(cx,cy,depth)=j
			if plant_direction == 'forward' or plant_direction == 'back':
				(cx,cy,depth)=(round(camPos[0]+x*cx,4),round(camPos[1]+y*depth,4),round(camPos[2]-cy-0.00035,4))
			elif plant_direction == 'right' or plant_direction == 'left':
				(cx,cy,depth)=(round(camPos[0]+x*depth,4),round(camPos[1]+y*cx,4),round(camPos[2]-cy-0.00035,4))
			new_coord.append((cx+0.005,cy-0.0027,depth+0.002))
		berry_positions[i]=new_coord
	return berry_positions, berry_positions_dictionary

def collect_berry(client_id,pos,box,camPos,plant_direction,last_berry):
	"""
	Purpose: Actuate the arm to collect berry at specified position
	---
	"""
	#open gripper
	call_open_close(client_id,"open")
	#move arm
	if plant_direction=='forward':
		moveArm(client_id,(pos[0],pos[1]-0.05,0.7),True)
	moveArm(client_id,(pos[0],pos[1],0.7),True)
	moveArm(client_id,pos,True)
	time.sleep(0.5)
	if last_berry:
		time.sleep(0.5)
	#close gripper
	call_open_close(client_id,"close")
	moveArm(client_id,(pos[0],pos[1],1),True)
	if plant_direction=='right' or plant_direction=='left':
		moveArm(client_id,(pos[0],pos[1],1),True)
		moveArm(client_id,(pos[0]-0.05,pos[1],0.5),True)
		if last_berry:
			moveArm(client_id,(box[0],pos[1],0.5),True)
		#moveArm(client_id,camPos,True)
		moveArm(client_id,camPos,True)
	time.sleep(0.1)
	if last_berry:
		if plant_direction=='forward':
			#moveArm(client_id,(box[0],box[1],0.5),True)
			moveArm(client_id,(box[0],box[1]-0.1,0.7),True)
		moveArm(client_id,box,False)
	else:
		# move arm to basket
		moveArm(client_id,(box[0],box[1],0.5),True)
		moveArm(client_id,box,True)
		time.sleep(0.7)
		# open gripper
		call_open_close(client_id,"open")

def determine_room_path(room):
	"""
	Purpose:
			Predefine paths to the room based on room entry.
	"""
	home = (4,4)
	if room[0]<home[0] and room[1]<home[1]:
		centre = (1,1)
		botPos = (0.535,0.53)
		if room == (3,0):
			pathRoom = [(3,0.2),(2,0.3),centre]
			pathRoom0 = [(2,0.2),(3,0.2),(4,0.2),home]
		elif room == (3,2):
			pathRoom = [(3,1.8),(1.5,1.8),centre]
			pathRoom0 = [(2,1.88),(3,1.88),(4,1.88),home]
		elif room == (2,3):
			pathRoom = [(1.7,4),(1.7,3),(1.5,2),centre]
			pathRoom0 = [(1.8,2),(1.8,3),(1.8,4),home]
		elif room == (0,3):
			pathRoom = [(0.8,4),(0.8,3),(0.5,2),centre]
			pathRoom0 = [(0.8,2),(0.8,3),(0.8,4),home]
	elif room[0]>home[0] and room[1]<home[1]:
		centre = (7,1)
		botPos = (3.74,0.53)
		if room == (5,0):
			pathRoom = [(5,0.2),(6,0.5),centre]
			pathRoom0 = [(6,0.2),(5,0.2),(4,0.2),home]
		elif room == (5,2):
			pathRoom = [(4,1.8),(5,1.75),(6,1.75),centre]
			pathRoom0 = [(6,1.8),(5,1.8),(4,1.8),home]
		elif room == (6,3):
			pathRoom = [(6.2,4),(6.2,3),(6.3,2),centre]
			pathRoom0 = [(6.25,2),(6.25,3),(6.25,4),home]
		elif room == (8,3):
			pathRoom = [(7.8,4),(7.8,3),(7.7,2),centre]
			pathRoom0 = [(7.8,2),(7.8,3),(7.8,4),home]
	elif room[0]<home[0] and room[1]>home[1]:
		centre = (1,7)
		botPos = (0.54,3.74)
		if room == (0,5):
			pathRoom = [(0.2,4),(0.2,5),(0.3,6),centre]
			pathRoom0 = [(0.2,6),(0.2,5),(0.2,4),home]
		elif room == (2,5):
			pathRoom = [(1.8,4),(1.8,5),(1.5,6),centre]
			pathRoom0 = [(1.8,6),(1.8,5),(1.8,4),home]
		elif room == (3,6):
			pathRoom = [(3,6.25),(2,6.35),centre]
			pathRoom0 = [(2,6.2),(3,6.2),(4,6.2),home]
		elif room == (3,8):
			pathRoom = [(3,7.8),(2,7.5),centre]
			pathRoom0 = [(2,7.8),(3,7.8),(4,7.8),home]
		elif room == (2,9):
			pathRoom = [(4,10),(2.8,10),(1.8,10),(1.8,9),(1.5,8),centre]
			pathRoom0 = [(1.8,8),(1.8,9),(1.8,10),(4,10),home]
		elif room == (0,9):
			pathRoom = [(4,10),(0.2,10),(0.2,9),(0.5,8),centre]
			pathRoom0 = [(0.2,8),(0.2,10),(4,10),home]
	elif room[0]>home[0] and room[1]>home[1]:
		centre = (7,7)
		botPos = (3.741,3.738)
		if room == (8,5):
			pathRoom = [(7.7,4),(7.7,5),(7.65,6),centre]
			pathRoom0 = [(7.8,6),(7.8,5),(7.8,4),home]
		elif room == (6,5):
			pathRoom = [(6.2,4),(6.2,5),(6.3,6),centre]
			pathRoom0 = [(6.2,6),(6.2,5),(6.2,4),home]
		elif room == (5,6):
			pathRoom = [(5,6.2),(6,6.2),centre]
			pathRoom0 = [(6,6.2),(5,6.2),(4,6.2),home]
		elif room == (5,8):
			pathRoom = [(5,7.7),(6,7.65),centre]
			pathRoom0 = [(6,7.7),(5,7.7),(4,7.7),home]
		elif room == (6,9):
			pathRoom = [(4,10),(5.2,10),(6.2,10),(6.2,9),(6.5,8),centre]
			pathRoom0 = [(6.2,8),(6.2,9),(6.2,10),(4,10),home]
		elif room == (8,9):
			pathRoom = [(4,10),(7.8,10),(7.8,9),(7.5,8),centre]
			pathRoom0 = [(7.8,8),(7.8,10),(4,10),home]
	return pathRoom, pathRoom0, centre, botPos

def deposit_berry(client_id,cb1, cb2):
	"""
	Purpose:
			Deposit berry from basket to respective collection box.
	"""
	# set collection box position
	cb1Pos=(0.54,5.85,0.4)
	cb2Pos=(3.74,5.85,0.4)
	# basket lid initialization
	_,lid0 = sim.simxGetObjectHandle(client_id,'Revolute_joint0',sim.simx_opmode_blocking)
	_,lid1 = sim.simxGetObjectHandle(client_id,'Revolute_joint1',sim.simx_opmode_blocking)
	_,lid2 = sim.simxGetObjectHandle(client_id,'Revolute_joint2',sim.simx_opmode_blocking)
	# initialize handles
	_,bm_bot = sim.simxGetObjectHandle(client_id,'BM_Bot',sim.simx_opmode_blocking)
	_,baskethandle = sim.simxGetObjectHandle(client_id,'basket',sim.simx_opmode_blocking)
	_,fs_bbasket = sim.simxGetObjectHandle(client_id,'force_sensor_bbasket',sim.simx_opmode_blocking)
	# if berries to be deposited in both collection boxes
	# elif berries to be deposited in collection box 1
	# elif berries to be deposited in collection box 2
	# set path accordingly
	pathCBs=[]
	if cb1 and cb2:
		pathCB1 = [(4,9),(2.2,10.8)]
		pathCB2 = [(6.5,10.7)]
		pathCBs = [pathCB1,pathCB2]
	elif cb1:
		pathCB1 = [(4,9),(2,11)]
		pathCBs = [pathCB1]
	elif cb2:
		pathCB2 = [(4,9),(6.5,10.7)]
		pathCBs = [pathCB2]
	# loop through boxes paths
	for pathCB in pathCBs:
		# if collection box 1
		# else(collection box 2)
		# set pos of basket and box accordingly
		if pathCB[-1]==(2.2,10.8):
			basket=(1.06,6.1)
			cb_pos = cb1Pos
			lid = lid1
			lidPos = -1.57
			pick = True
		else:
			basket=(3.22,6.1)
			cb_pos = cb2Pos
			lid = lid2
			lidPos = 1.57
			pick = False
		# pick basket only for collection box 1(basket will be held up for collection box 2)
		if pick:
			moveArm(client_id,(2.13,2.32,0.5))
			time.sleep(0.5)
			sim.simxSetJointTargetPosition(client_id,lid0,0,sim.simx_opmode_streaming)
			moveArm(client_id,(2.13,2.32,0.26))
			# move near collection box; accurate=True
			task_3.task_3_primary(client_id,pathCB)
			# unattach basket from force sensor
			_ = sim.simxSetObjectParent(client_id,baskethandle,bm_bot,True,sim.simx_opmode_blocking)
			# pick up basket
			# moveArm(client_id,(basket[0],basket[1],0.7),True)
			# sim.simxSetJointTargetPosition(client_id,lid0,0,sim.simx_opmode_streaming)
			# moveArm(client_id, (basket[0],basket[1],0.26)),True
			# time.sleep(0.5)
			call_open_close(client_id,"close")
			time.sleep(0.5)
			moveArm(client_id,(basket[0]-0.1,basket[1],0.5),True)
		else:
			# move near collection box; accurate=False
			task_3.task_3_primary(client_id,pathCB)
		# move arm above respective collection box
		moveArm(client_id, cb_pos,True)
		time.sleep(0.2)
		# open-wait-close respective basket lid
		sim.simxSetJointTargetPosition(client_id,lid,lidPos,sim.simx_opmode_streaming)
		time.sleep(1)
		sim.simxSetJointTargetPosition(client_id,lid,0,sim.simx_opmode_streaming)
		# move arm back on bot
		moveArm(client_id, (basket[0],basket[1],0.5),False)

def last_berry_check(cb1,cb2,current_box,cb_no,berries_collected):
	"""
	Purpose:
			Detect if current berry is last berry
	"""
	remaining = 0
	berries_remaining= {'Strawberry':0,'Lemon':0,'Blueberry':0}
	if cb1 and cb2:
		if cb_no==2:
			for key in current_box.keys():
				berries_remaining[key]=current_box[key]-berries_collected[key]
				#print(key,'actual berries remaining',berries_remaining[key])
				if berries_remaining[key]>=2:
					berries_remaining[key] = berries_remaining[key]-current_box[key]+2
				#print(key,'berries remaining',berries_remaining[key])
				remaining+=berries_remaining[key]
			#print('total berries remaining',remaining)
	else:
		for key in current_box.keys():
			berries_remaining[key]=current_box[key]-berries_collected[key]
			#print(key,'actual berries remaining',berries_remaining[key])
			if berries_remaining[key]>=2:
				berries_remaining[key] = berries_remaining[key]-current_box[key]+2
			#print(key,'berries remaining',berries_remaining[key])
			remaining+=berries_remaining[key]
		#print('total berries remaining',remaining)
	if remaining == 1:
		last_berry=True
	else:
		last_berry=False
	
	return last_berry

def collect_berry_from_room(client_id,room,berries_collected,current_box,cb_no,cb1,cb2):
	"""
	Purpose:
			Collect all required berry from the room and store it in respective side of basket.
	"""
	#determinig room centre and bot 3d position with the help of room entry and home position
	_, _, centre, botPos = determine_room_path(room)
	berries_remaining= {'Strawberry':0,'Lemon':0,'Blueberry':0}
	last_berry=False
	#basket lid initialization
	_,lid0 = sim.simxGetObjectHandle(client_id,'Revolute_joint0',sim.simx_opmode_blocking)
	sim.simxSetJointTargetPosition(client_id,lid0,1.57,sim.simx_opmode_streaming)
	# initialize gripper as open
	call_open_close(client_id,"open")
	# set side of basket to collect berry based on collection box
	if cb_no==1:
		basket = (botPos[0]-0.04,botPos[1]+0.12,0.275)
	else:
		basket = (botPos[0]+0.025,botPos[1]+0.12,0.275)
	# set expected cam position based on direction of plant
	if centre==(1,7):
		camPos = (botPos[0]+0,botPos[1]+0.3,0.5)
		plant_direction = 'forward'
	elif centre==(7,7):
		camPos = (botPos[0]+0.3,botPos[1]+0,0.5)
		plant_direction = 'right'
	elif centre==(7,1):
		moveArm(client_id,(botPos[0]+0.3,botPos[1]+0,0.5),True)
		camPos = (botPos[0]+0,botPos[1]-0.3,0.5)
		plant_direction = 'back'
	elif centre==(1,1):
		camPos = (botPos[0]-0.3,botPos[1]+0,0.5)
		plant_direction = 'left'
	# set arm position to given cam position for berry detection
	_,bm_bot = sim.simxGetObjectHandle(client_id,'BM_Bot',sim.simx_opmode_blocking)
	_,vs_attach = sim.simxGetObjectHandle(client_id,'vs_attach',sim.simx_opmode_blocking)
	_,vision_sensor_handle = sim.simxGetObjectHandle(client_id,"vision_sensor_2",sim.simx_opmode_blocking)
	# locate camera
	if  (cb1 and cb2 and cb_no== 1) or (cb1 and not cb2) or (cb2 and not cb1):
		moveArm(client_id,camPos,True)
		moveArm(client_id,camPos,True)
		moveArm(client_id,camPos,True)
		_ = sim.simxSetObjectParent(client_id,vision_sensor_handle,bm_bot,True,sim.simx_opmode_blocking)
	# detect berry pos from vision sensor
	berry_positions,berry_positions_dictionary = get_berries_position(client_id,camPos,plant_direction)
	# loop through keys in current box
	#print(current_box)
	for key in current_box.keys():
		#print(key,'to be picked',current_box[key])
		# loop for number of berries to be picked
		for _ in range(current_box[key]):
			#print(key,berry_positions[key])
			# if any berry
			if berry_positions[key]:
				remaining = 0
				# sort berry position based on berry z pos highest to lowest
				berry_positions[key].sort(key=lambda x: x[2],reverse = True)
				berry_positions_dictionary[key].sort(key=lambda x: x[1])
				# set position and its value(pos wrt vision sensor)
				pos = berry_positions[key][0]
				value = berry_positions_dictionary[key][0]
				# delete berry from dictionory as it will be picked
				del berry_positions[key][0]
				del berry_positions_dictionary[key][0]
				# move arm to expected cam position to send/evaluate berry pos data
				# moveArm(client_id,camPos,True)
				# moveArm(client_id,camPos,True)
				_ = send_identified_berry_data(client_id,key,round(-value[0],4),round(-value[1]+0.001,4),round(value[2],4))
				#print(cb1,cb2)
				last_berry=last_berry_check(cb1,cb2,current_box,cb_no,berries_collected)
				# put camera back on arm for last berry
				if last_berry:
					moveArm(client_id,camPos,True)
					moveArm(client_id,camPos,True)
					moveArm(client_id,camPos,True)
					_ = sim.simxSetObjectParent(client_id,vision_sensor_handle,vs_attach,True,sim.simx_opmode_blocking)
					#print('last berry of current room')
				# collect berry and put it in said side of basket
				if plant_direction=='right' or plant_direction=='left':
					moveArm(client_id,camPos,True)
				collect_berry(client_id,pos,basket,camPos,plant_direction,last_berry)
				#exclusive motion for plants in back direction
				if centre==(7,1):
					moveArm(client_id,(botPos[0]+0.3,botPos[1]+0,0.5),True)
					camPos = (botPos[0]+0,botPos[1]-0.3,0.5)
					plant_direction = 'back'
					if last_berry:
						moveArm(client_id,basket,False)
				# increment berries collected
				berries_collected[key]+=1
				#print(key,'collected',berries_collected[key])

	return berries_collected

def get_collection_box_data(data):
	"""
	Purpose:
			Arrange data in to different dict based on collection box number.
	"""
	cb1_berry = {}
	cb2_berry = {}
	if data['B'][-1] == '1' and not int(data['B'][0])==0:
		cb1_berry['Blueberry']=int(data['B'][0])
	if data['L'][-1] == '1' and not int(data['L'][0])==0:
		cb1_berry['Lemon']=int(data['L'][0])
	if data['S'][-1] == '1' and not int(data['S'][0])==0:
		cb1_berry['Strawberry']=int(data['S'][0])
	if data['B'][-1] == '2' and not int(data['B'][0])==0:
		cb2_berry['Blueberry']=int(data['B'][0])
	if data['L'][-1] == '2' and not int(data['L'][0])==0:
		cb2_berry['Lemon']=int(data['L'][0])
	if data['S'][-1] == '2' and not int(data['S'][0])==0:
		cb2_berry['Strawberry']=int(data['S'][0])
	collection_boxes = [cb1_berry,cb2_berry]

	return collection_boxes
##############################################################



def theme_implementation_primary( client_id, rooms_entry):
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
	# Opening Basket Lids
	_,lid1 = sim.simxGetObjectHandle(client_id,'Revolute_joint1',sim.simx_opmode_blocking)
	_,lid2 = sim.simxGetObjectHandle(client_id,'Revolute_joint2',sim.simx_opmode_blocking)
	sim.simxSetJointTargetPosition(client_id,lid1,0,sim.simx_opmode_streaming)
	sim.simxSetJointTargetPosition(client_id,lid2,0,sim.simx_opmode_streaming)
	# Opening JSON file
	f = open('Theme_Config.json',)
	# returns JSON object as a dictionary
	data = json.load(f)
	# initializing berries remaining dictionaries
	berries_remaining_cb1= {'Strawberry':0,'Lemon':0,'Blueberry':0}
	berries_remaining_cb2= {'Strawberry':0,'Lemon':0,'Blueberry':0}
	#creating seperate dictionaries for each collection box
	[cb1_berry,cb2_berry] = get_collection_box_data(data)   
	#initializing if collection box needs berry
	cb1=False
	cb2=False
	if cb1_berry and cb2_berry:
		cb1=True
		cb2=True
	elif cb1_berry:
		cb1=True
		cb2=False
	else:
		cb1=False
		cb2=True  
	collection_done_cb1 = True
	collection_done_cb2 = True
	#looping through all the rooms
	for room_number in range(1,5):
		collection_boxes = [cb1_berry,cb2_berry]
		# getting room entry point from rooms_entry
		room=rooms_entry[room_number-1]
		# get predefined path to the given room
		pathRoom, pathRoom0, centre, _ = determine_room_path(room)
		botPos = (2.15,2.15)
		if centre==(1,7):
			moveArm(client_id,(botPos[0]+0,botPos[1]+0.3,0.5),False)
		elif centre==(7,7):
			moveArm(client_id,(botPos[0]+0.3,botPos[1]+0,0.5),False)
		elif centre==(7,1):
			moveArm(client_id,(botPos[0]+0.3,botPos[1]+0,0.5),False)
		elif centre==(1,1):
			moveArm(client_id,(botPos[0]-0.3,botPos[1]+0,0.5),False)
		# move through the path to the given room and get to the centre of room
		task_3.task_3_primary(client_id,pathRoom,True)
		# #loop through boxes to solve one box at a time
		for current_box in collection_boxes: 
			# initialize if to collect
			collect = False
			# collect only if said amount of berry is greater than 0
			for key in current_box.keys():
				if current_box[key]>0:
					collect = True
			# see if there is any berry to be picked up
			if current_box and collect:					        
				#initialize berry collected data
				berries_collected_cb1 = {'Strawberry':0,'Lemon':0,'Blueberry':0}
				berries_collected_cb2 = {'Strawberry':0,'Lemon':0,'Blueberry':0}
				# collect as much required berry as possible
				#set different variables based on box number
				if current_box==collection_boxes[0]:
					cb1=True
					cb_no = 1
					# collect of required berry from the room and put it in basket. return dict of berries collected
					berries_collected_cb1=collect_berry_from_room(client_id,room,berries_collected_cb1,current_box,cb_no,cb1,cb2)
					# calculate remaining berry
					for key in cb1_berry.keys():
						berries_remaining_cb1[key]=cb1_berry[key]-berries_collected_cb1[key]
					#print(berries_remaining_cb1)
					#initialize if collection done
					collection_done_cb1 = True
					cb1 = False
					#if no remaining berries sets collection done as true else false
					for key in cb1_berry.keys():
						#update current box data for remaining berries
						cb1_berry[key]=berries_remaining_cb1[key]
						if not berries_remaining_cb1[key]==0:
							collection_done_cb1=False
							cb1 = True
				else:
					cb2=True
					cb_no = 2
					# collect of required berry from the room and put it in basket. return dict of berries collected
					berries_collected_cb2=collect_berry_from_room(client_id,room,berries_collected_cb2,current_box,cb_no,cb1,cb2)
					# calculate remaining berry
					for key in cb2_berry.keys():
						berries_remaining_cb2[key]=cb2_berry[key]-berries_collected_cb2[key]
					#print(berries_remaining_cb2)
					#initialize if collection done
					collection_done_cb2 = True
					cb2 = False
					#if no remaining berries set collection done as true else false
					for key in cb2_berry.keys():
						#update current box data for remaining berries
						cb2_berry[key]=berries_remaining_cb2[key]
						if not berries_remaining_cb2[key]==0:
							collection_done_cb2=False
							cb2=True
		#move back out of the room
		task_3.task_3_primary(client_id,pathRoom0,True)
		call_open_close(client_id,"open")
		# if collection done then break
		#print(collection_done_cb1,collection_done_cb2)
		if collection_done_cb1 and collection_done_cb2:
			break
	#deposit berry to said collection box
	if cb1_berry and cb2_berry:
		cb1=True
		cb2=True
	elif cb1_berry:
		cb1=True
		cb2=False
	else:
		cb1=False
		cb2=True  
	deposit_berry(client_id,cb1,cb2)


if __name__ == "__main__":

	# Room entry co-ordinate
	#rooms_entry = [(3,2)]     # example list of tuples
	rooms_entry = [(5, 2), (3, 2)]   # example list of tuples

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