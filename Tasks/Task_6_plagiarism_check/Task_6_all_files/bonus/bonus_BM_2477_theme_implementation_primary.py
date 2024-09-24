'''
*
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
*
'''


# Team ID:			[ 2477 ]
# Author List:		[ Swadhesh P U, Bala Sakthi R, Harshini T G R , Sankar Ganesh M]
# Filename:			[ theme_implementation.py ]
# Theme:			[ Berryminator ]
# Functions:		[ init_remote_api_server, start_simulation, stop_simulation, exit_remote_api_server, adjust_position, nav_to_room, 
# 						rotate_bot_right, rotate_bot_left, nav_outof_room, reach_cb, theme_implementation_primary]
# Global variables:	None 


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
import task_3
import task_4
import task_2a
import task_1b
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

	return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

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

	sim.simxFinish(client_id)

	##################################################

def adjust_position(client_id, wheel_joints):

	"""
	Purpose:
	---
	This function adjust the position of the bot if qr_code is not in center of vision sensor image.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`wheel_joints` 	:  [ list ]
		the list of handles of wheel joints
	
	Returns:
	---
	None
	
	Example call:
	---
	adjust_position(client_id, wheel_joints)
	
	"""

	############### To check whether Bot reached target qr ###############

	while(True):
		vision_sensor_image, image_resolution, return_code = task_3.get_vision_sensor_image(client_id)
		transformed_image = task_3.transform_vision_sensor_image(vision_sensor_image, image_resolution)

		if(len(task_3.detect_qr_codes(transformed_image))!=0):
			qr_midpoint=task_3.detect_qr_codes(transformed_image)[0][1]
			if(qr_midpoint[0] > 70):
				initial = task_3.encoders(client_id)[0]
				task_3.moveleftright(client_id, wheel_joints, -2)
				while(True):
					final = task_3.encoders(client_id)[0]
					if(abs(final - initial) >= 0.3):
						task_3.moveleftright(client_id, wheel_joints, 0)
						break
			elif(qr_midpoint[0] < 50):
				initial = task_3.encoders(client_id)[0]
				task_3.moveleftright(client_id, wheel_joints, +2)
				while(True):
					final = task_3.encoders(client_id)[0]
					if(abs(final - initial) >= 0.3):
						task_3.moveleftright(client_id, wheel_joints, 0)
						break
			if(qr_midpoint[1] > 70):
				initial = task_3.encoders(client_id)[0]
				task_3.moveforback(client_id, wheel_joints, 2)
				while(True):
					final = task_3.encoders(client_id)[0]
					if(abs(final - initial) >= 0.3):
						task_3.moveforback(client_id, wheel_joints, 0)
						break
			elif(qr_midpoint[1] < 50):
				initial = task_3.encoders(client_id)[0]
				task_3.moveforback(client_id, wheel_joints, -2)
				while(True):
					final = task_3.encoders(client_id)[0]
					if(abs(final - initial) >= 0.3):
						task_3.moveforback(client_id, wheel_joints, 0)
						break
			break

	##########################################################################

def nav_to_room(client_id, rooms_entry, index, Berry_count):

	"""
	Purpose:
	---
	This function navigates bot to the qr in front of the berry rack
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`rooms_entry` 	:  [ list ]
		the list of entry coordinates of rooms
	'index' 	:  [ integer ]
		the index of rooms_entry where to be entered
	'Berry_count' 	:  [ dictionary ]
		the dictionary that contains count of each berries to be plucked
	
	Returns:
	---
	None
	
	Example call:
	---
	nav_to_room(client_id, rooms_entry, index, Berry_count)
	
	"""
	
	##############	Navigating to Berries Rack	##############

	wheel_joints = task_3.init_setup(client_id)
	room_entry=rooms_entry[index]
	if(index != 0):
		prev_room_entry = rooms_entry[index - 1]
	else:
		prev_room_entry = None


	if(room_entry in [(0,5), (2,5), (0,3), (2,3)]):			
		task_3.task_3_primary(client_id, [(1,4)])
		if(room_entry in [(0,5), (2,5)]):
			if(room_entry == (0,5)):
				task_3.task_3_primary(client_id, [(0,6), (1,7)])
			elif(room_entry == (2,5)):
				task_3.task_3_primary(client_id, [(2,6), (1,7)])
				adjust_position(client_id, wheel_joints)
		elif(room_entry in [(0,3), (2,3)]):
			if(room_entry == (0,3)):
				task_3.task_3_primary(client_id, [(0,2), (1,1)])
			elif(room_entry == (2,3)):
				task_3.task_3_primary(client_id, [(2,2), (1,1)])


	if(room_entry in [(6,5), (8,5), (6,3), (8,3)]):		
		if(prev_room_entry not in [(6,5)]):
			task_3.task_3_primary(client_id, [(7,4)])
		if(room_entry in [(6,5), (8,5)]):
			if(room_entry == (6,5)):
				task_3.task_3_primary(client_id, [(6,6), (7,7)])
			elif(room_entry == (8,5)):
				task_3.task_3_primary(client_id, [(8,6), (7,7)])
		elif(room_entry in [(6,3), (8,3)]):
			if(room_entry == (6,3)):
				task_3.task_3_primary(client_id, [(6,2), (7,1)])
			elif(room_entry == (8,3)):
				task_3.task_3_primary(client_id, [(8,2), (7,1)])


	if(room_entry in [(3,6), (3,8), (5,6), (5,8)]):			
		
		if(room_entry in [(3,6), (3,8)]):
			rotate_bot_left(client_id, wheel_joints, 12.2)
		elif(room_entry in [(5,6), (5,8)]):
			if(prev_room_entry == (3,6)):
				rotate_bot_right(client_id, wheel_joints, 12.2)
			else:
				rotate_bot_right(client_id, wheel_joints, 12.2)
		if(prev_room_entry != (3,6)):
			task_3.task_3_primary(client_id, [(4,7)])
		if(room_entry in [(3,6), (3,8)]):
			if(room_entry == (3,6)):
				task_3.task_3_primary(client_id, [(2,6)])
			elif(room_entry == (3,8)):
				task_3.task_3_primary(client_id, [(2,8)])
			task_3.task_3_primary(client_id, [(1,7)])
		elif(room_entry in [(5,6), (5,8)]):
			if(room_entry == (5,6)):
				task_3.task_3_primary(client_id, [(6,6)])
			elif(room_entry == (5,8)):
				task_3.task_3_primary(client_id, [(6,8)])
			task_3.task_3_primary(client_id, [(7,7)])


	if(room_entry in [(3,0), (5,0), (3,2), (5,2)]):		

		if(prev_room_entry not in [(5,2)]):
			task_3.task_3_primary(client_id, [(4,1)])
		if(room_entry in [(3,0), (3,2)]):
			if(prev_room_entry not in [(5,2)]):
				rotate_bot_left(client_id,wheel_joints, 12.7)
			else:
				rotate_bot_left(client_id,wheel_joints, 12.1)
		elif(room_entry in [(5,0), (5,2)] and prev_room_entry not in [(5,6),(5,8)]):
			rotate_bot_right(client_id, wheel_joints, 11.7)
		if(room_entry in [(3,0),(3,2)]):
			if(room_entry == (3,0)):
				task_3.task_3_primary(client_id, [(2,0)])
			elif(room_entry == (3,2)):
				task_3.task_3_primary(client_id, [(2,2)])
			task_3.task_3_primary(client_id, [(1,1)])
		elif(room_entry in [(5,0), (5,2)]):
			if(room_entry == (5,0)):
				task_3.task_3_primary(client_id, [(6,0)])
			elif(room_entry == (5,2)):
				task_3.task_3_primary(client_id, [(6,2)])
			task_3.task_3_primary(client_id, [(7,1)])


	if(room_entry in [(6,5),(3,6),(5,2),(2,3)]):								
		adjust_position(client_id, wheel_joints)

	if(room_entry in [(3,6), (6,5), (5,2)]):					# To face towards berry
		rotate_bot_right(client_id, wheel_joints, 12.1)
	
	elif(room_entry in [(2,3)]):								# To face towards berry
		rotate_bot_left(client_id, wheel_joints, 12.1)
	
	elif(room_entry in [(6,3), (8,3)]):							# To face towards berry
		rotate_bot_right(client_id, wheel_joints, 24.7)

		
	##################################################

def rotate_bot_right(client_id, wheel_joints, value):

	"""
	Purpose:
	---
	This function rotate the bot clockwise until given encoder value reached
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`wheel_joints` 	:  [ list ]
		the list of handles of wheel joints
	'value' 	:  [ integer ]
		the distance covered by wheel upto which the bot wheel rotates
	
	Returns:
	---
	None
	
	Example call:
	---
	rotate_bot_right(client_id, wheel_joints, value)
	
	"""

	############## To rotate 90 degree right ##############
	initial = task_3.encoders(client_id)[0]
	task_3.rotate_right(client_id, wheel_joints, 2)
	while(True):
		final = task_3.encoders(client_id)[0]
		if(abs(final-initial) >= value):
			task_3.rotate_right(client_id, wheel_joints, 0)
			break
	#######################################################

def rotate_bot_left(client_id, wheel_joints, value):

	"""
	Purpose:
	---
	This function rotate the bot anti-clockwise until given encoder value reached
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`wheel_joints` 	:  [ list ]
		the list of handles of wheel joints
	'value' 	:  [ integer ]
		the distance covered by wheel upto which the bot wheel rotates
	
	Returns:
	---
	None
	
	Example call:
	---
	rotate_bot_left(client_id, wheel_joints, value)
	
	"""

	############## To rotate 90 degree left ##############
	initial = task_3.encoders(client_id)[0]
	task_3.rotate_left(client_id, wheel_joints, 2)
	while(True):
		final = task_3.encoders(client_id)[0]
		if(abs(final-initial) >= value):
			task_3.rotate_left(client_id, wheel_joints, 0)
			break
	######################################################
		
###########################################################

def nav_outof_room(client_id, rooms_entry, index, Berry_count):

	"""
	Purpose:
	---
	This function navigates bot to the home position
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`rooms_entry` 	:  [ list ]
		the list of entry coordinates of rooms
	'index' 	:  [ integer ]
		the index of rooms_entry where to be entered
	'Berry_count' 	:  [ dictionary ]
		the dictionary that contains count of each berries to be plucked
	
	Returns:
	---
	None
	
	Example call:
	---
	nav_outof_room(client_id, rooms_entry, index, Berry_count)
	
	"""

	############## To navigate out of the room ##############

	wheel_joints = task_3.init_setup(client_id)
	room_entry = rooms_entry[index]
	if(index < len(rooms_entry) - 1):
		nxt_room_entry = rooms_entry[index + 1]
	else:
		nxt_room_entry = None
	Berry_count = Berry_count['L'] + Berry_count['B'] + Berry_count['S']

	if(room_entry in [(0,5), (2,5), (2,3)]):		
		
		if(room_entry in [(2,3)]):
			rotate_bot_right(client_id, wheel_joints, 12.1)
		task_3.task_3_primary(client_id, [room_entry,(1,4)])
		adjust_position(client_id, wheel_joints)
		if(nxt_room_entry != (6,5)):
			task_3.task_3_primary(client_id, [(4,4)])
			adjust_position(client_id, wheel_joints)
		

	if(room_entry in [(6,5), (6,3), (8,3)]):		
		
		if(room_entry in [(6,3), (8,3)]):
			rotate_bot_left(client_id, wheel_joints, 24.7)
			task_3.task_3_primary(client_id, [room_entry, (7,4)])
			adjust_position(client_id, wheel_joints)
			if(nxt_room_entry not in [(0,3),(2,3)] or Berry_count == 0):
				task_3.task_3_primary(client_id, [(4,4)])
				adjust_position(client_id, wheel_joints)
		if(room_entry in [(6,5)]):
			if(room_entry == (6,5)):
				rotate_bot_left(client_id, wheel_joints, 12.0)
			task_3.task_3_primary(client_id, [room_entry, (7,4)]) 
			if(nxt_room_entry not in [(6,3), (8,3)] or Berry_count == 0):
				task_3.task_3_primary(client_id, [(4,4)])


	if(room_entry in [(3,6),(5,6),(5,8)]):			
		
		if(room_entry in [(3,6)]):
			rotate_bot_left(client_id, wheel_joints, 12)
			task_3.task_3_primary(client_id, [room_entry, (4,7)]) 

			rotate_bot_right(client_id, wheel_joints, 12.2)
			adjust_position(client_id, wheel_joints)
			if(nxt_room_entry not in [(5,6), (5,8)]) :
				task_3.task_3_primary(client_id, [(4,4)]) 

		elif(room_entry in [(5,6), (5,8)]):			
			task_3.task_3_primary(client_id, [room_entry, (4,7)]) 
			
			if( Berry_count == 0):
				adjust_position(client_id, wheel_joints)
				task_3.task_3_primary(client_id, [(4,9)])
				rotate_bot_left(client_id, wheel_joints, 12.6)
			
			elif(Berry_count != 0):
				if( nxt_room_entry != (5,2)):
					rotate_bot_left(client_id, wheel_joints, 12.2)
					adjust_position(client_id, wheel_joints)
					task_3.task_3_primary(client_id, [(4,4)]) 
				else:
					adjust_position(client_id, wheel_joints)
					task_3.task_3_primary(client_id, [(4,1)])

	if(room_entry in [(3,0), (3,2), (5,2)]):			

		if(room_entry in [(3,0), (3,2)]):
			task_3.task_3_primary(client_id, [room_entry, (4,1)])
			adjust_position(client_id, wheel_joints)
			task_3.task_3_primary(client_id, [(4,9)])
			rotate_bot_right(client_id, wheel_joints, 12.3)

		elif(room_entry in [(5,2)]):
			rotate_bot_left(client_id, wheel_joints, 12.0)
			task_3.task_3_primary(client_id, [room_entry, (4,1)])
			if(nxt_room_entry  in [(3,0), (3,2)] and Berry_count != 0):
				rotate_bot_left(client_id, wheel_joints, 12.3)
				adjust_position(client_id, wheel_joints)
			elif(Berry_count != 0):
				task_3.task_3_primary(client_id, [(4,4)])
				adjust_position(client_id, wheel_joints)
				rotate_bot_left(client_id, wheel_joints, 12.1)
			else:
				task_3.task_3_primary(client_id, [(4,9)])
				rotate_bot_left(client_id, wheel_joints, 12.6)

	########################################################

def reach_cb(client_id, Berry_details):

	"""
	Purpose:
	---
	This function navigates bot to qr near CB(s) and deposits the berries 
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	'Berry_details' 	:  [ dictionary ]
		the dictionary that contains the JSON file contents
	
	Returns:
	---
	None
	
	Example call:
	---
	reach_cb(client_id, Berry_details)
	
	"""

	############## To reach the collection box and drop the berries successfully inside them ##############

	wheel_joints = task_3.init_setup(client_id)

	CB_1_count = 0
	CB_2_count = 0
	for i in Berry_details:
		if(Berry_details[i][2:5] == "CB1"):
			CB_1_count += int(Berry_details[i][0])
		else:
			CB_2_count += int(Berry_details[i][0])
	

	task_3.task_3_primary(client_id, [(4,9), (2,11)])	
	return_code, joint_handle1 = sim.simxGetObjectHandle(client_id, 'container_joint', sim.simx_opmode_blocking)
	return_code, joint_handle2 = sim.simxGetObjectHandle(client_id, 'door1_joint', sim.simx_opmode_blocking)
	return_code, joint_handle3 = sim.simxGetObjectHandle(client_id, 'door2_joint', sim.simx_opmode_blocking)
	
	initial = task_3.encoders(client_id)[0]
	task_3.moveforback(client_id, wheel_joints, 2)
	while(True):
		final = task_3.encoders(client_id)[0]
		if(abs(final - initial) >= 2.2):
			break
	initial = task_3.encoders(client_id)[0]
	task_3.moveleftright(client_id, wheel_joints, -2)
	while(True):
		final = task_3.encoders(client_id)[0]
		if(abs(final - initial) >= 2.6):
			break
	task_3.moveforback(client_id, wheel_joints, 0)

	sim.simxSetJointTargetPosition(client_id, joint_handle1, 20 * math.pi / 180, sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id, joint_handle2, 90 * math.pi / 180, sim.simx_opmode_oneshot)
	
	returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
	returnCode, position2 = sim.simxGetJointPosition(client_id, joint_handle2, sim.simx_opmode_blocking)

	while(round(position1, 2) != round(20 * math.pi / 180, 2) or round(position2, 2)!=round(90 * math.pi / 180, 2)):

		returnCode,position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
		returnCode,position2 = sim.simxGetJointPosition(client_id, joint_handle2, sim.simx_opmode_blocking)

	sim.simxSetJointTargetPosition(client_id, joint_handle1, 0 * math.pi / 180, sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id, joint_handle2, 0 * math.pi / 180, sim.simx_opmode_oneshot)

	if( CB_2_count == 0):

		initial = task_3.encoders(client_id)[0]
		task_3.moveleftright(client_id, wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 3.5):
				break
		task_3.moveforback(client_id, wheel_joints, 0)

		rotate_bot_right(client_id, wheel_joints, 24.6)

		initial = task_3.encoders(client_id)[0]
		task_3.moveleftright(client_id, wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 3.5):
				break
		task_3.moveforback(client_id, wheel_joints, 0)

		initial = task_3.encoders(client_id)[0]
		task_3.moveforback(client_id, wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 3.5):
				break
		task_3.moveforback(client_id, wheel_joints, 0)

		sim.simxSetJointTargetPosition(client_id, joint_handle1, -20 * math.pi / 180, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id, joint_handle3, -90 * math.pi / 180, sim.simx_opmode_oneshot)
		
		returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
		returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)

		while(round(position1, 2) != round(-20 * math.pi / 180, 2) or round(position3, 2) != round(-90 * math.pi / 180, 2)):

			returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
			returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)
		
		sim.simxSetJointTargetPosition(client_id, joint_handle1, 0 * math.pi / 180, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id, joint_handle3, 0 * math.pi / 180, sim.simx_opmode_oneshot)
		
		returnCode,position1 = sim.simxGetJointPosition(client_id,joint_handle1, sim.simx_opmode_blocking)
		returnCode,position3 = sim.simxGetJointPosition(client_id,joint_handle3, sim.simx_opmode_blocking)

		while(round(position1, 2) != round(0 * math.pi / 180, 2) or round(position3, 2)!=round(0 * math.pi / 180, 2)):

			returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
			returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)

	if(CB_2_count > 0):

		initial = task_3.encoders(client_id)[0]
		task_3.moveleftright(client_id, wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 2.7):
				break
		
		initial = task_3.encoders(client_id)[0]
		task_3.moveforback(client_id, wheel_joints, -2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 1.8):
				break
		task_3.moveforback(client_id, wheel_joints, 0)

		task_3.task_3_primary(client_id, [(6,11)])

		initial = task_3.encoders(client_id)[0]
		task_3.moveforback(client_id,wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 2.2):
				break
		
		initial = task_3.encoders(client_id)[0]
		task_3.moveleftright(client_id, wheel_joints, 2)
		while(True):
			final = task_3.encoders(client_id)[0]
			if(abs(final - initial) >= 2.8):
				break
			
		task_3.moveforback(client_id, wheel_joints, 0)
		sim.simxSetJointTargetPosition(client_id, joint_handle1, -20 * math.pi / 180, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id, joint_handle3, -90 * math.pi / 180, sim.simx_opmode_oneshot)
		
		returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
		returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)

		while(round(position1, 2) != round(-20 * math.pi / 180, 2) or round(position3, 2) != round(-90 * math.pi / 180, 2)):

			returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
			returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)
		
		sim.simxSetJointTargetPosition(client_id, joint_handle1, 0 * math.pi / 180, sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id, joint_handle3, 0 * math.pi / 180, sim.simx_opmode_oneshot)
		
		returnCode,position1 = sim.simxGetJointPosition(client_id,joint_handle1, sim.simx_opmode_blocking)
		returnCode,position3 = sim.simxGetJointPosition(client_id,joint_handle3, sim.simx_opmode_blocking)

		while(round(position1, 2) != round(0 * math.pi / 180, 2) or round(position3, 2)!=round(0 * math.pi / 180, 2)):

			returnCode, position1 = sim.simxGetJointPosition(client_id, joint_handle1, sim.simx_opmode_blocking)
			returnCode, position3 = sim.simxGetJointPosition(client_id, joint_handle3, sim.simx_opmode_blocking)

	#######################################################################################################

def theme_implementation_primary( client_id, rooms_entry):
	"""
	Purpose:
	---
	This is the only  function that is called from the main function. Make sure to fill it
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
	
	##############	Decoding JSON file	##############

	f = open('Theme_Config.json')
	data = json.load(f)
	Berry_details = {}
	Berry_count = {}
	for i in data:
		Berry_details[i] = data[i]
		Berry_count[i] = int(data[i][0])
	f.close()

	######### To Set default joint position  #########

	return_code, joint_handle1 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking)
	return_code, joint_handle2 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r2', sim.simx_opmode_blocking)
	return_code, joint_handle3 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r3', sim.simx_opmode_blocking)

	sim.simxSetJointTargetPosition(client_id,joint_handle1,0*math.pi/180,sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id,joint_handle2,-25*math.pi/180,sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id,joint_handle3,25*math.pi/180,sim.simx_opmode_oneshot)

	##################################################
	
	i = 0
	while(Berry_count['L'] != 0 or Berry_count['S'] != 0 or Berry_count['B'] != 0):
		
		nav_to_room(client_id, rooms_entry, i, Berry_count)
		berry_positions_dictionary = task_4.detect_position(client_id)
		Berry_count=task_4.pluck_berries(client_id, berry_positions_dictionary, Berry_details, Berry_count)
		nav_outof_room(client_id, rooms_entry, i, Berry_count)
		i += 1
	
	reach_cb(client_id, Berry_details)

	##################################################

if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [ (2,5), (5,8), (5,2), (3,0)]     # example list of tuples

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

		