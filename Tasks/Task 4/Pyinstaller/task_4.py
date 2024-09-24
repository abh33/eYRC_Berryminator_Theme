'''
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
'''


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

import string
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode


import task_1b
import task_2a
import task_3

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()


# try:
# 		task_1b = __import__('task_1b')
# 		task_2a = __import__('task_2a')
# 		task_3  = __import__('task_3')
# except:
# 	print('task_1b or task_2a not in directory')




################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


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
	# Just in case, close all opened connections
	sim.simxFinish(-1)

	# Connect to CoppeliaSim
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

	# Start the simulation
	if client_id != -1:
		return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
	
	# Making sure that last command sent out had time to arrive
	sim.simxGetPingTime(client_id)	
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


	vision_sensor_image = []
	image_resolution = []
	return_code = 0

	##############	ADD YOUR CODE HERE	##############

	return_code, vision_sensor = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	# print('vision sensor get handle return code: ', return_code, vision_sensor)
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vision_sensor, 0, sim.simx_opmode_blocking)
	# print('vision sensor get image return code: ', return_code)
	# print(image_resolution)	

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

	transformed_image = np.array(vision_sensor_image, dtype=np.uint8)
	transformed_image.resize((image_resolution[0], image_resolution[1], 3))
	transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
	transformed_image = cv2.flip(transformed_image, 0)	

	##################################################

	return transformed_image


def stop_simulation(client_id):
	"""
	Purpose:
	---
	This function should stop the running simulation in CoppeliaSim server.
	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
	The test_task_1c executable script will handle that condition.
	
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
	
	# Stop the simulation
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)

	# Making sure that last command sent out had time to arrive
	sim.simxGetPingTime(client_id)
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
	# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
	sim.simxGetPingTime(client_id)

	# Now close the connection to CoppeliaSim
	sim.simxFinish(client_id)

	##################################################

#TODO: Needs to be given to teams along with the executable.

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



# dummy_positions = []
# dummy_handles = []
# random_berry=[]
# def insert_dummies(client_id, berry_positions_dictionary, vision_sensor_handle, arm_handle, target_handle):

# 	global random_berry
# 	for i in berry_positions_dictionary.values():
# 		if i != []:
# 			random_berry = random.choice(i)

# 	return random_berry

def berry_to_pick(berry_positions_dictionary, color):

	out = []
	dist = 99
	for i in berry_positions_dictionary[color]:
		if i[2] < dist:
			dist = i[2]
			out  = i
		# dist.append(i[2])
			
	return out


def berry_to_pick_2(berry_positions_dictionary, color):
	out = []
	# dist = 99
	for i in berry_positions_dictionary[color]:
		if (122 <= i[3] <= 133) and (122 <= i[3] <= 133):
			# dist = i[2]
			out  = i
		# dist.append(i[2])

	return out


previous_berry_position = []


def rotate_gripper(client_id, arm_joint_handles):


	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
		
	berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	

	for i in berry_positions_dictionary.values():
		for j in i:
			if (90 <= j[3] <= 160) and (90 <= j[4] <= 160):
				if (0.13 <= j[2] <= 0.22):
					returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[5], 4.71, sim.simx_opmode_blocking)
					rotate_gripper(client_id, arm_joint_handles)

	returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[5], 0.1959, sim.simx_opmode_blocking)




def hold(client_id):
	global previous_berry_position

	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)


	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	try:	
		berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
		berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	except:
		berry_positions_dictionary  = {}


	for i in berry_positions_dictionary.values():
		for j in i:
			if (122 <= j[3] <= 133) and (110 <= j[4] <= 145):
				print('')
				# print(j[2])
				# print(type(j[2]))
				if round(j[2],2) <= 0.24:
					if j == previous_berry_position:
						# time.sleep(2.)
						print("Exiting hold")
						return 1
						# print("TRUE")
				previous_berry_position = j

	
	#time.sleep(0.5)
	r = hold(client_id)


previous_berry_position_dict = {"dfsd":[]}

def hold_2(client_id, which):
	global previous_berry_position_dict

	if which == "IK":
		emptybuff = bytearray()
		return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)

	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	try:	
		berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
		berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	except:
		berry_positions_dictionary  = {}



	# for i in berry_positions_dictionary.values():
	# 	for j in i:


	# 		if (122 <= j[3] <= 133) and (122 <= j[3] <= 133):
				
	# 			print(j[2])
	# 			# print(type(j[2]))
	# 			if round(j[2],2) <= 0.2:
	# 				if j == previous_berry_position:
	# 					# time.sleep(2.)
	# 					print("Exiting hold")
	# 					return 1
	# 					# print("TRUE")
	# 			previous_berry_position = j

	# print("OLD")
	# print(previous_berry_position_dict)
	# print("NEW")
	# print(berry_positions_dictionary)

	
	
	#time.sleep(0.5)
	if berry_positions_dictionary != previous_berry_position_dict:
		previous_berry_position_dict = berry_positions_dictionary
		hold_2(client_id, which)
	else:
		print('true')




def call_move_target_dummy(client_id, pose, ref_frame):

	# pose = [str(pose)]
	pose = [str(pose[0]) +"%"+ str(pose[1]) + "%"+ str(pose[2]) + "%" + str(pose[3]) + "%" + str(pose[4]) + "%" + str(pose[5]) + "%" + str(pose[6])]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'move_target_dummy',[ref_frame],[],pose,emptybuff,sim.simx_opmode_blocking)


def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)


def task_4_detect_berries( client_id, vision_sensor_handle, required_berry):

	# Detecting berries
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
		
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
		
	berry_positions_dictionary  = {}
	berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	#print('Positions: ',berry_positions_dictionary)

	# Finding nearest berry
	berry_position = berry_to_pick(berry_positions_dictionary, required_berry)

	return_code=send_identified_berry_data(client_id,required_berry,berry_position[0],berry_position[1],berry_position[2])
	
	if(return_code==0):
		print('---------------------------------------------------------------')
		print('Identified Berry Data sent successfully')
		print('Data Sent-')
		print(required_berry,berry_position[0],berry_position[1],berry_position[2])
		print('---------------------------------------------------------------')

	return berry_position


def get_arm_joint_handles(client_id):
	
	arm_joint_handles = [0,0,0,0,0,0]
	
	for i in range(6):
		if i == 0:
			r, arm_joint_handles[i] = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking)
		else:
			r, arm_joint_handles[i] = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_'+str(i)+str(i+1), sim.simx_opmode_blocking)

	return arm_joint_handles


def move_arm_using_fk(client_id, arm_joint_handles, arm_joint_values):

	for i in range(6):
		returnCode = sim.simxSetJointTargetPosition( client_id, arm_joint_handles[i], arm_joint_values[i], sim.simx_opmode_blocking)


def call_ik():

	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'ik',[],[],[],emptybuff,sim.simx_opmode_blocking)



def task_4_primary( client_id):

	task_3.task_3_primary(client_id, [(4,4)])

	# Get joint handles
	arm_joint_handles = get_arm_joint_handles(client_id)
 
	# Required berries in order
	required_berries = ["Strawberry", "Lemon", "Blueberry"]
	#required_berries = ["Blueberry"]

	# Waiting for 1 simulation sec so that at least one time step is simulated
	return_code, init_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
	check = 0
	while check <= 1.0:
		return_code, new_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
		if return_code == sim.simx_return_remote_error_flag:
			print("[ERROR] Task 4 scene main script was tampered. Exiting ...")
			sys.exit()
		if new_sim_time == '':
			new_sim_time = '0'
		check = float(new_sim_time)

	# Opening the gripper, if not opened already
	call_open_close(client_id, "open")

	# Getting the handles
	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	return_code, arm_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm', sim.simx_opmode_blocking)
	return_code, target_handle = sim.simxGetObjectHandle(client_id, 'target', sim.simx_opmode_blocking)

	# Detection and drop poses
	# Order: Strawberry, Lemon, Blueberry
	det_poses = [ [-0.3241951466, 0.2503266335, 0.2305259705, 0.4223516583, -0.5737341642, -0.4877355099, 0.5045418143],
			      [-0.04515767097, 0.2664012909, 0.441881597, 0.4994738996, -0.4998606145, -0.500082314, 0.5005825162],
			      [0.3072581291, 0.3182327747, 0.2113682181, 0.4234820604, -0.5760093927, -0.4873718023, 0.5013431907] ]

	drop_poses = [ [-0.5031784773, 0.1173064709, 0.2308821529, 0.4223517478, -0.5737342238, -0.4877355993, 0.5045414567],
				   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431],
				   [-0.5066001415, 0.0784137249, 0.4445034266, 0.7064390779, -0.0298046805, 0.02858715504, 0.7065679431] ]

	# Detection joint angles ... for FK
	det_angles = [ [1.3395402, 0.349066, 1.65806, -1.6231, -2.96706, 0.471239],
				   [-0.46705011, -0.92380277, 1.93923533, -1.0449286, -1.22173, 0.0137305052],
				   [-1.5708, 0.2583087, 2.07083316, -2.3090706, -0.1347394,-0.04502949]]

	home = [-0.00430398194, -1.1887438, 2.6755897, -1.3182472, -1.7058848, 0.0034191]

	for i in required_berries:
		# Index
		if i == "Strawberry":
			index = 0
		elif i == "Lemon":
			index = 1
		elif i == "Blueberry":
			index = 2
			# Going to home
			angles = home
			move_arm_using_fk(client_id, arm_joint_handles, angles)
			hold_2(client_id, "FK")


		# # Going to detection pose first using IK
		# pose = det_poses[index]
		# call_move_target_dummy(client_id, pose, 0)    # 0, since wrt base
		# hold_2(client_id)

		# Going to detection pose first using FK
		angles = det_angles[index]
		move_arm_using_fk(client_id, arm_joint_handles, angles)
		hold_2(client_id, "FK")

		# input("Proceed?")

		# Detecting berries and getting the position of nearest berry wrt VS
		berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

		# Going to NEAR berry 
		# print("GOING NEAR")
		pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
		call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
		# time.sleep(10.)
		r = hold_2(client_id, "IK")


		# Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
		berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

		# Going to NEAR berry 2
		# print("GOING NEAR")
		pose = [berry_position[0], berry_position[1], berry_position[2]-0.07, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
		call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
		# time.sleep(10.)
		r = hold_2(client_id, "IK")

		# Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
		berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

		# Going to NEAR berry 3
		# print("GOING NEAR")
		pose = [berry_position[0], berry_position[1], berry_position[2]-0.05, 0.4994738996, -0.4998605847, -0.500082314, 0.5005825758]  # position wrt VS, orientation wrt base
		call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
		# time.sleep(10.)
		r = hold_2(client_id, "IK")

		# Detecting berries and getting the position of nearest berry wrt VS i.e. at the center
		berry_position = task_4_detect_berries( client_id, vision_sensor_handle, i)

		# Going to berry
		# print("Going to berry")
		pose = [berry_position[0], berry_position[1], berry_position[2], 0.5595337152, -0.4297599792, -0.5619855523, 0.4317415357]
		call_move_target_dummy(client_id, pose, 1)  # wrt vision sensor
		r = hold(client_id)

		# Closing the gripper
		call_open_close(client_id, "close")
		time.sleep(2.)

		# # Going to detection pose
		# pose = det_poses[index]
		# if i == "Strawberry":
		# 	pose = [1.3395402, 0.122173, 1.65806, -1.6231,-1.5708, 0.471239]
		# call_move_target_dummy(client_id, pose, 0)    # 0, since wrt base
		# hold_2(client_id, "IK")

		# Going to detection pose using FK
		angles = det_angles[index]
		move_arm_using_fk(client_id, arm_joint_handles, angles)
		hold_2(client_id, "FK")

		# Going to drop pose
		pose = drop_poses[index]
		call_move_target_dummy(client_id, pose, 0)    # 0, since wrt base
		hold_2(client_id, "IK")
		# time.sleep(5.)

		# Opening the gripper
		call_open_close(client_id, "open")
		time.sleep(2.)

		rotate_gripper(client_id, arm_joint_handles)


if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(4,0),(4,10),(2,10),(2,0)]    # You can give any number of different co-ordinates


	##################################################
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

		task_4_primary(client_id)
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
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()