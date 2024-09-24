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


# Team ID:			[ eYRC#BM#2783 ]
# Author List:		[  B Nandhkishore, Sadha Sivam M, L G Divyanth ]
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
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()

##############################################################
#Introduction-
#This code provides the functionality to direct the traversal of bot through the provided target points
##############################################################


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
def neighbour_determine(current):
	"""
	Purpose
	---
	This function determines the neighbourhood points of a given point in the QR plane

	Input Arguments
	---
	`current`		: (tuple)
	Contains the coordinates of the current point

	Returns
	---
	List of tuples containing the neighbourhood points

	Example Call
	---
	neighbours = neighbour_determine(current)

	"""
	neighbours = []
	i = current[0]
	j = current[1]
	if i in range(1,8):
		if j in range(1,11):
			neighbours = [(i,j-1),(i,j+1),(i+1,j),(i-1,j)]
		
		elif j == 11:
			neighbours = [(i,j-1),(i+1,j),(i-1,j)]
		
		elif j == 0:
			neighbours = [(i,j+1),(i+1,j),(i-1,j)]
	
	elif i == 0:
		if j in range(1,11):
			neighbours = [(i,j-1),(i,j+1),(i+1,j)]
		
		elif j == 11:
			neighbours = [(i+1,j),(i,j-1)]
		
		elif j == 0:
			neighbours = [(i,j+1),(i+1,j)]
	
	elif i == 8:
		if j in range(1,11):
			neighbours = [(i,j-1),(i,j+1),(i-1,j)]
		
		elif j == 11:
			neighbours = [(i-1,j),(i,j-1)]
		
		elif j == 0:
			neighbours = [(i-1,j),(i,j+1)]

	return neighbours


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
	sim.simxFinish(-1) # close all opened connections
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
	`return_code` 	:  [ integer ]
		the return code generated from the start running simulation remote API
	
	Example call:
	---
	return_code = start_simulation()
	
	"""
	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	if client_id!=-1:
		return_code = sim.simxStartSimulation(client_id,sim.simx_opmode_blocking)

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
	return_code, v_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	r, v_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, v_handle, 0, sim.simx_opmode_blocking)
	
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
	img = np.array(vision_sensor_image, dtype=np.uint8)	#Convert vision_sensor_image list to array
	img_1 = np.reshape(img, [image_resolution[1],image_resolution[0],3])	#Convert to 3D array
	img_2 = img_1[...,::-1] #BGR to RGB
	transformed_image = np.flip(img_2, 0)	#Flip about x-axis


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
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)

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


def detect_qr_codes(transformed_image):
	
	"""
	Purpose:
	---
	This function receives the transformed image from the vision sensor and detects qr codes in the image

	Input Arguments:
	---
	`transformed_image` 	:  [ numpy array ]
		the transformed image array
	
	Returns:
	---
	None
	
	Example call:
	---
	detect_qr_codes()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	q_code = decode(transformed_image)

	qr_code =[]
	x=0
	y=0

	for obj in q_code:
		q_data = obj.data.decode("utf-8")
		qr_code = q_data
		points = obj.polygon
		(x,y,w,h) = obj.rect
		q_coordinate = (x,y)
	
		
	
	##################################################
	
	return qr_code,x,y


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

	##############	ADD YOUR CODE HERE	##############
	
	h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float)		#Matrix to transform joint velocities to translational velocity of the robot
	v = np.matrix([[rot_vel],[forw_back_vel],[-1*left_right_vel]], float)			#Veloctiy in vector form
	j = np.matmul(h,v)										
	j = 20*j	#Matrix of desired joint velocities
	
	j1 = j[0,0]
	j2 = j[1,0]
	j3 = j[2,0]
	j4 = j[3,0]

	
	returnCode = sim.simxSetJointTargetVelocity(client_id, wheel_joints[0], j1, sim.simx_opmode_oneshot)	#Actuation of joints with the required velocities
	returnCode = sim.simxSetJointTargetVelocity(client_id, wheel_joints[1], j2, sim.simx_opmode_oneshot)
	returnCode = sim.simxSetJointTargetVelocity(client_id, wheel_joints[2], j3, sim.simx_opmode_oneshot)
	returnCode = sim.simxSetJointTargetVelocity(client_id, wheel_joints[3], j4, sim.simx_opmode_oneshot)
	
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

	##############	ADD YOUR CODE HERE	##############
	return_code, j1_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_blocking) #Obtaining joint values for each wheel of the robot
	return_code, j2_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
	return_code, j3_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)
	return_code, j4_handle = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
	wheel_joints = [j1_handle, j2_handle, j3_handle, j4_handle] 											 #Storing the joint handles in a list
	
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

	return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")

	for index,joint_val in enumerate(joints_position):
		joints_position[index]=float(joint_val)

	return joints_position


def rot_by_angle(angle, rot, client_id):

	"""
	Purpose
	---
	This function rotates the bot by the angle passed to it 

	Input Arguments
	---
	`angle`			: [float]
		The angle by which the bot is to be rotated, in radians
	
	`rot`			: [list of floats]
		The list containing the angular displacements of wheels from the start of simulation
	
	`client_id`		: [integer]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns
	---
	a list of float values, prev_req_rot

	Example call
	---
	prev_req_rot = rot_by_angle(-1.57,prev_req_rot,client_id)
	
	"""

	vx = 0			#Desired 'x' component of the velocity
	vy = 0
	if angle == 0:
		return rot			#Desired 'y' component of the velocity

	w = abs(angle)*0.1/angle						#Angular velocity

	tot_time = abs(angle)/0.1						#Total time required to traverse between points

	req_rot = [0,0,0,0]
	act_rot = [0,0,0,0]

	i_del_v = np.matrix([[0],[0],[0]], float)		#Variable to store integral of the error 
	del_v_prev = np.matrix([[0],[0],[0]], float)

	h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float)	#Transformation matrix based on the kinenatics of the robot with mecanum wheels
	v = np.matrix([[w],[vy],[-1*vx]], float)		#Velocity in matrix form
	j = np.matmul(h,v)								#Convertion of velocity to joint input values
	
	dt = 0.05
	t = 0
	req_rot_m = tot_time*20*j		#Required rotation of joints to traverse between the set of	given 2 consecutive current and target points			
	wheel_joints = init_setup(client_id)

	for h in range(len(req_rot)):
		req_rot[h] = rot[h] + req_rot_m[h,0] 	#Conversion of matrix to list

	error_rot = []		#List to store the error in rotation of joints

	cur_rot = encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
	cur_rot_2 = cur_rot.copy()		#Duplication of the encoder data 

	cur_rot[1] = cur_rot_2[3]		#Swapping of values in the list to match the indexing convention of joints in our code and encoder data
	cur_rot[3] = cur_rot_2[1]
		
	for g in range(len(cur_rot)):
		diff = req_rot[g] - cur_rot[g] 	#Calculation of error in current time step
		error_rot.append(diff)
	
	error_rot_abs = [abs(ele) for ele in error_rot]
	angle_rot = 0
	while (abs(angle) - angle_rot > 0.045):
		
		set_bot_movement(client_id, wheel_joints, vy, vx, w)
		cur_rot = encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
		cur_rot_2 = cur_rot.copy()		#Duplication of the encoder data 

		cur_rot[1] = cur_rot_2[3]		#Swapping of values in the list to match the indexing convention of joints in our code and encoder data
		cur_rot[3] = cur_rot_2[1]
		
		error_rot = []

		act_rot_m = t*20*j		#Actual required rotation of joints till the current iteration as matrix

		angle_rot = abs(w)*t	

		for g in range(len(req_rot)):
			act_rot[g] = act_rot_m[g,0] + rot[g]	#Convertion of matrix to list
			
		error_rot = []		#List to store the error in rotation of joints
			
		for g in range(len(cur_rot)):
			diff = act_rot[g] - cur_rot[g] 	#Calculation of error in current time step
			error_rot.append(diff)
			

		if t==0:
			set_bot_movement(client_id, wheel_joints, vy, vx , w ) #Moving the bot with corrected velocity
			t = t+dt


		else:
			del_j = np.matrix([[error_rot[0]], [error_rot[1]], [error_rot[2]], [error_rot[3]]], float)	#Convertion of list to matrix
			del_j = del_j/dt	#Instantaneous error in angular velocities of the joints

				
			h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float) #Matrix to transform joint velocities to translational velocity of the robot
			del_v = 0.05*np.matmul(np.linalg.pinv(h), del_j)	#Error joint velocities converted to error in robot velocity
			
			i_del_v = i_del_v + 0.04*del_v			#Integral error term
			d_del_v = (del_v - del_v_prev)/dt		#Differential error Term
			del_v_prev = del_v						#Error in previous time step
		
			p_del_v = 0.15*del_v		#Proportional correction term
			i_del_v = 0.1*i_del_v			#Integral correction term
			d_del_v = 0*d_del_v			#Differential correction term - Was not used - Gain set to zero

			c_del_v = p_del_v + i_del_v + d_del_v 	#Final correction term
			
			set_bot_movement(client_id, wheel_joints, vy + c_del_v[1,0], vx - c_del_v[2,0], w + c_del_v[0,0])

		error_rot_l = []

		t = t+dt 	#incrementing to the next time step

				
			
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	return req_rot	


def nav_logic(prev_req_rot, target_points,v_mag, ornt, client_id):
	"""
	Purpose:
	---
	This function should implement your navigation logic. 

	Input Arguments
	---
	`prev_req_rot`		: [list of floats]
		Angular Displacement of wheels

	`target_points`		: [list of tuples]
		List of points to be traversed
	
	`v_mag`				: [float]
		Bot traversal velocity
	
	`ornt`				: [integer]
		Bot orientation
	
	`client_id`			: [integer]
		the client id of the communication thread returned by init_remote_api_server()

	Returns
	---
	a list of float values, prev_req_rot

	Example Call
	---
	prev_req_rot = nav_logic(prev_req_rot, route_map, 90, client_id)
	"""
	wheel_joints = init_setup(client_id)


	req_rot = [0,0,0,0]			#Total required rotation of joints to traverse from current point to the next target point
	act_rot = [0,0,0,0]			#Actual required rotation of joints till any time 't'
	t = 0						#Current time
	dt = 0.05
	
	
	wheel_joints = init_setup(client_id)
	
	t_last = 0										#Temprorary variable used to store the time at which previous target point was reached
	
	i_del_v = np.matrix([[0],[0],[0]], float)		#Variable to store integral of the error 
	del_v_prev = np.matrix([[0],[0],[0]], float)	#Variable to store error in previous time step


	dist_cov = 0									#Variable to store the distance covered till the current iteration

	for i in range(len(target_points) - 1):

		
		current = target_points[i]			#Current point at which bot is present
		target = target_points[i+1]			#Next target point to be traversed


		del_y = target[1]-current[1]		#Difference between 'y' coordinates of the points
		del_x = target[0]-current[0]		#Difference between 'x' coordinates of the points
		theta = math.atan2(del_y, del_x)	#Angle made by a vector pointing from current point to the target point

		
		vx = v_mag*math.cos(theta)			#Desired 'x' component of the velocity
		vy = v_mag*math.sin(theta)			#Desired 'y' component of the velocity
		w = 0								#Angular velocity
		
	

		dist = 0.535*math.sqrt(del_y*del_y + del_x*del_x)	#Distance between points in the scene file
		tot_time = dist/v_mag								#Total time required to traverse between points

		h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float)	#Transformation matrix based on the kinenatics of the robot with mecanum wheels
		
		if ornt == 0:
			v = np.matrix([[w],[vy],[-1*vx]], float) #Moving the bot with corrected velocity
		if ornt == 90:
			v = np.matrix([[w],[vx],[vy]], float)
		if ornt == 180:
			v = np.matrix([[w],[-1*vy],[vx]], float)
		if ornt == 270:
			v = np.matrix([[w],[-1*vx],[-1*vy]], float)
		j = np.matmul(h,v)								#Convertion of velocity to joint input values
		
		req_rot_m = tot_time*20*j		#Required rotation of joints to traverse between the set of	given 2 consecutive current and target points			


		

		for h in range(len(req_rot)):
			req_rot[h] = prev_req_rot[h] + req_rot_m[h,0] 	#Conversion of matrix to list

		
	
	#Reading and processing image from the vision sensor:	
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
		qr_data = detect_qr_codes(transformed_image)
		qr_data = list(qr_data)
		
		dist_cov = 0 		#Initializing distance covered to zero before start of the iteration
	

		while True:
			
			cur_rot = encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
			cur_rot_2 = cur_rot.copy()		#Duplication of the encoder data 



			cur_rot[1] = cur_rot_2[3]		#Swapping of values in the list to match the indexing convention of joints in our code and encoder data
			cur_rot[3] = cur_rot_2[1]


			dist_cov = v_mag*(t-t_last)		#Distance covered till the current iteration
		
			act_rot_m = (t-t_last)*20*j		#Actual required rotation of joints till the current iteration as matrix
			for g in range(len(req_rot)):
				act_rot[g] = act_rot_m[g,0] + prev_req_rot[g]	#Convertion of matrix to list
			
			error_rot = []		#List to store the error in rotation of joints
			
			for g in range(len(cur_rot)):
				diff = act_rot[g] - cur_rot[g] 	#Calculation of error in current time step
				error_rot.append(diff)
			

			if t==0:
				if ornt == 0:
					set_bot_movement(client_id, wheel_joints, vy, vx , w ) #Moving the bot with corrected velocity
				if ornt == 90:
					set_bot_movement(client_id, wheel_joints, (vx), -1*(vy), 0)
				if ornt == 180:
					set_bot_movement(client_id, wheel_joints, -1*(vy), -1*(vx), w)
				if ornt == 270:
					set_bot_movement(client_id, wheel_joints, -1*(vx), (vy), w)	#No correction term to be added to velocities at t=0
				t = t+dt


			else:
				del_j = np.matrix([[error_rot[0]], [error_rot[1]], [error_rot[2]], [error_rot[3]]], float)	#Convertion of list to matrix
				del_j = del_j/dt	#Instantaneous error in angular velocities of the joints

				
				h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float) #Matrix to transform joint velocities to translational velocity of the robot
				del_v = 0.05*np.matmul(np.linalg.pinv(h), del_j)	#Error joint velocities converted to error in robot velocity
			
				i_del_v = i_del_v + 0.04*del_v			#Integral error term
				d_del_v = (del_v - del_v_prev)/dt		#Differential error Term
				del_v_prev = del_v						#Error in previous time step

				

				p_del_v = 0.16*del_v		#Proportional correction term
				i_del_v = i_del_v			#Integral correction term
				d_del_v = 0.004*d_del_v			#Differential correction term - Was not used - Gain set to zero

				c_del_v = p_del_v + i_del_v + d_del_v 	#Final correction term
			

				if ornt == 0:
					set_bot_movement(client_id, wheel_joints, vy + c_del_v[1,0], vx - c_del_v[2,0], w + c_del_v[0,0]) #Moving the bot with corrected velocity
				elif ornt == 90:
					set_bot_movement(client_id, wheel_joints, (vx) + c_del_v[1,0], -1*(vy) - c_del_v[2,0],  w + c_del_v[0,0])
				elif ornt == 180:
					set_bot_movement(client_id, wheel_joints, -1*(vy) + c_del_v[1,0], -1*(vx) - c_del_v[2,0], w + c_del_v[0,0])
				elif ornt == 270:
					set_bot_movement(client_id, wheel_joints, -1*(vx) + c_del_v[1,0], (vy) - c_del_v[2,0],  w + c_del_v[0,0])
				
				
				vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)		#Reading and processing the image from the vision sensor
				transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
				qr_data,x,y = detect_qr_codes(transformed_image)		#QR Code data obtained
				qr_data = list(qr_data)
				qr_data_1 =[]
				
				
				if qr_data:			#condition to check if some data was retrieved from the image
					qr_data_1 = [int(qr_data[1]), int(qr_data[4])]		#Retrieving coordinated from QR code data
					if not qr_data[5] == ')':
						qr_data_1 = [int(qr_data[1]), 10*int(qr_data[4])+int(qr_data[5])]	

				if qr_data_1 == [(target[0]), (target[1])]: 	#condition to check if the coordinates retrieved from the QR code is the required target coordinates
					set_bot_movement(client_id, wheel_joints, 0, 0, 0)
					t_last = t						#Updating the time at which robot reaches the required target point
					cur_rot = encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
					cur_rot_2 = cur_rot.copy()		#Duplication of the encoder data 
					cur_rot[1] = cur_rot_2[3]		#Swapping of values in the list to match the indexing convention of joints in our code and encoder data
					cur_rot[3] = cur_rot_2[1] 		#Copying the required rotation variable to update in the next iteration
					prev_req_rot = cur_rot.copy()
					set_bot_movement(client_id, wheel_joints, 0, 0, 0)
					break							#Break out of the loop since the target has been reached

			
				t = t+dt 	#incrementing to the next time step

				
		i_del_v = 0*i_del_v		#Integral term reset to zero - Flushing out the integral term after reaching every target point
		set_bot_movement(client_id, wheel_joints, 0, 0, 0)	#Bot velocity set to zero to start again

		cur_rot = encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
		cur_rot_2 = cur_rot.copy()		#Duplication of the encoder data 
		cur_rot[1] = cur_rot_2[3]		#Swapping of values in the list to match the indexing convention of joints in our code and encoder data
		cur_rot[3] = cur_rot_2[1] 		#Copying the required rotation variable to update in the next iteration
		prev_req_rot = cur_rot.copy()
		set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	return req_rot
		
		
					
		

def shortest_path(current,target,pathway):
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.

	Input Argument
	---
	`current`:			[tuple]
		A tuple containing the coordinates of current point
	
	`target`:			[tuple]
		Tuple containing the coordinate of point to be reached
	
	`pathway`:			[list of tuples]
		Pre determined list of permissible points that can be traversed
	
	Returns
	---
	route_map, a list of tuples that go from current point to target point

	Example Call
	---
	route_map = shortest_path(current,target,pathway)

	"""
	route_map = []
	route_map.append(current)

	while current != target:
		minim = math.inf
		min_point = current 
		neighbours = neighbour_determine(current)
		for point in neighbours:
			if point in pathway and point not in route_map:
				if ((point[0]-target[0])**2 + (point[1]-target[1])**2)**0.5 < minim:
					min_point = point
					minim = ((point[0]-target[0])**2 + (point[1]-target[1])**2)**0.5

		route_map.append(min_point)
		current = min_point

	return route_map


		
def task_3_primary(client_id, current, target_points, obstacles):
	
	
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
	
	
	route_map = []
	for target in target_points:
		route_map += shortest_path(current,target,obstacles)
		current = target
	
	return_value = nav_logic(route_map,client_id)


if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(2,3),(3,7)]    # You can give any number of different co-ordinates


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