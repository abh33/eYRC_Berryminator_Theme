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


# Team ID:			[ BM_1809]
# Author List:		[ Shivanka, Muditha, Treshan, Ashen ]
# Filename:			task_3.py
# Functions:		
# Global variables:	
# 					[targets, ci, Kp, Ki, Kd, error_sum, last_error, scl, point, iniPos, wheel_joints, enco, curr_enco, y ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

#from tkinter import Y
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from numpy.core.getlimits import _KNOWN_TYPES
from numpy.lib.function_base import average
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


# function for calculating PID error values
def PID(encoders_c, enco,  error_sum, last_error, state):

	# create zero starting encoder values after every completed point traversed
	encoders = [abs(encoders_c[i] - enco[i]) for i in range(len(enco))]

	# check if the bot drives forward or backward and get the error
	if state == 1:		# forward
		error = max((encoders[0] - encoders[2]), (encoders[3] - encoders[1]))
	else:				# backward
		error = min((encoders[0] - encoders[2]), (encoders[3] - encoders[1]))

	# PID error calculation
	P = Kp*error
	I = (error_sum+error)*Ki
	D = (error-last_error)*Kd

	correction = P+I+D

	error_sum += error
	last_error = error
	return correction*state, error_sum

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

	sim.simxFinish(client_id) # just in case, close all opened connections
	client_id=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

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

	return_code, sensor = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_oneshot_wait)
	opt = 0
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, sensor, opt, sim.simx_opmode_streaming)
	while (sim.simxGetConnectionId(client_id) != -1):
		return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, sensor, 0, sim.simx_opmode_buffer)
		if vision_sensor_image!=0 and len(image_resolution)==2:
			break

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

	img = np.array(vision_sensor_image,dtype=np.uint8)
	img.resize([image_resolution[1],image_resolution[0],3])
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	transformed_image = img[::-1]

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
	
	qr_codes = []

	for barcode in decode(transformed_image):
		qr = []
		x_co,y_co = 0,0
		data = barcode.data.decode('utf-8')
		qr.append(data)
		for point in barcode.polygon:
			x_co += point[0]
			y_co += point[1]
		qr.append(tuple((x_co/4, y_co/4)))
		qr_codes.append(qr)
	

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

	##############	ADD YOUR CODE HERE	##############
	
	# if forw_back_vel == 0 and left_right_vel == 0:
	# 	fl_v = fr_v = rl_v = rr_v = 0
	# else:

	a = 2.3753+1.5852
	cof = np.array([[1, 1, -a], [1, -1, a], [1, -1, -a], [1, 1, a]])
	vel = np.array([forw_back_vel, left_right_vel, rot_vel])
	[fl_v, fr_v, rl_v, rr_v] = np.matmul(cof, vel)

	velocity = [fl_v, fr_v, rl_v, rr_v]
	
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[0], velocity[0], sim.simx_opmode_oneshot)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[1], velocity[1], sim.simx_opmode_oneshot)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[2], velocity[2], sim.simx_opmode_oneshot)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[3], velocity[3], sim.simx_opmode_oneshot)

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

	return_code, fl_joint = sim.simxGetObjectHandle(client_id, 'rollingJoint_fl', sim.simx_opmode_blocking)
	return_code, fr_joint = sim.simxGetObjectHandle(client_id, 'rollingJoint_fr', sim.simx_opmode_blocking)
	return_code, rl_joint = sim.simxGetObjectHandle(client_id, 'rollingJoint_rl', sim.simx_opmode_blocking)
	return_code, rr_joint = sim.simxGetObjectHandle(client_id, 'rollingJoint_rr', sim.simx_opmode_blocking)

	wheel_joints = [fl_joint, fr_joint, rl_joint, rr_joint]

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


def nav_logic():
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""

	#####################################################
	
	global curr_enco
	global error_sum
	curr_enco = encoders(ci)	# encoder values array
	
	# x and y distances for the next point from the current point 
	row = y[point][0] - iniPos[0]
	col = y[point][1] - iniPos[1]
	#v = 10	# velocity
	angle = math.atan2(row, col)	# angle for the next point from the current point

	# check if the vehicle drives forward or backward
	state = 1
	if col<0:
		state = -1	
	correction, error_sum = PID(curr_enco, enco, error_sum, last_error, state)	# call the PID function

	Vt = scl * correction		# calculate rotational velocity

	# calculate forw_back_vel and left_right_vel
	forw_back_vel = v*np.cos(angle)
	left_right_vel = v *np.sin(angle)
	set_bot_movement(ci, wheel_joints, forw_back_vel, left_right_vel , Vt)	# call the function
	
	#####################################################


def shortest_path():
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
	"""

	#####################################################

	# global y
	# global iniPos

	# if rel_origin == 'anti_clk_quarter':
	# 	iniPos = (iniPos[1], 8 - iniPos[0])
	# 	y = [(point[1], 8 - point[0]) for point in targets]
	# elif rel_origin == 'clk_quarter':
	# 	iniPos = (11 - iniPos[1], iniPos[0])
	# 	y = [(11 - point[1], point[0]) for point in targets]
	# elif rel_origin == 'clk_semi':
	# 	iniPos = (8 - iniPos[0], 11 - iniPos[1])
	# 	y = [(8 - point[0], 11 - point[1]) for point in targets]
	# elif rel_origin == 'normal':
	# 	y = targets



	#####################################################


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
	######################################################

	# global variables
	global targets
	global ci

	global Kp
	global Ki
	global Kd
	global error_sum
	global last_error
	global scl
	global point
	global v
	global reach_factor

	global iniPos
	global rel_origin
	global wheel_joints
	global enco
	global curr_enco

	enco = encoders(client_id) # [0, 0, 0, 0]		# initial encoder values

	max_speed = 10
	enc_per_qr_dic = {'lr':9, 'fb':8, 'di':20}

	# target points and client id value
	targets = target_points[2]
	global y
	y = target_points[1]
	iniPos = target_points[0][0]		# initial position
	#rel_origin = target_points[0][2]
	ci = client_id

	# PID constants and error values
	Kp = 4
	Ki = 0
	Kd = 0.7
	scl = 0.001
	error_sum = 0
	last_error = 0
	
	#shortest_path()		# call the function
	wheel_joints = init_setup(ci)		# call the function


	# traveresing through points
	point = 0
	reach_factor = 1
	while point < len(y):
		
		next_point = y[point]
		left_right_state = bool(abs(iniPos[0] - next_point[0]))
		forward_back_state = bool(abs(iniPos[1] - next_point[1]))
		# print(iniPos, " -----" , next_point)
		if left_right_state == 1 and forward_back_state == 0:
			dir = 'lr'
			wheel = 0
		elif left_right_state == 0 and forward_back_state == 1:
			dir = 'fb'
			wheel = 0
		elif left_right_state == 1 and forward_back_state == 1:
			dir = 'di'
			if (iniPos[0] < next_point[0] and iniPos[1] > next_point[1]) or (iniPos[0] > next_point[0] and iniPos[1] < next_point[1]):
				wheel = 1
			else:
				wheel = 0
		else:
			break

		distance = max(abs(iniPos[0] - next_point[0]), abs(iniPos[1] - next_point[1]))
		enco_per_qr = enc_per_qr_dic[dir]
		curr_enco = encoders(ci)
		enco_diff = enco[wheel] - curr_enco[wheel]		
		# print(iniPos, next_point, dir, distance)
		
		#print(curr_enco[0], abs(enco_diff))
		if abs(enco_diff) > enco_per_qr * (distance-0.6):
			#print("Bot has reached")
			reach_factor = 1 - (abs(enco_diff) - enco_per_qr * (distance - 0.6))/(enco_per_qr * 0.4)
			if reach_factor<0:
				reach_factor = 0
			#set_bot_movement(ci, wheel_joints, 0, 0, 0)

		# if reach_factor == 1:
		# 	v = 20
		# else:
		# 	v = 8 * reach_factor**2 + 1

		v = 12 * reach_factor ** 2 + 3
		# print(reach_factor, v, enco_diff)
		#v = 2
		nav_logic()		# call the function
		if v == max_speed:
			continue
		
		#print(curr_enco[0],enco[0], enco_diff)

		# Get image array and its resolution from Vision Sensor in CoppeliaSim scene
		try:
			vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)

			if ((return_code == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(vision_sensor_image) > 0)):

				# Get the transformed vision sensor image captured in correct format
				try:
					transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
				
					if (type(transformed_image) is np.ndarray):
						qr_codes_list = detect_qr_codes(transformed_image)
						
						# detect qr code in a defined range of vision sensor
						qr_R = 400
						qr_S = (512 - qr_R)/ 2
						if len(qr_codes_list) != 0:	
							# print(qr_codes_list, targets[point])						
							if eval(qr_codes_list[0][0]) == targets[point] and qr_S < qr_codes_list[0][1][0] < qr_S  + qr_R and qr_S < qr_codes_list[0][1][1] < 512:
								#set_bot_movement(client_id, wheel_joints, 0, 0, 0)
								iniPos = y[point]
								reach_factor = 1
								enco = curr_enco
								error_sum = 0
								last_error = 0
								point += 1						

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

	######################################################


if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points =  [(4, 3)]    # You can give any number of different co-ordinates


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