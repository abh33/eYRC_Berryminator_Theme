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


current_coordinate       = [ 0, 1]    # This will store the latest co-ordinate detected from qrcodes

correction_qr_up_down    = 0           # These three variables will correct the drift horizontaly, vertically and in heading0
correction_qr_left_right = 0 
correction_qr_rot        = 0

bot_up_down              = 0           # These three variables are the input velocities to the bot to traverse, based on one's logic
bot_left_right           = 0
bot_rot                  = 0

#dijkstra for shortest path
path = []
costs = []

################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


# This function will hold the program till the target co-ordinate is reached
# While in hold, it will detect qr code if any and reset the velocities 
def wait_to_reach(client_id, wheel_joints,target_coordinate):

	while current_coordinate != list(target_coordinate):
		# print("Current co-ordinatee: ", current_coordinate)
		coordinate_detection(client_id)
		set_bot_movement(client_id, wheel_joints, bot_up_down, bot_left_right, bot_rot)



# This function takes a target co-ordinate and accordingly actuates the bot
def set_bot_for_coordinate(rec_client_id,wheel_joints, target_coordinate):
	global bot_left_right, bot_up_down, bot_rot

	client_id=rec_client_id

	coordinate_detection(client_id)

	if target_coordinate[0] == current_coordinate[0]:          # X coor same
		if target_coordinate[1] > current_coordinate[1]:       # Is target Y > current Y
			# print("Straight")
			bot_up_down    = -4                                # Going straight
			bot_left_right = 0
			bot_rot        = 0
			wait_to_reach(client_id, wheel_joints, target_coordinate)

			# print("Straight 2")
			bot_up_down    = 0
			bot_left_right = 0
			bot_rot        = 0
			set_bot_movement(client_id,wheel_joints, bot_up_down, bot_left_right, bot_rot)
			# print("Reached point: ", target_coordinate)

		else:
			# print("DOWN")
			bot_up_down    = 2                                 # Going down 
			bot_left_right = 0
			bot_rot        = 0
			wait_to_reach(client_id, wheel_joints, target_coordinate)

			bot_up_down    = 0
			bot_left_right = 0
			bot_rot        = 0
			set_bot_movement(client_id,wheel_joints, bot_up_down, bot_left_right, bot_rot)
			# print("Reached point: ", target_coordinate)


	elif target_coordinate[1] == current_coordinate[1]:        # Y co-ordinate same
		if target_coordinate[0] > current_coordinate[0]:       # Is target X greater
			
			bot_up_down    = 0
			bot_left_right = -4                                # Going right
			bot_rot        = 0
			wait_to_reach(client_id, wheel_joints, target_coordinate)

			bot_up_down    = 0
			bot_left_right = 0
			bot_rot        = 0
			set_bot_movement(client_id,wheel_joints, bot_up_down, bot_left_right, bot_rot)
			# print("Reached point: ", target_coordinate)

		else:
			bot_up_down    = 0
			bot_left_right = 4                                  # Going left
			bot_rot        = 0
			wait_to_reach(client_id, wheel_joints, target_coordinate)

			bot_up_down    = 0
			bot_left_right = 0
			bot_rot        = 0
			set_bot_movement(client_id,wheel_joints, bot_up_down, bot_left_right, bot_rot)
			# print("Reached point: ", target_coordinate)



def coordinate_detection(client_id):

	global current_coordinate

	#############################
	# Get image array and its resolution from Vision Sensor in ComppeliaSim scene
	try:
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
		# print("Resolution: ", image_resolution)

		if ((return_code == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(vision_sensor_image) > 0)):
			# print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

			# Get the transformed vision sensor image captured in correct format
			try:
				transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)

				if (type(transformed_image) is np.ndarray):

					qr_codes_list = detect_qr_codes(transformed_image)
					if len(qr_codes_list) > 0:
						# print(qr_codes_list[0])
						# print(qr_codes_list[0][0])
						# print(type(qr_codes_list))
						# qr_codes_list = ["'"] + qr_codes_list + ["'"]
						qr_codes_list = str(qr_codes_list[0][0])
						current_coordinate = eval(qr_codes_list) #list(qr_codes_list[0][0])
						# print("X co-ordinate: ", current_coordinate[0])
						# print("Y co-ordinate: ", current_coordinate[1])

						current_coordinate = [current_coordinate[0], current_coordinate[1]]

					# cv2.imshow('transformed image', transformed_image)
					# if cv2.waitKey(1) & 0xFF == ord('q'):
					# 	break
					

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
	#######################


def cost_fn(p1,p2):
	cost = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)
	return cost

#building the graph dict with key as coords and value as neighbours
def build_graph(matrix):
	graph = {}
	
	for coord in matrix:
		possible = [(coord[0]-1,coord[1]),(coord[0]+1,coord[1]),(coord[0],coord[1]-1),(coord[0],coord[1]+1)]
		neighbours = []
		for neigh in possible:
			if neigh in matrix:
				neighbours.append(neigh)
		graph[coord] = neighbours
	return graph

def generate_edges(graph):
	edges=[]
	for key,list_1 in graph.items():
		for value in list_1:
			edges.append((key,value))
	
	return edges


def dijkstra(graph, start, end):
	global path, costs
	costs = []
	path = path + [start]
	if start == end:
		return path
	neighs = graph[start]
	for i in neighs:
		cost = cost_fn(i,end)
		costs.append(cost)
	ind = costs.index(min(costs))
	goto = neighs[ind]
	if goto not in path:
		newpath = dijkstra(graph, goto, end)
	return newpath       

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

	global correction_qr_up_down, correction_qr_left_right, correction_qr_rot

	qr_codes = []

	detected = decode(transformed_image)

	for detected_qr in detected:
		message = (detected_qr.data).decode()
		p1, p2, p3, p4 = detected_qr.polygon
		centre = (((p1.x + p2.x + p3.x + p4.x)/4), ((p1.y + p2.y + p3.y + p4.y)/4))
		qr_codes.append([message, centre])
		# print()
		# print(detected_qr.polygon)
		# print("\nQR code: ", message)
		# print(centre)
		num = (p4.y - p1.y)
		den = (p4.x - p1.x)
		angle = math.degrees(math.atan2(num, den))
		# print("ANGLE:", angle)

		P1 = 0.01
		if bot_up_down == 0 and bot_left_right !=0:
			if centre[1] < 120:
				# Correction: Go up
				correction_qr_up_down = P1 * (centre[1] - 127)       # - 0.2
			elif centre[1] > 135:
				correction_qr_up_down = P1 * (centre[1] - 127)       #  0.2
			else:
				correction_qr_left_right = 0
				correction_qr_up_down = 0
		elif bot_up_down !=0 and bot_left_right == 0:
			if centre[0] < 120:
				# Correction: Go up
				correction_qr_left_right = P1 * (127 - centre[0])    # 0.2
			elif centre[1] > 135:
				correction_qr_left_right = P1 * (127 - centre[0])    # - 0.2
			else:
				correction_qr_left_right = 0
				correction_qr_up_down = 0

		# print("UP(-) / DOWN(+): ", correction_qr_up_down)
		# print("LEFT(+) / RIGHT(-): ", correction_qr_left_right)

		angle_error = angle #-90 - angle
		# print("ANGLE error: ", angle_error)
		P2 = 0.0005
		# correction_qr_rot = - P2 * angle_error
		# print("Correction ROT: ", correction_qr_rot)
		# if abs(angle_error) > 0.5:
		# 	time.sleep(1.)
	
	return qr_codes


def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):

	"""
	Purpose:
	---
	This function receives the desired forward/back, left/right, rotational velocites of the bot.
	It will translate these desired velocities into individual joint velocities(4 joints) and actuate the joints
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



	forw_back_vel  = forw_back_vel  + correction_qr_up_down
	left_right_vel = left_right_vel + correction_qr_left_right 
	rot_vel        = rot_vel        + correction_qr_rot

	# print()
	# print(forw_back_vel)
	# print(left_right_vel)
	# print(rot_vel)

	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],-forw_back_vel-left_right_vel-rot_vel,sim.simx_opmode_oneshot)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],-forw_back_vel+left_right_vel-rot_vel,sim.simx_opmode_oneshot)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],-forw_back_vel-left_right_vel+rot_vel,sim.simx_opmode_oneshot)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],-forw_back_vel+left_right_vel+rot_vel,sim.simx_opmode_oneshot)



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

	#Since client_id is defined in task_2a.py file, it needs to be assigned here as well.

	##############	ADD YOUR CODE HERE	##############
	wheel_joints=[-1,-1,-1,-1] 									# front left, rear left, rear right, front right
	return_code,wheel_joints[0]=sim.simxGetObjectHandle(client_id,'rollingJoint_fl',sim.simx_opmode_blocking)
	return_code,wheel_joints[1]=sim.simxGetObjectHandle(client_id,'rollingJoint_rl',sim.simx_opmode_blocking)
	return_code,wheel_joints[2]=sim.simxGetObjectHandle(client_id,'rollingJoint_rr',sim.simx_opmode_blocking)
	return_code,wheel_joints[3]=sim.simxGetObjectHandle(client_id,'rollingJoint_fr',sim.simx_opmode_blocking)

	# return_code, BM_Bot_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot', sim.simx_opmode_blocking)
	# return_code = sim.SetObjectPosition( client_id, BM_Bot_handle, -1 , [ 4, 4, 9.5750e-02], sim.simx_opmode_blocking)


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

	return joints_position


def nav_logic():
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""


def shortest_path(edges, starting_node, goal):
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
	"""
	#############################################
	visited = []
	queue = [[starting_node]]
	
	while queue:
		path = queue.pop(0)
		node = path[-1]
		if node not in visited:
			neighbours = []
			#for edge in graph.edges:
			for edge in edges:
				#print(edge, graph.edges)
				if edge[0] == node:
					neighbours.append(edge[1])
				elif edge[1] == node:
					neighbours.append(edge[0])
			for neighbour in neighbours:
				new_path = list(path)
				new_path.append(neighbour)
				queue.append(new_path)
				
				if neighbour == goal:
					return new_path
			
			visited.append(node)
			
	return []
	#############################################

def init_traversal(client_id):
	global wheel_joints,full,obstacles_coords,graph,edges_of_graph

	# Get wheel joint handles
	wheel_joints = init_setup(client_id)
	set_bot_movement(client_id,wheel_joints, 0, 0, 0)

	obstacles_coords=[(4,0)] # Append to this list. This will updated only at the beginning of the run.

	paths = []

	#full list contining all coordinates
	#full is the list containing all coords as tuples
	matrix = []
	col = []
	for i in range(9):
		for j in range(12):
			node=(i,j)
			if node not in obstacles_coords:
				matrix.append((i,j))
	# 		col.append((i,j))
	# 	matrix.append(col)
	# 	col = []
	# full = []
	# for i in matrix:
	# 	for j in i:
	# 		full.append(j)

	graph = build_graph(matrix)	
	edges_of_graph=generate_edges(graph)


def traverse_bot(client_id,start_coord,end_coord):
	global path, costs, correction_qr_left_right, correction_qr_up_down, correction_qr_rot,wheel_joints,edges_of_graph

	######################################

	set_bot_movement(client_id,wheel_joints, 0, 0, 0)

	# print('=====================================')
	# print(start,end,graph)
	# print('=====================================')			
	#p     = dijkstra(graph,start,end)
	path=shortest_path(edges_of_graph, start_coord, end_coord)
	# paths.append(p)
	# path = []
	# costs = []

	print('=====================================')
	print("Path Calculated= ",path)
	print('=====================================')

	for i in range(len(path)):
		end_point = path[i]
		# print(end_point)
		set_bot_for_coordinate( client_id, wheel_joints, end_point)

	
	correction_qr_left_right = 0
	correction_qr_up_down    = 0
	correction_qr_rot        = 0
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)

	######################################


def task_3_primary( client_id, target_points):
	
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot traverses all the target navigational co-ordinates and then only
	the function ends.

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
	
	global path, costs, correction_qr_left_right, correction_qr_up_down, correction_qr_rot

	# Get wheel joint handles
	wheel_joints = init_setup(client_id)
	set_bot_movement(client_id,wheel_joints, 0, 0, 0)

	#full list contining all coordinates
	#full is the list containing all coords as tuples
	matrix = []
	col = []
	for i in range(9):
		for j in range(12):
			col.append((i,j)) 
		matrix.append(col)
		col = []
	full = []
	for i in matrix:
		for j in i:
			full.append(j)

	start = (0,0)                     # start point
	target_points.insert( 0, (0,0))
	points_to_traverse = target_points

	paths = []
	for i in range(len(points_to_traverse)):
		if i < (len(points_to_traverse)-1):
			start = points_to_traverse[i]
			end   = points_to_traverse[i+1]				  # target co-ordinate
			graph = build_graph(full)
			print('=====================================')
			print(start,end,graph)
			print('=====================================')			
			p     = dijkstra(graph,start,end)
			paths.append(p)
			path = []
			costs = []

	for p in paths:
		print(p)
		for i in range(len(p)):
			end_point = p[i]
			# print(end_point)
			set_bot_for_coordinate( client_id, wheel_joints, end_point)

	
	correction_qr_left_right = 0
	correction_qr_up_down    = 0
	correction_qr_rot        = 0
	set_bot_movement(client_id, wheel_joints, 0, 0, 0)
	


if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(4,4)]    # You can give any number of different co-ordinates


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
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()