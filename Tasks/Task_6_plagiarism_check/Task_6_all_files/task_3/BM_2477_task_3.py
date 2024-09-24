'''
*
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
*
'''


# Team ID:			[ 2477 ]
# Author List:		[ Swadhesh P U, Sankar Ganesh M, Bala Sakthi R, Harshini T G R ]
# Filename:			task_3.py
# Functions:		[init_remote_api_server, start_simulation, stop_simulation, exit_remote_api_server, get_vision_sensor_image, transform_vision_sensor_image, detect_qr_codes, init_setup,moveforback, moveleftright, movediagonal1, movediagonal2, rotate_left, rotate_right, check_qr, nav_logic, encoders, task_3_primary]
# Global variables:	
# 					[ no_of_left_rot, no_of_right_rot ]


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

	client_id=sim.simxStart('127.0.0.1',19997,True,True,5000,5)

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
	
	return_code=sim.simxStartSimulation(client_id,sim.simx_opmode_oneshot)

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

	return_code, handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_oneshot_wait)
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, handle, 0, sim.simx_opmode_blocking)
	
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

	transformed_image = np.array(vision_sensor_image, dtype = np.uint8)
	transformed_image.resize([image_resolution[0], image_resolution[1], 3])	
	transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)	
	transformed_image = np.flip(transformed_image, 1)

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
	
	img=transformed_image
	barcodes = decode(img)

	for barcode in barcodes:
		
		a, b, w, h = barcode.rect
		x = a + (w / 2)
		y = b + (h / 2)
		
		bdata = barcode.data.decode("utf-8")
		bdata=bdata.replace(" ", "")		
		qr_codes.append([f"{bdata}", (x, y)])
		
	##################################################
	
	return qr_codes


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
	
	return_code,handle_1=sim.simxGetObjectHandle(client_id,'rollingJoint_fl',sim.simx_opmode_blocking)
	return_code,handle_2=sim.simxGetObjectHandle(client_id,'rollingJoint_fr',sim.simx_opmode_blocking)
	return_code,handle_3=sim.simxGetObjectHandle(client_id,'rollingJoint_rl',sim.simx_opmode_blocking)
	return_code,handle_4=sim.simxGetObjectHandle(client_id,'rollingJoint_rr',sim.simx_opmode_blocking)
	wheel_joints=[handle_1,handle_2,handle_3,handle_4]

	##################################################

	return wheel_joints


##############	SET TARGET VELOCITY	##############

wheel_joints=[]
def moveforback(client_id,wheel_joints,for_back_vel):

	##############   To Set Forward Velocity	##############

	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],for_back_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],for_back_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],for_back_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],for_back_vel, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)

	########################################################## 	

def moveleftright(client_id,wheel_joints,left_right_vel):

	##############   To Set Left / Right Velocity	##############

	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],left_right_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],-left_right_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],-left_right_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],left_right_vel, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)

	###############################################################

def movediagonal1(client_id,wheel_joints,diagonal_vel):

	##############   To Set Diagonal Velocity(for top right and bottom left diagonal movement)	##############	
	
	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],diagonal_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],0, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],0, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],diagonal_vel, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)

	##########################################################################################################


def movediagonal2(client_id,wheel_joints,diagonal_vel):

	##############   To Set Diagonal Velocity(for top left and bottom right diagonal movement)	##############	

	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],0, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],diagonal_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],diagonal_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],0, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)

	##########################################################################################################

no_of_left_rot=0
def rotate_left(client_id,wheel_joints,rotate_vel):

	##############   To Set Rotional Velocity (anti-clockwise)	##############	

	global no_of_left_rot
	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],-rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],-rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],rotate_vel, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)
	no_of_left_rot+=1

	##########################################################################

no_of_right_rot=0
def rotate_right(client_id,wheel_joints,rotate_vel):

	##############   To Set Rotional Velocity(clockwise)	##############

	global no_of_right_rot
	sim.simxPauseCommunication(client_id,True)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],-rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],rotate_vel, sim.simx_opmode_streaming)
	return_code=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],-rotate_vel, sim.simx_opmode_streaming)
	sim.simxPauseCommunication(client_id,False)
	no_of_right_rot+=1

	##########################################################################



def check_qr(client_id,wheel_joints,mx,my):

	############### To check whether Bot reached target qr ###############

	Destination_value=(mx,my)
	while(True):
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
		
		if(len(detect_qr_codes(transformed_image))!=0):
			qr_value=detect_qr_codes(transformed_image)[0][0]

			if(len(qr_value)==5):
				qr_value=(int(qr_value[1]),int(qr_value[3]))
			else:
				qr_value=(int(qr_value[1]),int(qr_value[3:5]))

			if(qr_value==Destination_value):
				moveleftright(client_id, wheel_joints,0)
				break

	##########################################################################

def nav_logic(client_id,wheel_joints,target_points):
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""

	towards_left,towards_right = False, False
	if(no_of_left_rot > no_of_right_rot):
		towards_left = True
	elif(no_of_left_rot < no_of_right_rot):
		towards_right = True

	A=0
	while(True):
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)

		if(len(detect_qr_codes(transformed_image))!=0):
			qr_value=detect_qr_codes(transformed_image)[0][0]
			if(len(qr_value)==5):
				qr_value=(int(qr_value[1]),int(qr_value[3]))
			else:
				qr_value=(int(qr_value[1]),int(qr_value[3:5]))
			break
	x,y=qr_value
	for i in target_points:
		(x1,y1)=i
		mx,my=x,y
		while(mx!=x1 and my!=y1):
			if(x1>x and y1>y):
				mx+=1
				my+=1
			elif(x1<x and y1<y):
				mx-=1
				my-=1
			elif(x1>x and y1<y):
				mx+=1
				my-=1
			else:
				mx-=1
				my+=1
		if(abs(x-mx)==abs(y-my)):
			if(mx>x and my>y):
				if(towards_left):
					movediagonal2(client_id,wheel_joints,-4)
				elif(towards_right):
					movediagonal2(client_id,wheel_joints,4)
				else:
					movediagonal1(client_id,wheel_joints,4)

			elif(mx<x and my<y):

				if(towards_left):
					movediagonal2(client_id,wheel_joints,4)
				elif(towards_right):
					movediagonal2(client_id,wheel_joints,-4)
				else:
					movediagonal1(client_id,wheel_joints,-4)
				
			elif(mx<x and my>y):

				if(towards_left):
					movediagonal1(client_id,wheel_joints,4)
				elif(towards_right):
					movediagonal1(client_id,wheel_joints,-4)
				else:
					movediagonal2(client_id,wheel_joints,4)

			elif(mx>x and my<y):
				
				if(towards_left):
					movediagonal1(client_id,wheel_joints,-4)
				elif(towards_right):
					movediagonal1(client_id,wheel_joints,4)
				else:
					movediagonal2(client_id,wheel_joints,-4)

			check_qr(client_id,wheel_joints,mx,my)

		x,y=mx,my	
		if(x1>x):
			if(towards_left):
				moveforback(client_id, wheel_joints,-6)
			elif(towards_right):
				moveforback(client_id, wheel_joints,2.5)
			else:
				moveleftright(client_id, wheel_joints,6)

			check_qr(client_id,wheel_joints,x1,y)

			if(y1>y):
				moveforback(client_id, wheel_joints,2.5)
						
			elif(y1<y):
				moveforback(client_id, wheel_joints,-6)
			
			check_qr(client_id,wheel_joints,x1,y1)
						
			
		elif(x1<x):
			if(towards_left):
				moveforback(client_id, wheel_joints,2.5)
			elif(towards_right):
				moveforback(client_id, wheel_joints,-6)
			else:
				moveleftright(client_id, wheel_joints,-6)

			check_qr(client_id,wheel_joints,x1,y)

					
			if(y1>y):
				moveforback(client_id, wheel_joints,2.5)
						
			elif(y1<y):
				moveforback(client_id, wheel_joints,-6)
			
			check_qr(client_id,wheel_joints,x1,y)
						

		else:
			check_qr(client_id,wheel_joints,x1,y)
			if(y1>y):
				if(towards_left):
					moveleftright(client_id, wheel_joints,5.5)
				elif(towards_right):
					moveleftright(client_id, wheel_joints,-6)
				else:
					moveforback(client_id, wheel_joints,2.5)
					
			elif(y1<y):
				if(towards_left):
					moveleftright(client_id, wheel_joints,-6)
				elif(towards_right):
					moveleftright(client_id, wheel_joints,6)
				else:
					moveforback(client_id, wheel_joints,-6)
			check_qr(client_id,wheel_joints,x1,y1)

		(x,y)=(x1,y1)

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
	wheel_joints=init_setup(client_id)

	vision_sensor_image=[]

	while(len(vision_sensor_image)==0):
		vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)

	nav_logic(client_id,wheel_joints,target_points)

if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(0, 2), (0, 1), (3, 4), (6, 10)]    # You can give any number of different co-ordinates

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