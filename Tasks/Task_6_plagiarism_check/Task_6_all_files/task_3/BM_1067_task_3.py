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


# Team ID:			BM_1067
# Author List:		Mohd Zaid, Umesh, Prathamesh, Raj
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



################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def compensate(client_id,wheel_joints):

	"""
	Purpose:
	---
	This function compensates/improves the position to bot, if qr code 
	cant be detected (out of range) 
	or 
	is used to get the bot 
	perfectly over the qr code to compensate any previous drifts

	Input Arguments:
	---
	speed1 , speed2

	Returns:
	---
	None

	Example call:
	---
	comprehend()

	"""
	#wheel_joints = init_setup(client_id)
	vision_sensor_image, image_resolution,_ = get_vision_sensor_image(client_id)
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	qr_code,cx,cy,point = detect_qr_codes(transformed_image)
	while not qr_code:
		check1 = np.mean(transformed_image[0:256,256:511])
		check2 = np.mean(transformed_image[0:256,0:256])
		check3 = np.mean(transformed_image[256:511,0:256])
		check4 = np.mean(transformed_image[256:511,256:511])
		maxcheck = max(check1,check2,check3,check4)
		if check1>check3 and check1>check4 and check2>check3 and check2>check4:
			speed3 = 1 
			speed4 = 0
		elif check2>check1 and check2>check4 and check3>check1 and check3>check4:
			speed3 = 0 
			speed4 = -1
		elif check1>check3 and check1>check2 and check4>check3 and check4>check2:
			speed3 = 0 
			speed4 = 1
		elif check3>check1 and check3>check2 and check4>check1 and check4>check2:
			speed3 = -1 
			speed4 = 0
		elif check1==maxcheck:
			speed3 = 1
			speed4 = 1
		elif check2==maxcheck:
			speed3 = -1
			speed4 = 1
		elif check3==maxcheck:
			speed3 = -1
			speed4 = -1
		elif check4==maxcheck:
			speed3 = 1
			speed4 = -1
		set_bot_movement(client_id,wheel_joints,speed3*0.5,speed4*0.5,0)
		vision_sensor_image, image_resolution,_ = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
		qr_code,cx,cy,point = detect_qr_codes(transformed_image)
	if point and point[0][1]>point[1][1]:
		temp=point[0]
		point[0]=point[1]
		point[1]=temp
	er = (point[0][0]-point[1][0])
	erp=0
	e1 = (cx-256)
	ep1 = 0
	e2 = (cy-256)
	ep2 = 0
	while (abs(e1)>5 or abs(e2)>5 or er>1 or er<-1) and qr_code:
		vision_sensor_image, image_resolution,_ = get_vision_sensor_image(client_id)
		transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
		qr_code,cx,cy,point = detect_qr_codes(transformed_image)
		while cx<0 and cy<0:
			vision_sensor_image, image_resolution,_ = get_vision_sensor_image(client_id)
			transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
			qr_code,cx,cy,point = detect_qr_codes(transformed_image)
		if point and point[0][1]>point[1][1]:
			temp=point[0]
			point[0]=point[1]
			point[1]=temp
		rot=0
		if point:
			er=(point[0][0]-point[1][0])
			p = er/25
			d = er-erp
			erp=er
			rot= p+d/1000
			if rot>10:
				rot=10
			elif rot<-10:
				rot=-10
		e1=(256-cy)
		p = e1/50
		d = e1-ep1
		ep1=e1
		speed1= p+d/500
		if abs(e1)<5:
			speed1=0
		if speed1>10:
			speed1=10
		elif speed1<-10:
			speed1=-10
		e2=(cx-256)
		p = e2/50
		d = e2-ep2
		ep2=e2
		speed2= p+d/500
		if abs(e2)<5:
			speed2=0
		if speed2>10:
			speed2=10
		elif speed2<-10:
			speed2=-10
		#print(e1,e2,er)
		set_bot_movement(client_id,wheel_joints,speed1,speed2,rot)
	set_bot_movement(client_id,wheel_joints,0,0,0)


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

	for j in range(2):
		return_code,camera = sim.simxGetObjectHandle(client_id,"vision_sensor_1",sim.simx_opmode_blocking)
		return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id,camera,0,sim.simx_opmode_streaming+65535)
		return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id,camera,0,sim.simx_opmode_buffer)
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
	img = np.array(vision_sensor_image, dtype = np.uint8)
	img.resize([image_resolution[0],image_resolution[1],3])
	transformed_image = cv2.flip(img,0)
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
	# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	sim.simxGetPingTime(client_id)

    # Now close the connection to CoppeliaSim:
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
	qr_code = None
	cx=-1
	cy=-1
	point=[]
	decodedObjects = decode(transformed_image)
	for obj in decodedObjects:
		qr_code = str(obj.data.decode("utf-8"))
		qr_code = (int(qr_code.split(',')[0][1:]),int(qr_code.split(',')[1][1:-1]))
		point = obj.polygon
		(x1,y1)=point[0][:]
		(x2,y2)=point[2][:]
		cx=x1+(((x2-x1))/2)
		cy=y1+(((y2-y1))/2)
		point = sorted([point[0][:],point[1][:],point[2][:],point[3][:]])
	##################################################
	
	return qr_code,cx,cy,point


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
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[0],forw_back_vel+left_right_vel+rot_vel, sim.simx_opmode_streaming)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[1],forw_back_vel-left_right_vel+rot_vel, sim.simx_opmode_streaming)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[2],forw_back_vel+left_right_vel-rot_vel, sim.simx_opmode_streaming)
	sim.simxSetJointTargetVelocity(client_id, wheel_joints[3],forw_back_vel-left_right_vel-rot_vel, sim.simx_opmode_streaming)

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
	wheel_joints = [-1,-1,-1,-1]
	return_code1=1
	return_code2=1
	return_code3=1
	return_code4=1
	while(return_code1==1 or return_code2==1 or return_code3==1 or return_code4==1):
		return_code1,wheel_joints[0] = sim.simxGetObjectHandle(client_id,"rollingJoint_fl",sim.simx_opmode_blocking)
		return_code2,wheel_joints[1] = sim.simxGetObjectHandle(client_id,"rollingJoint_rl",sim.simx_opmode_blocking)
		return_code3,wheel_joints[2] = sim.simxGetObjectHandle(client_id,"rollingJoint_rr",sim.simx_opmode_blocking)
		return_code4,wheel_joints[3] = sim.simxGetObjectHandle(client_id,"rollingJoint_fr",sim.simx_opmode_blocking)

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


def nav_logic(client_id,joint_pos):
		"""
		Purpose:
		---
		This function should implement your navigation logic. 
		"""
		wheel_joints = init_setup(client_id)
		set_bot_movement(client_id,wheel_joints,0,0,0)
		[j0,j1]=joint_pos
		jp = encoders(client_id)
		e0=j0-jp[0]
		e1=j1-jp[1]
		ep0=0
		ep1=0
		s=100
		e=2
		while e0<-e or e1<-e or e0>e or e1>e:
			jp = encoders(client_id)
			if e0<-e or e0>e:
				e0=j0-jp[0]
				p=e0*2
				d=e0-ep0
				ep0=e0
				speedj0 = p+d/1000
				if speedj0>s:
					speedj0=s
				elif speedj0<-s:
					speedj0=-s
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[0],speedj0, sim.simx_opmode_streaming)
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[2],speedj0, sim.simx_opmode_streaming)
			else:
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[0],0, sim.simx_opmode_streaming)
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[2],0, sim.simx_opmode_streaming)
			if e1<-e or e1>e :
				e1=j1-jp[1]
				p=e1*2
				d=e1-ep1
				ep1=e1
				speedj1 = p+d/1000
				if speedj1>s:
					speedj1=s
				elif speedj1<-s:
					speedj1=-s
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[1],speedj1, sim.simx_opmode_streaming)
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[3],speedj1, sim.simx_opmode_streaming)
			else:
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[1],0, sim.simx_opmode_streaming)
				sim.simxSetJointTargetVelocity(client_id, wheel_joints[3],0, sim.simx_opmode_streaming)
		set_bot_movement(client_id,wheel_joints,0,0,0)
		#time.sleep(0.1)
	

def shortest_path(dest):
		"""
		Purpose:
		---
		This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
		"""
		s=11.1
		j0=dest[0]*(s)+dest[1]*(s)
		j1=dest[0]*(-s)+dest[1]*(s)
		joint_pos=[j0,j1]

		return joint_pos


def task_3_primary(client_id, target_points, to_compensate=False):
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
	for dest in target_points:
		#print('moving to',dest)
		dest=(dest[0]-4,dest[1]-4)
		joint_pos=shortest_path(dest)
		nav_logic(client_id,joint_pos)
	if to_compensate:
		compensate(client_id,init_setup(client_id))


if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(4, 5), (8, 5), (8, 9), (3, 1)]    # You can give any number of different co-ordinates


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