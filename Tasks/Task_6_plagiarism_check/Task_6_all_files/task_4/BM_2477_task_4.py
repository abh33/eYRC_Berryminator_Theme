'''
*
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
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
# Filename:			task_4.py
# Functions:		[ init_remote_api_server, start_simulation, stop_simulation, exit_remote_api_server, call_open_close, send_identified_berry_data, task_4_primary]
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from ssl import CHANNEL_BINDING_TYPES
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode
import task_3
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
	
	return_code=sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)

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

def call_open_close(client_id, command):

	################# Call gripper script #################

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)

	#######################################################

def detect_position( client_id):

	################	DETECT POSITIONS  ##############

	vision_sensor_image=[]
	vision_sensor_depth_image=[]

	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	while(len(vision_sensor_image)==0):
		vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)

	while(len(vision_sensor_depth_image)==0):
		vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)
	
	"""berries = ['lemon_1','lemon_2','strawberry_1','strawberry_2','blueberry_1','blueberry_2']
	for i in berries:
		return_code, berry_handle = sim.simxGetObjectHandle(client_id, i, sim.simx_opmode_blocking)
		returnCode, position = sim.simxGetObjectPosition(client_id, berry_handle, vision_sensor_handle, sim.simx_opmode_blocking)
		print("position of :",i," : ",position)"""
	#################################################

	return berry_positions_dictionary

def open_gripper(client_id):

	#################	Open Gripper  ################

	return_code, gripper_joint = sim.simxGetObjectHandle(client_id, 'RG2_openCloseJoint', sim.simx_opmode_blocking)
	returnCode,pos=sim.simxGetJointPosition(client_id,gripper_joint,sim.simx_opmode_blocking)

	while(round(pos,3)!=round(0.045542240142822266,3)):

		returnCode,pos=sim.simxGetJointPosition(client_id,gripper_joint,sim.simx_opmode_blocking)

		call_open_close(client_id, "open")
	
	#################################################

def close_gripper(client_id):

	#################	Close Gripper  ################

	return_code, gripper_joint = sim.simxGetObjectHandle(client_id, 'RG2_openCloseJoint', sim.simx_opmode_blocking)
	returnCode,pos=sim.simxGetJointPosition(client_id,gripper_joint,sim.simx_opmode_blocking)

	while(round(pos,4)!=round(-0.04078209400177002,4)):

		returnCode,pos=sim.simxGetJointPosition(client_id,gripper_joint,sim.simx_opmode_blocking)

		call_open_close(client_id, "close")

	#################################################

def set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, pos1, pos2, pos3):
	"""
	Input arguments:

	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	joint_handle1: Handle of the first revolute joint present in base and link1 intersection in the robotic arm

	joint_handle2: Handle of the second revolute joint present in link1 and link2 intersection in the robotic arm
	
	joint_handle3: Handle of the third revolute joint present in link2 and link3 intersection in the robotic arm

	pos1: Angle at which first revolute joint stops

	pos2: Angle at which second revolute joint stops

	pos3: Angle at which third revolute joint stops

	"""
	##################################### Set Joint Drop Position ####################################

	sim.simxSetJointTargetPosition(client_id,joint_handle1,pos1*math.pi/180,sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id,joint_handle2,pos2*math.pi/180,sim.simx_opmode_oneshot)
	sim.simxSetJointTargetPosition(client_id,joint_handle3,pos3*math.pi/180,sim.simx_opmode_oneshot)

	returnCode,position1=sim.simxGetJointPosition(client_id,joint_handle1,sim.simx_opmode_blocking)
	returnCode,position2=sim.simxGetJointPosition(client_id,joint_handle2,sim.simx_opmode_blocking)
	returnCode,position3=sim.simxGetJointPosition(client_id,joint_handle3,sim.simx_opmode_blocking)

	while(round(position1,2)!=round(pos1*math.pi/180,2) or round(position2,2)!=round(pos2*math.pi/180,2) or round(position3,2)!=round(pos3*math.pi/180,2)):

		returnCode,position1=sim.simxGetJointPosition(client_id,joint_handle1,sim.simx_opmode_blocking)
		returnCode,position2=sim.simxGetJointPosition(client_id,joint_handle2,sim.simx_opmode_blocking)
		returnCode,position3=sim.simxGetJointPosition(client_id,joint_handle3,sim.simx_opmode_blocking)

	#####################################################################################################


def move_to_berry( client_id, joint_handle1, joint_handle2, joint_handle3, berry_pos):
	"""
	Input arguments:

	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	joint_handle1: Handle of the first revolute joint present in base and link1 intersection in the robotic arm

	joint_handle2: Handle of the second revolute joint present in link1 and link2 intersection in the robotic arm

	joint_handle3: Handle of the third revolute joint present in link2 and link3 intersection in the robotic arm

	berry_pos: 3d position of target berry

	"""
	########################################## Move Tip to Berry #########################################
	
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer=sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'collect_berries',[],berry_pos,[],emptybuff,sim.simx_opmode_blocking)

	returnCode,position1_initial=sim.simxGetJointPosition(client_id,joint_handle1,sim.simx_opmode_blocking)
	returnCode,position2_initial=sim.simxGetJointPosition(client_id,joint_handle2,sim.simx_opmode_blocking)
	returnCode,position3_initial=sim.simxGetJointPosition(client_id,joint_handle3,sim.simx_opmode_blocking)

	while(True):

		returnCode,position1_final=sim.simxGetJointPosition(client_id,joint_handle1,sim.simx_opmode_blocking)
		returnCode,position2_final=sim.simxGetJointPosition(client_id,joint_handle2,sim.simx_opmode_blocking)
		returnCode,position3_final=sim.simxGetJointPosition(client_id,joint_handle3,sim.simx_opmode_blocking)

		if(round(position1_initial-position1_final,2)==0 and round(position2_initial-position2_final,2)==0 and round(position3_initial-position3_final,2)==0):
			break
		position1_initial=position1_final
		position2_initial=position2_final
		position3_initial=position3_final

	##########################################################################################################

def pluck_a_berry( client_id, berry_name, berry_CB, berry_pos, last_berry):
	"""
	Input arguments:

	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	berry_CB: The collection box in which the rewspective berry has to be dropped

	berry_pos: 3d position of target berry

	"""
	############################################	Pluck a Berry	###########################################

	return_code, joint_handle1 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking)
	return_code, joint_handle2 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r2', sim.simx_opmode_blocking)
	return_code, joint_handle3 = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r3', sim.simx_opmode_blocking)

	call_open_close(client_id, "open")

	move_to_berry( client_id, joint_handle1, joint_handle2, joint_handle3, berry_pos)

	close_gripper(client_id)
	
	if(berry_CB=='CB1' ):
		if(berry_pos[0] > 0.2):
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, 165, 10, -10)
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, 165, -25, 25)
		else:
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, 165, -20, 20)
	else:
		if(berry_pos[0] > 0.2):
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, -165, 10, -10)
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, -165, -25, 25)
		else:
			set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, -165, -25, 25)

	open_gripper(client_id)

	call_open_close(client_id, "close")
	
	if( last_berry):

		sim.simxSetJointTargetPosition(client_id,joint_handle3,25*math.pi/180,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id,joint_handle2,-25*math.pi/180,sim.simx_opmode_oneshot)
		sim.simxSetJointTargetPosition(client_id,joint_handle1,0*math.pi/180,sim.simx_opmode_oneshot)

	else:
		set_joint_position( client_id, joint_handle1, joint_handle2, joint_handle3, 0, -25, 25)
	
	###########################################################################################################


def pluck_berries( client_id, berry_positions_dictionary,Berry_details,Berry_count):
	"""
	Input arguments:

	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	berry_CB: The collection box in which the respective berry has to be dropped

	berry_positions_dictionary: It contains the 3d position of all berries

	Berry_details: Contains the details of collection box with respect to json file

	Berry_count: Contains the number of berries to be dropped

	"""
	########################################### 	Pluck Berries	#######################################

	lemon_in_plant=len(berry_positions_dictionary['Lemon'])
	strawberry_in_plant=len(berry_positions_dictionary['Strawberry'])
	blueberry_in_plant=len(berry_positions_dictionary['Blueberry'])

	lemon_plucked=0
	strawberry_plucked=0
	blueberry_plucked=0

	lemon_CB=Berry_details['L'][2:5]
	strawberry_CB=Berry_details['S'][2:5]
	blueberry_CB=Berry_details['B'][2:5]

	last_berry = False
	for i in range(Berry_count['L']):
		if(lemon_in_plant != 0):
			if((lemon_in_plant == 1 or lemon_plucked == Berry_count['L'] - 1) and   blueberry_plucked == Berry_count['B'] and strawberry_plucked == Berry_count['S']):
				last_berry = True
			send_identified_berry_data(client_id,'Lemon',berry_positions_dictionary['Lemon'][i][0],berry_positions_dictionary['Lemon'][i][1],berry_positions_dictionary['Lemon'][i][2])
			pluck_a_berry( client_id, 'Lemon', lemon_CB, berry_positions_dictionary['Lemon'][i], last_berry)
			lemon_plucked+=1
			lemon_in_plant-=1

	for i in range(Berry_count['S']):
		if(strawberry_in_plant != 0):
			if((strawberry_in_plant == 1 or strawberry_plucked == Berry_count['S'] - 1) and   blueberry_plucked == Berry_count['B'] ):
				last_berry = True

			send_identified_berry_data(client_id,'Strawberry',berry_positions_dictionary['Strawberry'][i][0],berry_positions_dictionary['Strawberry'][i][1],berry_positions_dictionary['Strawberry'][i][2])
			pluck_a_berry( client_id, 'Strawberry', strawberry_CB, berry_positions_dictionary['Strawberry'][i], last_berry)
			strawberry_plucked+=1
			strawberry_in_plant-=1

	for i in range(Berry_count['B']):
		if(blueberry_in_plant != 0):
			if(blueberry_in_plant == 1 or blueberry_plucked == Berry_count['B'] - 1):
				last_berry = True
			
			send_identified_berry_data(client_id,'Blueberry',berry_positions_dictionary['Blueberry'][i][0],berry_positions_dictionary['Blueberry'][i][1],berry_positions_dictionary['Blueberry'][i][2])
			pluck_a_berry( client_id,  'Blueberry', blueberry_CB, berry_positions_dictionary['Blueberry'][i], last_berry)

			blueberry_plucked+=1
			blueberry_in_plant-=1

	Berry_count['L']-=lemon_plucked
	Berry_count['S']-=strawberry_plucked
	Berry_count['B']-=blueberry_plucked
		
	####################################################################################################

	return Berry_count


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
##############################################################


def task_4_primary(client_id):
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()


	Returns:
	---

	Example call:
	---
	task_4_primary(client_id)

	"""
	##############	NAVIGATION	##############
	task_3.task_3_primary(client_id, [(4,3),(4,4)])
	##########################################

	berry_positions_dictionary=detect_position( client_id)
	pluck_berries( client_id, berry_positions_dictionary)

	##########################################


if __name__ == "__main__":

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