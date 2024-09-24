'''
*****************************************************************************************
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
*****************************************************************************************
'''


# Team ID:			[ eYRC#BM#2783 ]
# Author List:		[  B Nandhkishore, Sadha Sivam M, L G Divyanth ]
# Filename:			task_4.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from shutil import move
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode
from task_3 import init_remote_api_server, nav_logic, start_simulation, stop_simulation, exit_remote_api_server
import task_3,task_2a,task_1b

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
def call_open_close(client_id, command):
	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)


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
def detect_berries(client_id):
	"""
	Purpose:
	---
	Returns the detected berries'positions as a dictionary

	Input Arguments:
	---
	`client_id`			:[integer]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	Returns:
	---
	A dictionary of all the deteted berries' positions

	Example call:
	---
	berry_positions_dictionary = detect_berries(client_id)

	"""
	
	return_code,vision_sensor_handle = sim.simxGetObjectHandle(client_id,'vision_sensor_2',sim.simx_opmode_blocking)
	
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	vision_sensor_depth_image, depth_image_resolution, return_code = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)

	berries_dictionary = {}
	berry_positions_dictionary = {}
	berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)

	return berry_positions_dictionary

def joint_velocity(client_id):
	
	"""
	Purpose:
	---
	This function returns the joint velocities of all the joints in the robotic arm.

	Input Arguments:
	---
	`client_id`:		[integer]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	Returns:
	---
	None

	Example call:
	---
	jv = joint_velocity(client_id)


	"""
	return_code, j1_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r1', sim.simx_opmode_blocking) #Obtaining joint values for each wheel of the robot
	return_code, j2_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r2', sim.simx_opmode_blocking)
	return_code, j3_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r3', sim.simx_opmode_blocking)
	return_code, j4_handle = sim.simxGetObjectHandle(client_id, 'robotic_arm_rj_r4', sim.simx_opmode_blocking)
	
	joints = [j1_handle, j2_handle, j3_handle, j4_handle] 

	j_vel = []

	for j in joints:
		return_code, l_vel, a_vel = sim.simxGetObjectVelocity(client_id, j, sim.simx_opmode_blocking)
	
		a_vel_abs = [abs(ele) for ele in a_vel]
		j_vel.append(max(a_vel_abs))

	j_vel = sum(j_vel)
	return j_vel	


def move_to_point(client_id, p):
	"""
	Purpose:
	---
	This function moves the gripper to the passed point, p

	Input Arguments:
	---
	`client_id`			:[integer]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	`p`					:[tuple]
		point to which the target_dummy is to b moved, following inverse kinematics

	Returns:
	---
	None

	Example Call:
	---
	move_to_point(client_id, l1)

	"""
	return_code, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm', 1, 'kinematics', [],p, [], bytearray('', encoding='ascii'), sim.simx_opmode_blocking)
	jv = joint_velocity(client_id)
	while jv>0.4:
		return_code, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm', 1, 'kinematics', [],p, [], bytearray('', encoding='ascii'), sim.simx_opmode_blocking)
		jv = joint_velocity(client_id)
	
def pluck_berry(client_id, berry_pos, cb_no):
	"""
	Purpose:
	---
	This function actuates the gripper mechanism to pluck the berry, whose coordinate has been passed and deposit
	in the intermediate basket for the corresponding collection box number

	Input Arguments:
	---
	`client_id`			:[integer]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	`berry_pos`			:[tuple]
		contains the width, height and depth values of the berry to be plucked
	
	`cb_no`				:[integer]
		value of collection box number

	Returns:
	---
	None

	Example Call:
	---
	pluck_berry(client_id, berry_pos, "Blueberry")

	"""
	l = berry_pos


	if l[0] > 0: #Blueberry
		
		
		
		k = 0
		y = 0.02
		while y <= 0.24:
			l1 = [l[1]+0.035,y,k]
			move_to_point(client_id,l1)
			k += -(l[0] + 0.16)/12
			y += 0.02
		
		y = 0.24
		
		j=30
		while j>1:
			call_open_close(client_id, "open")
			j-=1

		k = -(l[0] + 0.16)
	
		l1 = [l[1]-0.045,y,k]
		move_to_point(client_id,l1)


		while k < (-l[0] - 0.02):
			l1 = [l[1]-0.045,y,k]
			move_to_point(client_id,l1)
			y += (l[2]- 0.24)/8
			k += 0.02

	
	else:	#Strawberry
		
		
		
		k = 0
		y = 0.02
		while y <= 0.24:
			l1 = [l[1]+0.035,y,k]
			move_to_point(client_id,l1)
			k += (-l[0] +  0.16)/12
			y += 0.02
		
		y = 0.24

		j=30
		while j>1:
			call_open_close(client_id, "open")
			j-=1

		k = (-l[0] + 0.16)

		l1 = [l[1]-0.045,y,k]
		move_to_point(client_id,l1)

		
		while k > (-l[0] + 0.02):
			l1 = [l[1]-0.045,y,k]
			move_to_point(client_id,l1)
			y += (l[2] - 0.24)/8
			k -= 0.02

	
	l1 = [l[1] - 0.035, l[2], -l[0]]
	move_to_point(client_id, l1)

	l1 = [l[1] - 0.02, l[2], -l[0]]
	move_to_point(client_id, l1)

	l1 = [l[1] - 0.01, l[2], -l[0]]
	move_to_point(client_id, l1)

	l1 = [l[1], l[2], -l[0]]
	move_to_point(client_id, l1)

	j=30
	while j>1:
		call_open_close(client_id, "close")
		j-=1

	
	l1 = [l[1]-0.01, l[2]-0.04, -l[0]-0.02]
	move_to_point(client_id, l1)
	
	if l[0]<0:
		l1 = [0.3, 0.1, -l[0]] 		#l1 = [h,front,left]
		move_to_point(client_id, l1)

		l1 = [0.3, 0.1, 0] 		#l1 = [h,front,left]
		move_to_point(client_id, l1)


	if cb_no =='1':
		cb1 = [0.3, -0.08, -0.12]
		move_to_point(client_id, cb1)

	else:
		cb2 = [0.3, 0.08, -0.12]
		move_to_point(client_id, cb2)


	j=50
	while j>1:
		call_open_close(client_id, "open")
		j-=1


	l1 = [0.2, 0.1, 0] 		#l1 = [h,front,left]
	move_to_point(client_id, l1)

	##############################################################
def task_4_primary(client_id, current, target_points, obstacles):	
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
	task_3.task_3_primary(client_id,target_points, obstacles, current)
	
	
	berry_positions_dictionary = detect_berries(client_id)
	lemon = berry_positions_dictionary["Lemon"]
	berry_pos = lemon[0]
	return_code = send_identified_berry_data(client_id, 'Lemon', berry_pos[0], -berry_pos[1], berry_pos[2]-0.045)
	pluck_berry(client_id, berry_pos, 'Lemon')

	berry_positions_dictionary = detect_berries(client_id)
	strawberry = berry_positions_dictionary["Strawberry"]
	berry_pos = strawberry[0]
	return_code = send_identified_berry_data(client_id, "Strawberry", berry_pos[0], -berry_pos[1], berry_pos[2]-0.045)
	pluck_berry(client_id, berry_pos, "Strawberry")	

	berry_positions_dictionary = detect_berries(client_id)
	blueberry = berry_positions_dictionary["Blueberry"]
	berry_pos = blueberry[0]
	return_code = send_identified_berry_data(client_id, "Blueberry", berry_pos[0], -berry_pos[1], berry_pos[2]-0.045)
	pluck_berry(client_id, berry_pos, "Blueberry")	

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
		#time.sleep(1)        

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