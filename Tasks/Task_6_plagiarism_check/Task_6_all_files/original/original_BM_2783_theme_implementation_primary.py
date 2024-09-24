'''
*****************************************************************************************
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
*****************************************************************************************
'''


# Team ID:			[ eYRC#BM#2783 ]
# Author List:		[  B Nandhkishore, Sadha Sivam M, L G Divyanth ]
# Filename:			theme_implementation.py
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
import json
from pyzbar.pyzbar import decode
from task_3 import init_remote_api_server, start_simulation, stop_simulation, exit_remote_api_server
import task_4, task_3, task_2a, task_1b

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
def unload_berries(client_id,prev_req_rot,dir, cb_no):
	
	return_code, box_j_handle = sim.simxGetObjectHandle(client_id, 'box_rj', sim.simx_opmode_blocking) #Obtaining joint values of the robot
	return_code, d1_j_handle = sim.simxGetObjectHandle(client_id, 'd1_rj', sim.simx_opmode_blocking)
	return_code, d2_j_handle = sim.simxGetObjectHandle(client_id, 'd2_rj', sim.simx_opmode_blocking)
	
	dist = 0.16
	dist_cov = 0

	v_mag = 0.2
	vx = dir*v_mag						#Direction of movement changes before and after unloading
	vy =0
	w =0
	v = np.matrix([[w],[vy],[-1*vx]], float)

	h = np.matrix('-0.3965 1 -1; 0.3965 1 1; 0.3965 1 -1; -0.3965 1 1', float)
	j = np.matmul(h,v) 					#Calculation of joint velocities

	tot_time = dist/v_mag				#Total time calculation
	req_rot_m = tot_time*20*j			#Total rotation of each joints required to reach the desired configuration
	wheel_joints = task_3.init_setup(client_id)		#Acquiring joint id's of the wheel joints



	req_rot = [0,0,0,0]			#Total required rotation of joints to traverse from current point to the next target point
	act_rot = [0,0,0,0]			#Actual required rotation of joints till any time 't'
	t = 0						#Current time
	dt = 0.05
	
	
	
	t_last = 0										#Temprorary variable used to store the time at which previous target point was reached
	
	i_del_v = np.matrix([[0],[0],[0]], float)		#Variable to store integral of the error 
	del_v_prev = np.matrix([[0],[0],[0]], float)	#Variable to store error in previous time step
	dist_cov = 0									#Variable to store the distance covered till the current iteration
		
	for h in range(len(req_rot)):
		req_rot[h] = prev_req_rot[h] + req_rot_m[h,0] 	#Conversion of matrix to list
	
	while dist - dist_cov > 0.05:
			
			cur_rot = task_3.encoders(client_id) 	#Actual rotation of joints retrieved from the encoders
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
				task_3.set_bot_movement(client_id, wheel_joints, vy, vx , w ) #Moving the bot with corrected velocity
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
				d_del_v = 0.004*d_del_v		#Differential correction term

				c_del_v = p_del_v + i_del_v + d_del_v 	#Final correction term
	

				task_3.set_bot_movement(client_id, wheel_joints, vy + c_del_v[1,0], vx - c_del_v[2,0], w + c_del_v[0,0]) #Moving the bot with corrected velocity
				t = t+dt 	#incrementing to the next time step

				
	i_del_v = 0*i_del_v											#Integral term reset to zero - Flushing out the integral term after reaching every target point
	task_3.set_bot_movement(client_id, wheel_joints, 0, 0, 0)	#Bot velocity set to zero to start again

	box_j_pos = sim.simxSetJointTargetPosition(client_id, box_j_handle,20, sim.simx_opmode_oneshot) #Setting joint position for unloading
	time.sleep(2)

	if cb_no == 1:
		d1_j_pos = sim.simxSetJointTargetPosition(client_id, d1_j_handle,20, sim.simx_opmode_oneshot)	#Setting joint position for unloading in Collection Box 1
		time.sleep(4)

	if cb_no == 2:
		d2_j_pos = sim.simxSetJointTargetPosition(client_id, d2_j_handle,20, sim.simx_opmode_oneshot)	#Setting joint position for unloading in Collection Box 2
		time.sleep(4)
	return req_rot

def pluck_module(cb_no,qt,client_id):
	berry_type = ['Lemon','Strawberry','Blueberry']						#Defining berry types
	berry_positions_dictionary = task_4.detect_berries(client_id)		#Detecting berrry coordinates

	#Plucking berries of each type available
	for berry in berry_type:
		btype = berry[0]		#Retrieving first letter of Berry type (L,S,B)
		berry_pos_list = berry_positions_dictionary[berry]	#Retrieving the positions of the berry
		berry_count = len(berry_pos_list)		#Retrieving number of berries available
	
		#Pluck berries until they are available in the room or until required number of berries have been plucked
		while not (qt[btype] == 0) and not (berry_count == 0):
			berry_pos = berry_pos_list[0]			
			return_code = task_4.send_identified_berry_data(client_id, berry, berry_pos[0], -berry_pos[1], berry_pos[2]-0.05) #sending identified berry data
			task_4.pluck_berry(client_id, berry_pos, cb_no[btype]) 	#Pluck the desired berry
			qt[btype] -= 1		#Decrimenting the count after succesfully plucking the berries
			berry_positions_dictionary = task_4.detect_berries(client_id)		#Detecting currently available berries and their positions after plucking
			berry_pos_list = berry_positions_dictionary[berry]		
			berry_count = len(berry_pos_list)

	#checking whether all berries are plucked
	if sum(qt.values()) == 0:
		return 0
	else:
		return 1


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
	---w
	theme_implementation_primary(client_id, rooms_entry)
	
	"""
	current = (4,4)

	f = open('Theme_Config.json')
	data = json.load(f)
	
	qt = {'L':int(data['L'][0]),		#Retrieving number of berries required 
		  'S':int(data['S'][0]),		
		  'B':int(data['B'][0])}


	cb_no = {'B':data["B"][4],			#Retrieving appropriate collection boxes
			 'L':data["L"][4],
			 'S':data["S"][4]}

	room_coords = [(1,7), (7,7), (7,1), (1,1)]	#Defining room center coordinates
	T_P = (4,9)									#Intermediate terminal point for navigation
	CB_1 = [(1,10)]								#Collection box coordinates
	CB_2 = [(7,10)]
	
	CB_POINTS = CB_1 + CB_2
	pathway = [(4,i) for i in range(0,12)] + [(i,10) for i in range(0,9)] + [(i,4) for i in range(0,9)] 	#Obstacle free pathway for navigation
	route_map = []	
	prev_req_rot = [0,0,0,0]		#Initial rotation of joints during start of start of simulation is zero
	j = 0
	ortn = 0
	status = 1

	rooms_entry = rooms_entry[0:2]		
	

	for i in range(len(rooms_entry)):
		j = i

		#Checking for requirement of rotation of the bot before entering a room
		if i == 0:			
			if rooms_entry[i][1] != 5:
				if ortn != 90:
					prev_req_rot = task_3.rot_by_angle(-1.57,prev_req_rot,client_id)
					ortn = 90
			else:
				if ortn != 0:
					prev_req_rot = task_3.rot_by_angle(1.57,prev_req_rot,client_id)
					ortn = 0
		if i == 1:
			if rooms_entry[i][1] != 5 and rooms_entry[i][1] != 9:
				if ortn != 90:
					prev_req_rot = task_3.rot_by_angle(-1.57,prev_req_rot,client_id)
					ortn = 90
			else:
				if ortn != 0:
					prev_req_rot = task_3.rot_by_angle(1.57,prev_req_rot,client_id)
					ortn = 0

		if i == 2 or i == 3:
			if rooms_entry[i][1] != 3:
				if ortn != 90:
					prev_req_rot = task_3.rot_by_angle(-1.57,prev_req_rot,client_id)
					ortn = 90
			else:
				if ortn != 0:
					prev_req_rot = task_3.rot_by_angle(1.57,prev_req_rot,client_id)
					ortn = 0

		route_map = task_3.shortest_path(current,rooms_entry[i],pathway+[rooms_entry[i]]) #Retrieving shortest path

		prev_req_rot = task_3.nav_logic(prev_req_rot,route_map,0.5,ortn,client_id)	#Navigating through the generated shortest path

	#Genrating path for entering the room center from the rooms entry
		if rooms_entry[i][1] == 5:
			path = [rooms_entry[i],(rooms_entry[i][0],6), room_coords[i]]		
		if rooms_entry[i][1] >=6 and i==0:
			path = [rooms_entry[i],(rooms_entry[i][0]-1,rooms_entry[i][1]), room_coords[i]]
		if rooms_entry[i][1] >=6 and i==1:
			path = [rooms_entry[i],(rooms_entry[i][0]+1,rooms_entry[i][1]), room_coords[i]]

	#Navigating inside the room
		prev_req_rot = task_3.nav_logic(prev_req_rot,path,0.25,ortn,client_id)
		current = route_map[-1]

	#Rotation of bot for leaving the room through the entry point
		if ortn != 90*i:
			prev_req_rot = task_3.rot_by_angle(-1.57*(i - ortn/90),prev_req_rot,client_id)

	#Status of whether the required amount of berries are plucked, status = 0 all berries plucked
		status = pluck_module(cb_no,qt,client_id)
		
		if ortn != 90*i:
			prev_req_rot = task_3.rot_by_angle(1.57*(i - ortn/90),prev_req_rot,client_id)

		path.reverse()
		path = path + [route_map[-2]]  #Path for coming out of the room through the entry point
		route_map +=  [room_coords[i]]
		route_map = route_map[0:3]

		prev_req_rot = task_3.nav_logic(prev_req_rot,path,0.25,ortn,client_id) #Navigating through the generated path for coming out of the room
		current = path[-1]

		route_map = []
		
	#Break out of the loop and navigate to collection boxes if required number of berries are plucked
		if status == 0:
			break
	
	#Deciding the amount of rotation to orient the storage basket in the desired configuration for unloading
	if ortn == 0:
			prev_req_rot = task_3.rot_by_angle(1.57,prev_req_rot,client_id)
	
	if ortn == 90:
		for i in range(2):
			prev_req_rot = task_3.rot_by_angle(-1.57,prev_req_rot,client_id)


	#Navigate to the collection boxes via terminal points
	route_map = task_3.shortest_path(current,T_P,pathway)
	prev_req_rot = task_3.nav_logic(prev_req_rot,route_map,0.5,270,client_id)

	#Navigate to collection box 1
	route_map = task_3.shortest_path(T_P,CB_1[0],pathway)
	prev_req_rot = task_3.nav_logic(prev_req_rot,route_map,0.5,270,client_id)

	#Unload the beries to Collection box 1 and return to the lane
	prev_req_rot = unload_berries(client_id,prev_req_rot,1, 1)
	prev_req_rot = unload_berries(client_id,prev_req_rot,-1, 1)

	#Navigate to collection box 1
	route_map = task_3.shortest_path(CB_1[0],CB_2[0],pathway)
	prev_req_rot = task_3.nav_logic(prev_req_rot,route_map,0.5,270,client_id)

	#Unload the beries to Collection box 2 and return to the lane
	prev_req_rot = unload_berries(client_id,prev_req_rot,1, 2)
	prev_req_rot = unload_berries(client_id,prev_req_rot,-1, 2)
	time.sleep(3)
	route_map = []




if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [(3,6),(6,9)]     # example list of tuples

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