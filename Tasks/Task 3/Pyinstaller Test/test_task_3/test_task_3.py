'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 3         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task_3.py
*  Created:				
*  Last Modified:		
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode
import platform
import csv
import string
import random
from datetime import datetime
from itertools import islice
import cryptocode
import uuid
import random


global output_list_child, rtf_python, init_real_time, end_real_time, end_simulation_time
eval_rtf_python = 0


# Global variable "client_id" for storing ID of starting the CoppeliaSim Remote connection
# NOTE: DO NOT change the value of this "client_id" variable here
client_id = -1


if hasattr(sys, "frozen"):
	# print("executable", sys.executable)
	sys.path.append(os.path.dirname(sys.executable))


# NOTE: Refer https://stackoverflow.com/questions/39885354/pyinstaller-cannot-add-txt-files
def resource_path(relative_path):
	""" Get absolute path to resource, works for dev and for PyInstaller """
	try:
		# PyInstaller creates a temp folder and stores path in _MEIPASS
		base_path = sys._MEIPASS
	except Exception:
		base_path = os.environ.get("_MEIPASS2",os.path.abspath("."))

	return os.path.join(base_path, relative_path)


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	sim = __import__('sim')
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()


def init_remote_api_server():
	"""
	Purpose:
	---
	This function should first close any open connections and then start
	communication thread with server i.e. CoppeliaSim.

	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
	The evaluation script will handle that condition.

	Input Arguments:
	---
	None

	Returns:
	---
	`client_id`     :  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable

	Example call:
	---
	client_id = init_remote_api_server()

	NOTE: This function will be automatically called by evaluation script before starting the simulation.
	"""

	global client_id
	##############  ADD YOUR CODE HERE  ##############

	sim.simxFinish(-1)  # just in case, close all opened connections
	client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to CoppeliaSim
	sim.simxGetPingTime(client_id)

	##################################################

	return client_id


def start_simulation():
	"""
	Purpose:
	---
	This function should first start the simulation if the connection to server
	i.e. CoppeliaSim was successful and then wait for last command sent to arrive
	at CoppeliaSim server end.

	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
	The evaluation script will handle that condition.

	Input Arguments:
	---
	None

	Returns:
	---
	`return_code`   :  [ integer ]
		the return code generated from the start running simulation remote API

	Example call:
	---
	return_code = start_simulation()

	NOTE: This function will be automatically called by evaluation at the start of simulation.
	"""

	global client_id

	##############  ADD YOUR CODE HERE  ##############

	# return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
	if client_id != -1:
		return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

	# Making sure that last command sent out had time to arrive
	sim.simxGetPingTime(client_id)

	##################################################

	return return_code


def stop_simulation():

	"""
	Purpose:
	---
	This function should stop the running simulation in CoppeliaSim server.

	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
	The evaluation script will handle that condition.
	
	Input Arguments:
	---
	None
	
	Returns:
	---
	`return_code`   :  [ integer ]
		the return code generated from the stop running simulation remote API
	
	Example call:
	---
	return_code = stop_simulation()
	
	"""

	global client_id

	##############  ADD YOUR CODE HERE  ##############
 
	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait)
	sim.simxGetPingTime(client_id)

	##################################################

	return return_code


def exit_remote_api_server():

	"""
	Purpose:
	---
	This function should wait for the last command sent to arrive at the Coppeliasim server
	before closing the connection and then end the communication thread with server
	i.e. CoppeliaSim using simxFinish Remote API.

	Input Arguments:
	---
	None

	Returns:
	---
	None

	Example call:
	---
	exit_remote_api_server()

	"""

	global client_id

	##############  ADD YOUR CODE HERE  ##############

	sim.simxGetPingTime(client_id)
	sim.simxFinish(client_id)

	##################################################


# This function will replace the BM_Bot in the scene
def replacement():
	global client_id

	# Getting object handles
	return_code, vs_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	return_code, BM_Bot_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot', sim.simx_opmode_blocking)


	# Before removing their model, setting its position to (0,0) and orientation
	return_code = sim.ioxbbmnbkj( client_id, BM_Bot_handle, -1 , [ 0, 0, 9.5750e-02], sim.simx_opmode_blocking)
	return_code = sim.vxbvnn( client_id, BM_Bot_handle, -1 , [ math.pi/2, 0, math.pi/2], sim.simx_opmode_blocking)

	# Making Vision sensor parentless
	return_code = sim.simxSetObjectParent( client_id, vs_handle, -1, True, sim.simx_opmode_blocking)

	# Removing their BM_Bot
	return_code = sim.simxRemoveModel( client_id, BM_Bot_handle, sim.simx_opmode_blocking)

	# Adding our BM_Bot, by default at (0,0) which is what we required
	return_code, BM_Bot_handle = sim.simxLoadModel( client_id, resource_path('BM_Bot.ttm'), 0, sim.simx_opmode_blocking) #Load the new model

	# Making BM_Bot_base as parent of vision_sensor_1
	return_code, BM_Bot_base_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot_base', sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, vs_handle, BM_Bot_base_handle, True, sim.simx_opmode_blocking)


# Need this function to call arm_check function only once after atleast one time step of simulation
def wait_for_check_scene():
	global client_id

	return_code_signal = 1
	flag_scene_all_ok = '0'
	init_timeout = time.time()
	end_timeout = init_timeout
	timeout = end_timeout - init_timeout

	# inputBuffer = bytearray()
	# return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc',\
	# 								sim.sim_scripttype_customizationscript,'arm_check',[],[],[],inputBuffer,sim.simx_opmode_blocking)


	# We will wait till we get the signal value. If the correct flag_scene_all_ok signal value is not obtained,
	#  or the timer expires the code will NOT proceed further.
	while(return_code_signal != 0 or flag_scene_all_ok != '1'):
		return_code_signal,flag_scene_all_ok = sim.simxGetStringSignal(client_id, 'gfh36801nc', sim.simx_opmode_blocking)
		end_timeout = time.time()
		timeout = end_timeout - init_timeout
		flag_scene_all_ok = str(flag_scene_all_ok, 'utf-8')
		# print('flag_scene_all_ok',flag_scene_all_ok)
		# print('return_code_signal',return_code_signal)
		if(timeout > 10):
			print('\n[ERROR] Scene could not be checked. Check CoppeliaSim status bar.')
			print('Exiting evaluation')
			end_program()
			sys.exit()
			break


def load_eval_model():
	global client_id

	# Remove model if previously loaded.
	try:
		return_code, dummy_handle = sim.simxGetObjectHandle(client_id, 'BM_Dummy_3', sim.simx_opmode_blocking)
		if(return_code == 0): #This means that the object exists from before
			return_code = sim.simxRemoveModel( client_id, dummy_handle, sim.simx_opmode_blocking)
		
		#cwd = os.getcwd()
		#return_code,evaluation_screen_handle=sim.simxLoadModel(client_id,cwd+'//evaluation_projector_screen.ttm',0,sim.simx_opmode_blocking) #Load the new model
		return_code, disc_handle = sim.simxLoadModel( client_id, resource_path('task_3_dummy.ttm'), 0, sim.simx_opmode_blocking) #Load the new model
		if(return_code != 0):
			# print('[ERROR] Evaluation script failed to load. Please try again.')
			print('\n[ERROR] Evaluation failed to start. Please try again.')
			end_program()
			sys.exit()
		else:
			# print('Evaluation script loaded successfully.')
			print('\nEvaluation started successfully.')
	except Exception:
		end_program()


# End the program
# Will stop the simulation, call organize_screen_end, clear string signals, exit the server
def end_program():


	return_code = stop_simulation()

	return_code, disc_handle = sim.simxGetObjectHandle(client_id, 'BM_Dummy_3', sim.simx_opmode_blocking)

	if (return_code == 0):				# This means that the object exists from before
		inputBuffer = bytearray()
		return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'BM_Dummy_3',\
									sim.sim_scripttype_customizationscript,'organize_screen_end',[],[],[],inputBuffer,sim.simx_opmode_blocking)


	# This is used to clear all the signals.
	return_code = sim.simxClearStringSignal(client_id, '', sim.simx_opmode_oneshot)

	try:
		exit_remote_api_server()

		if (start_simulation() == sim.simx_return_initialize_error_flag):
			print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

		else:
			print('\n[ERROR] Failed disconnecting from Remote API server!')
			# print('[ERROR] exit_remote_api_server function in task_2a.py is not configured correctly, check the code!')

	except Exception:
		# print('\n[ERROR] Your exit_remote_api_server function in task_2a.py throwed an Exception. Kindly debug your code!')
		print('\n[ERROR] The connection to Remote API Server did not end successfully!')
		print('Stop the CoppeliaSim simulation manually if required.\n')
		# traceback.print_exc(file=sys.stdout)
		print()


def get_child_data():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, data_child, retBuffer = sim.simxCallScriptFunction(client_id, 'BM_Dummy_3',\
									sim.sim_scripttype_childscript,'get_required_data_child',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("data_child:")
	# print(data_child)

	return data_child


def task_3_cardinal_main(target_points):

	global output_list_child, rtf_python, eval_rtf_python, init_real_time, end_real_time, end_simulation_time

	try:
		task_3 = __import__('task_3')

	except ImportError:
		print('\n[ERROR] task_3.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_3.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_3.py threw an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()


	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Students should have opened task_3_scene.ttt

			# Loading our task 3 dummy model
			load_eval_model()

			# Check scene
			wait_for_check_scene()

			# Replacing BM_Bot
			replacement()

			# Setting string signal with the target points
			points = str(target_points[0]) +"%"+ str(target_points[1]) + "%"+ str(target_points[2]) + "%" + str(target_points[3])
			# points = str(target_points[0]) +"%"+ str(target_points[1]) + "%"+ str(target_points[2]) + "%" + str(target_points[3])  + "%" + str(target_points[4])  + "%" + str(target_points[5])  + "%" + str(target_points[6])  + "%" + str(target_points[7])
			returnCode = sim.simxSetStringSignal(client_id, "points", points, sim.simx_opmode_oneshot)

			init_real_time = time.time()

			#  Start simulation
			return_code = start_simulation()

			if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
				print('\nSimulation started correctly in CoppeliaSim.')

			else:
				print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
				end_program()
				sys.exit()

			try:
				# Running student's logic for four random co-ordinates
				task_3.task_3_primary(client_id, target_points)

			except KeyboardInterrupt:
				print('\n[ERROR] Test script for Task 3 interrupted by user!')
				# end_program()			

			except Exception:
				print('\n[ERROR] Your task_3_primary() function threw an error.')
				print('Kindly debug your code.')
				traceback.print_exc(file=sys.stdout)
				print()
				end_program()
				sys.exit()
			


			# input('Press enter to end ')   ## DELETE THIS

			# Getting data from child script
			output_list_child = get_child_data()
			# p = ",".join(data_child)
			# output_list_child = p.split(',')
			# print(output_list_child)

			#  Stop simulation
			return_code = stop_simulation()
			end_real_time = time.time()

			# RTF calculation
			end_simulation_time = 0; rtf_python = 0
			return_code, end_simulation_time = sim.simxGetStringSignal(client_id, 'time', sim.simx_opmode_blocking)
			rtf_python = float("{0:.5f}".format(float(end_simulation_time)/(end_real_time - init_real_time)))
			print('\nCalculated Real-Time Factor (rtf) = ', rtf_python)
			if rtf_python >= 0.8:
				eval_rtf_python = 1
			else:
				eval_rtf_python = 0

			
			end_program()

	   
		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			# print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			input('Press enter to exit')
			sys.exit()

	except KeyboardInterrupt:
			print('\n[ERROR] Test script for Task 3 interrupted by user!')
			end_program()
			sys.exit()

	except Exception:
		print('\nUh oh! An unknown ERROR occured.')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		end_program()
		sys.exit()


def random_setpoints():
	grid_list=[]
	for i in range(1,9):
		for j in range(1,12):
			tup=(i,j)
			grid_list.append(tup)

	# print(grid_list)
	count=0
	random_setpoint_list=[(4,5)]
	while count <3:
		random_setpoint=random.choice(grid_list)

		if(3 <= math.sqrt(math.pow(random_setpoint[0]-random_setpoint_list[-1][0], 2) + math.pow(random_setpoint[1]-random_setpoint_list[-1][1], 2)) <= 4):
			if random_setpoint not in random_setpoint_list:
				random_setpoint_list.append(random_setpoint)
				count+=1
	# print(random_setpoint_list)
	return random_setpoint_list


# Main function for testing Task 3
try:
	if __name__ == '__main__':

		print("*******************************************************************")
		print("*                                                                 *")
		print("*        =================================================        *")
		print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
		print("*        =================================================        *")
		print("*                                                                 *")
		print("*    This test suite is intended to check the output of Task 3    *")
		print("*            of Berryminator (BM) Theme (eYRC 2021-22)            *")
		print("*                                                                 *")
		print("*******************************************************************")

		try:
			team_id = int(input('\nEnter your Team ID (for e.g.: "1234" or "321"): '))

		except ValueError:
			print("\n[ERROR] Enter your Team ID which is an integer!\n")
			sys.exit()

		platform_uname = platform.uname().system

		conda_env_name = os.environ['CONDA_DEFAULT_ENV']

		expected_conda_env_name = 'BM_' + str(team_id)

		if conda_env_name == expected_conda_env_name:
			
			conda_env_name_flag = 1

		else:
			
			conda_env_name_flag = 0
			print("\n[WARNING] Conda environment name is not found as expected: BM_%s. Run this file with correct conda environment.\n" %(str(team_id)))
			sys.exit()

		if conda_env_name_flag == 1:
			
			encrypted_team_id   = cryptocode.encrypt(str(team_id), "SKDFc?rt=5_X9jb2")
			encrypted_date_time = cryptocode.encrypt(str(datetime.now()), "SKDFc?rt=5_X9jb2")
			encrypted_platform  = cryptocode.encrypt(platform_uname, "SKDFc?rt=5_X9jb2")
			encrypted_mac       = cryptocode.encrypt(str(hex(uuid.getnode())), "SKDFc?rt=5_X9jb2")


			print("Keep CoppeliaSim opened with your task_3_scene")

			input("\nPress ENTER to start Task 3 evaluation  ")
			print()


			# Generating random target points
			# target_points = [(4,0),(4,11),(2,11),(2,0)]
			# target_points = [(49,10),(32,8),(2,10),(0,0)]
			# target_points = [4,0,4,10,2,10,2,0]           # x1,y1,x2,y2...x4,y4
			# target_points = [44,0,21,10,0,10,0,0]           # x1,y1,x2,y2...x4,y4

			random_points = random_setpoints()
			enc_random_points        = cryptocode.encrypt( str(random_points), "SKDFc?rt=5_X9jb2")
			# random_points = [(4,0),(4,10),(2,10),(2,0)]

			print("Navigation co-ordinates for this run: ", random_points)

			task_3_cardinal_main(random_points)

			# print()
			# input("Press ENTER to run your task_3.py file  ")
			# print()

			# os.system("python temp.py {args[0]} {args[1]} {args[2]} {args[3]} {args[4]} {args[5]} {args[6]} {args[7]} ".format(args=(target_points)))


			score = int(output_list_child[0])
			flag_whether_signal_received = int(output_list_child[1])
			path  = output_list_child[2:]

			if flag_whether_signal_received:

				print("\nNavigation Result :")
				if score == 40:
					print("\nGood job! BM_Bot traversed all the target navigational co-ordinates.")
				elif score == 30:
					print("\nNice work! BM_Bot traversed only three of the target navigational co-ordinates.")
				elif score == 20:
					print("\nSatisfactory work! BM_Bot traversed only two of the target navigational co-ordinates.")
				elif score == 10:
					print("\n Decent work! BM_Bot traversed only one of the target navigational co-ordinates.")
				else:
					print("\nUh oh! BM_Bot failed to traverse any navigational co-ordinate.")
					print("Please try to improve your navigation logic.")

				if eval_rtf_python:
					print("\nRTF :  SATISFIED")
				else:
					print("\nRTF :  NOT SATISFIED")

			else:
				print("\n[ERROR] Something went wrong in CoppeliaSim.")
				print("Kindly rerun the exe")
				print("task_3_result.txt NOT generated")
				sys.exit()

			
			# Password = "SKDFc?rt=5_X9jb2"
			# enc_random_points        = cryptocode.encrypt( str(random_points), "SKDFc?rt=5_X9jb2")
			enc_score                = cryptocode.encrypt( str(score), "SKDFc?rt=5_X9jb2")
			enc_path                 = cryptocode.encrypt( str(path), "SKDFc?rt=5_X9jb2")	
			enc_eval_rtf_python      = cryptocode.encrypt( str(eval_rtf_python), "SKDFc?rt=5_X9jb2")
			enc_end_simulation_time  = cryptocode.encrypt( str(float(end_simulation_time)), "SKDFc?rt=5_X9jb2")
			enc_init_real_time       = cryptocode.encrypt( str(init_real_time), "SKDFc?rt=5_X9jb2")
			enc_end_real_time        = cryptocode.encrypt( str(end_real_time), "SKDFc?rt=5_X9jb2")
			enc_rtf_python           = cryptocode.encrypt( str(rtf_python), "SKDFc?rt=5_X9jb2")
		

			f = open("task_3_result.txt", "w")

			f.write(encrypted_team_id); f.write("\n")
			f.write(encrypted_date_time); f.write("\n")
			f.write(encrypted_platform); f.write("\n")
			f.write(encrypted_mac); f.write("\n")

			f.write(enc_random_points); f.write("\n")
			f.write(enc_score); f.write("\n")
			f.write(enc_path); f.write("\n")
			f.write(enc_eval_rtf_python); f.write("\n")
			f.write(enc_end_simulation_time); f.write("\n")
			f.write(enc_init_real_time); f.write("\n")
			f.write(enc_end_real_time); f.write("\n")
			f.write(enc_rtf_python); f.write("\n")

			f.close()

			print()
			print("task_3_result.txt generated")


except KeyboardInterrupt:
	print('\n[ERROR] Test script for Task 3 interrupted by user!')
	end_program()


except Exception:
	print('\n[ERROR] An Exception occurred')
	print('Stop the CoppeliaSim simulation manually if started.\n')
	traceback.print_exc(file=sys.stdout)
	print()
	end_program()
	sys.exit()