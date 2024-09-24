'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 2B         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task2b.py
*  Created:				
*  Last Modified:		1
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
import os
import sys
import platform
import csv
import numpy as np
import string
import random
from datetime import datetime
from itertools import islice
import cryptocode
import uuid
import traceback
import math
import time



global output_list_child_eval, output_list_custom_eval, output_list_child, output_list_custom, rtf_python
eval_rtf_python = 0
position = [0, 0, 0]

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


def get_customization_data():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, data_customization, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
									sim.sim_scripttype_customizationscript,'get_required_data_custom',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("eval_data_customization:")
	# print(eval_data_customization)

	return data_customization

def get_child_data():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, data_child, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
									sim.sim_scripttype_childscript,'get_required_data_child',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("eval_data_child:")
	# print(eval_data_child)

	return data_child


def get_customization_data_for_eval():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, eval_data_customization, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
									sim.sim_scripttype_customizationscript,'get_required_data_custom_eval',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("eval_data_customization:")
	# print(eval_data_customization)

	return eval_data_customization


def get_child_data_for_eval():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, eval_data_child, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
									sim.sim_scripttype_childscript,'get_required_data_child_eval',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("eval_data_child:")
	# print(eval_data_child)

	return eval_data_child


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
		return_code, disc_handle = sim.simxGetObjectHandle(client_id, 'Disc_BM_2B', sim.simx_opmode_blocking)
		if(return_code == 0): #This means that the object exists from before
			return_code, arm_handle = sim.simxGetObjectHandle( client_id, 'robotic_arm', sim.simx_opmode_blocking)
			return_code = sim.simxSetObjectParent( client_id, arm_handle, -1, True, sim.simx_opmode_blocking)
			return_code = sim.simxRemoveModel( client_id, disc_handle, sim.simx_opmode_blocking)
		#cwd = os.getcwd()
		#return_code,evaluation_screen_handle=sim.simxLoadModel(client_id,cwd+'//evaluation_projector_screen.ttm',0,sim.simx_opmode_blocking) #Load the new model
		return_code, disc_handle = sim.simxLoadModel( client_id, resource_path('discforce_task_2b.ttm'), 0, sim.simx_opmode_blocking) #Load the new model
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
	global client_id


	return_code = stop_simulation()

	return_code, disc_handle = sim.simxGetObjectHandle(client_id, 'Disc_BM_2B', sim.simx_opmode_blocking)

	if (return_code == 0):				# This means that the object exists from before
		return_code, arm_handle = sim.simxGetObjectHandle( client_id, 'robotic_arm', sim.simx_opmode_blocking)
		if return_code == 0:
			return_code = sim.simxSetObjectParent( client_id, arm_handle, -1, True, sim.simx_opmode_blocking)
			returnCode = sim.simxSetObjectPosition( client_id, arm_handle, -1, position, sim.simx_opmode_blocking)
		inputBuffer = bytearray()
		return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
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


def task_2b_cardinal_main():

	global output_list_child_eval, output_list_custom_eval, position, client_id, output_list_child, output_list_custom
	global rtf_python, eval_rtf_python

	# # Importing the sim module for Remote API connection with CoppeliaSim
	# try:
	# 	sim = __import__('sim')
		
	# except Exception:
	# 	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	# 	print('\n[WARNING] Make sure to have following files in the directory:')
	# 	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	# 	sys.exit()


	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')


			# Students should have opened task_2b_scene.ttt with robotic arm present in it

			# Getting arm's default position
			return_code, arm_handle = sim.simxGetObjectHandle( client_id, 'robotic_arm', sim.simx_opmode_blocking)
			if return_code != sim.simx_return_ok:
				print("[ERROR] Couldn't find robotic_arm in opened scene")
				print("Make sure you have opened task_2b_scene.ttt which contains your robotic_arm")
				end_program()
				sys.exit()

			# Save original position of robotic arm 	
			returnCode, position = sim.simxGetObjectPosition( client_id, arm_handle, -1, sim.simx_opmode_blocking) 
			
			# Loading our disc force model
			load_eval_model()

			wait_for_check_scene()

			init_real_time = time.time()


			#  Start simulation
			return_code = start_simulation()

			if (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation started correctly in CoppeliaSim.')

			else:
				print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
				end_program()
				sys.exit()


			# Waiting for 1 simulation sec so that at least one time step is simulated
			# sleep(1.)
			return_code, init_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
			# if return_code != sim.simx_return_ok:   # NOT SURE ABOUT THIS CONDITION
			# 	print("[ERROR] MAIN Script present in the task_2b_scene has been modifed.")
			# 	print(("Download the scene file again."))
			# 	end_program()
			# 	sys.exit()

			check = 0
			while check <= 1.0:
				return_code, new_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
				if return_code == sim.simx_return_remote_error_flag:
					print("[ERROR] Task 2B scene main script was tampered. Exiting ...")
					end_program()
					sys.exit()
				if new_sim_time == '':
					new_sim_time = '0'
				check = float(new_sim_time)


			# AFTER STARTING SIMULATION, CHECKING DYNAMICS AND MASS
			inputBuffer = bytearray()
			return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc_BM_2B',\
											sim.sim_scripttype_childscript,'dynamics_and_mass_check',[],[],[],inputBuffer,sim.simx_opmode_blocking)

			
			# Delay for stabilizing RTF and to check stability while evaluating 
			# 10 secs
			return_code, init_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
			check = 0
			while check <= 10.0:
				return_code, new_sim_time = sim.simxGetStringSignal( client_id, 'time', sim.simx_opmode_blocking)
				# print(return_code)
				if return_code == sim.simx_return_remote_error_flag:
					print("\n[ERROR] It seems simulation was stopped in between. Exiting ...")
					end_program()
					sys.exit()
				check = float(new_sim_time) - float(init_sim_time)


			# Getting data from child script
			data_child = get_child_data()
			p = ",".join(data_child)
			output_list_child = p.split(',')

			# Getting EVAL data from child script
			eval_data_child = get_child_data_for_eval()
			p = ",".join(eval_data_child)
			output_list_child_eval = p.split(',')

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



			# This delay is necessary otherwise RTF is not received correctly
			# time.sleep(0.1)
			

			# Getting data from customization script
			data_customization = get_customization_data()
			p = ",".join(data_customization)
			output_list_custom = p.split(',')

			# Getting EVAL data from customization script
			eval_data_customization = get_customization_data_for_eval()
			p = ",".join(eval_data_customization)
			output_list_custom_eval = p.split(',')

			# This function will stop the simulation, call organize_screen_end, clear string signals, exit the server
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
			print('\n[ERROR] Test script for Task 2B interrupted by user!')
			end_program()
			sys.exit()

	except Exception:
		print('\nUh oh! An unknown ERROR occured.')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		end_program()
		sys.exit()


# Main function for testing Task 2B
try:
	if __name__ == '__main__':

		print("*******************************************************************")
		print("*                                                                 *")
		print("*        =================================================        *")
		print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
		print("*        =================================================        *")
		print("*                                                                 *")
		print("*    This test suite is intended to check the output of Task 2B   *")
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
			
			encrypted_team_id = cryptocode.encrypt(str(team_id), "13_madison_kingdom")
			encrypted_date_time = cryptocode.encrypt(str(datetime.now()), "13_madison_kingdom")
			encrypted_platform = cryptocode.encrypt(platform_uname, "13_madison_kingdom")
			encrypted_mac = cryptocode.encrypt(str(hex(uuid.getnode())), "13_madison_kingdom")


			print("Keep CoppeliaSim opened with your task_2b_scene")

			print()
			input("Press ENTER to start Task 2B evaluation  ")
			print()

			task_2b_cardinal_main()

			eval_no_of_joints = int(output_list_custom_eval[0])
			eval_vol          = int(output_list_custom_eval[1])
			torque            = output_list_custom_eval[2]
			force             = output_list_custom_eval[3]
			eval_rtf          = eval_rtf_python                                   # int(output_list_custom_eval[4])
			individual_values = output_list_custom_eval[5:]

			eval_all_dynamics = int(output_list_child_eval[0])
			eval_mass         = int(output_list_child_eval[1])

			no_of_joints      = output_list_custom[0]
			vol               = output_list_custom[1]
			rtf               = rtf_python                                        # output_list_custom[2]
			mass              = output_list_child[0]
			dynamics_not_enabled_list = output_list_child[1:]


			# print(eval_no_of_joints)
			# print(eval_vol)
			# print(eval_rtf)
			# print(eval_all_dynamics)
			# print(eval_mass)

			print("\nChecking Constraints\n")
			if eval_no_of_joints:
				print("No. of joints          :    SATISFIED")
			else:
				print("No. of joints          :    NOT SATISFIED")

			if eval_vol:
				print("Bounding Box Volume    :    SATISFIED")
			else:
				print("Bounding Box Volume    :    NOT SATISFIED")

			print("Total Torque Required  :   ", torque, " Nm")
			print("Total Force Required   :   ", force, " N")
			print("Torque/ Force required for individual joints in order: ", individual_values)
			
			if eval_rtf:
				print("RTF                    :    SATISFIED")
			else:
				print("RTF                    :    NOT SATISFIED")

			# print(eval_all_dynamics)
			if eval_all_dynamics:
				print("Dynamics enabled       :    SATISFIED")
			else:
				print("Dynamics enabled       :    NOT SATISFIED")

			if eval_mass:
				print("Total Mass             :    SATISFIED")
			else:
				print("Total Mass             :    NOT SATISFIED")
			
			

			# Password = "13_madison_kingdom"
			enc_eval_no_of_joints = cryptocode.encrypt( str(eval_no_of_joints), "13_madison_kingdom")
			enc_eval_vol          = cryptocode.encrypt( str(eval_vol), "13_madison_kingdom")
			enc_torque            = cryptocode.encrypt( str(torque), "13_madison_kingdom")
			enc_force             = cryptocode.encrypt( str(force), "13_madison_kingdom")
			enc_individual_values = cryptocode.encrypt( str(individual_values), "13_madison_kingdom")
			enc_eval_rtf          = cryptocode.encrypt( str(eval_rtf), "13_madison_kingdom")

			enc_eval_all_dynamics = cryptocode.encrypt( str(eval_all_dynamics), "13_madison_kingdom")
			enc_eval_mass         = cryptocode.encrypt( str(eval_mass), "13_madison_kingdom")

			enc_no_of_joints      = cryptocode.encrypt( str(no_of_joints), "13_madison_kingdom")
			enc_vol               = cryptocode.encrypt( str(vol), "13_madison_kingdom")
			enc_rtf               = cryptocode.encrypt( str(rtf), "13_madison_kingdom")
			enc_mass              = cryptocode.encrypt( str(mass), "13_madison_kingdom")

			enc_dynamics_not_enabled_list = cryptocode.encrypt( str(dynamics_not_enabled_list), "13_madison_kingdom")


			f = open("task_2b_result.txt", "w")

			f.write(encrypted_team_id); f.write("\n")
			f.write(encrypted_date_time); f.write("\n")
			f.write(encrypted_platform); f.write("\n")
			f.write(encrypted_mac); f.write("\n")

			f.write(enc_eval_no_of_joints); f.write("\n")
			f.write(enc_eval_vol); f.write("\n")
			f.write(enc_torque); f.write("\n")
			f.write(enc_force); f.write("\n")
			f.write(enc_individual_values); f.write("\n")
			f.write(enc_eval_rtf); f.write("\n")
			f.write(enc_eval_all_dynamics); f.write("\n")
			f.write(enc_eval_mass); f.write("\n")
		
			f.write(enc_no_of_joints); f.write("\n")
			f.write(enc_vol); f.write("\n")
			f.write(enc_rtf); f.write("\n")
			f.write(enc_mass); f.write("\n")

			f.write(enc_dynamics_not_enabled_list); f.write("\n")

			f.close()

			print()
			print("task_2b_result.txt generated")


except KeyboardInterrupt:
	print('\n[ERROR] Test script for Task 2B interrupted by user!')
	end_program()


except Exception:
	print('\n[ERROR] An Exception occurred')
	print('Stop the CoppeliaSim simulation manually if started.\n')
	traceback.print_exc(file=sys.stdout)
	print()
	end_program()
	sys.exit()