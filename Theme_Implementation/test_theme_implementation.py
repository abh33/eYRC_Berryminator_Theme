'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 4         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task_4.py
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
import gspread

global output_list_child, output_list_custom, rtf_python, init_real_time, end_real_time, end_simulation_time
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

# Headers to be added in the 1st row of the Google Sheet.
headers=[
'Date and Time',
'Platform',
'Mac',
'Correctly Identified',
'Correctly Plucked',
'Correctly Dropped',
'Collisions',
'Mass of Arm',
'Total simulation time',
'RTF',
'No. of joints',
'Torque',
'Force'
]

# Credentials required to communicate with Google Sheets.
# https://docs.gspread.org/en/latest/index.html
axebnfgh={
  "type": "service_account",
  "project_id": "bm-eyrc21-task4",
  "private_key_id": "0311b6ddeca15448ddb7a296e2538b39c6935c9b",
  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQCnL2a7fnkjYLph\n/azJAuMECrhCLRl3N1M4Axit1eac3Qr8IyyYVMcGWZuTSKr/HPuKcHL5EKmNt7AL\nfYGI9Qf9IM+/F7JF3sZORZj1ajHijVrqy7isO84r8Xa6NgUV/115q1jgpTWnajwq\n6IhQKlfRAXCxgfY0byhdckJejtH7PQqFU6N/SAO68MLa+fuSR9IyuM1/36ehvQ3W\nkJuqHCqEUa/MG1RfuPVMlwOmx7rd+v7NV/etU1s459vBvuSqqa6S1SvH8TaFCWEG\nEIDXcE0UsyPeO5iH4s9X2udx4ANy1tvpYgmuV8u2Lw1YmN9ds8ujQIeUR8tNS3tw\nzf8/dKNLAgMBAAECggEADhgIDuinuVdHWckYXq5lyT/xbka+CFemJikrVJrygNPT\nlWjjsrMm6hP2HRGkX12n85uXiFlNaeXhKz7Og5ILNTLQa07J7RZNh74Emd/KH8wE\ngkRf/PO3uur1XtenUHDsMOcZ1tT788aYFufudzpH7UCJVGAyi2i5huiBzCs5wXZY\nzGiNela+5XiHaMyZZTk4LbboHtb86X7tXBJRW2ckXDJq1S51fsbsHoFs281v2r+5\nGmzg7U7K8T91wcOk27qmVPr53GEcut1vcwwGjC1cjySRdCAjUVfxygu5Khrw8Ax1\nMBT2McYl3ETUaEAdbhotEnI3bZxqQTLmJFcw4y9TWQKBgQDZYWMiPYdV2BuxpDkP\nScyJdJiXTl1Emi7J1C4/hr6jrZdp2uTvm/DidmXPp3mCBT5AHo6sUZVwsEQOkUT0\nDSu2v0SaTn7oao838SIaZqvCngh5wXfzpg/uD2jLUE70DqaJWIZ7rmOQzqTmh2Zq\n2V1lKRpSGYX1/CQ8othV5GIzaQKBgQDE4xlLNe0yjVhrq5EePedkIWWzVg5Nibi/\nS27L+dMvdryqdtE10qy7dx+l4Zcf9pMAtHyLlFjKiWS63lGs/ku6oiX2seScjT6Q\nsrkGLtnWPRCXj6iOMZRV25IIlc3OEMHqXI1zKv4EKXuWDx3hlxFf2Gwi2EtLwznb\nUTb6M+pukwKBgCwBZgMhd4suQ3TzrMVmiCxvWRCQQZtIpx4egPpFOcSCuLsqH9xN\nsjb2BuhBm/xxOUfbp8BEh78XP7+/MrlBDU9iau3d5B0CrVmCzAeSL0UnnsmfG7xM\nKr9jiTBXYpRPfvEg5aCWDfvbiVBNqrMIq7p11qk2F4NTdrZ7tNW2dRa5AoGBALmj\nqzi4CwNsNF+o7/aExcUqnDrAvFGttm1gEYLv9oi9Oty7lT1bfBRyGHwOvbCcjDTQ\n+NGeuS2W6EwtzPMA/qRluMfnKbfWkZRbf4tt52VSRG+zO2l/I+7kTcgcf2V5dW2l\ngJWDhuijuGxfqnwHNZGafTNJn4068TVyWRUGmFdTAoGBAIDfLGFDRWLiE1VNYD+w\nf/T2k+mR5PNJucQdH73Rs0FiJEBVGZJBKKEnnbVCxdl4C3luG4Lf+LCtR8qyVtCV\nVxh/NTar8ruAB4C0AJBg2Rm5l+vRFwIqdhd6CsbiukTwhx6gZkagF3F/tdn27fxt\n/XDydNOoJDKTbpkPB0TlX2YE\n-----END PRIVATE KEY-----\n",
  "client_email": "eyrc21-22bmtask5@bm-eyrc21-task4.iam.gserviceaccount.com",
  "client_id": "100801905280618216431",
  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
  "token_uri": "https://oauth2.googleapis.com/token",
  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/eyrc21-22bmtask5%40bm-eyrc21-task4.iam.gserviceaccount.com"
}


def send_data_e_yantra_server():

	submission_attempts=0

	flag_data_sent_successfully=0 # 0- Failed sending data, 1- Successfully sent data OR Master Flag in the sheet is 0 (Can be done manually ONLY). 

	while submission_attempts<5:
		print('\n#################################################################')
		print('\nAttempting to send data to e-Yantra Servers.')
		print('\nMake sure you have an active internet connection.')

		try:
			service_acc = gspread.service_account_from_dict(axebnfgh)
			g_sheet=service_acc.open('Real_Time_Performance_of_Teams_in_Task_5')
			worksheet_list = g_sheet.worksheets()
			master_wksheet = g_sheet.worksheet('master')		
			if(master_wksheet.acell('Z1').value=='1'):

				list_of_worksheets=[]

				for obj in worksheet_list:
					list_of_worksheets.append(obj._properties['title'])


				if(str(team_id) not in list_of_worksheets):
					wksheet = g_sheet.add_worksheet(title=str(team_id), rows="10000", cols="20")
					wksheet.update('A1:M1',[headers]) #Add header row for the first time.
				else:
					wksheet = g_sheet.worksheet(str(team_id))


				row_num_to_write=len(wksheet.get_all_values())+1 # This will give number of rows which are filled.

				data_to_push=[curr_date_time,platform_uname,str(hex(uuid.getnode())),str(ci_list_to_show),str(pluck_list_to_show),
								str(dropped_list_to_show),str(len(collisions_list_to_show)),str(mass_of_arm),str(float(end_simulation_time)),
									str(rtf_python),str(no_of_joints),str(torque),str(force)]

				wksheet.update('A'+str(row_num_to_write)+':M'+str(row_num_to_write),[data_to_push])

				if(len(wksheet.get_all_values())+1==row_num_to_write+1): # Verifying if the row is filled or not.
					print("\nSuccessfully sent data to e-Yantra Servers.")
					print('\n#################################################################')
					submission_attempts=5
					flag_data_sent_successfully=1

			else:
				submission_attempts=5
				flag_data_sent_successfully=1
				print('\nSubmission to server is not available now.')
				print('\n#################################################################')
		
		except:
			submission_attempts+=1
			flag_data_sent_successfully=0
			print('\nFAILED sending the data.')
			print('\nNumber of times remaining: ',5-submission_attempts)
			input('\nPress any key to try again.')

	return flag_data_sent_successfully

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
	return_code, vs_1_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	return_code, vs_2_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	return_code, nameplate_handle = sim.simxGetObjectHandle(client_id, 'name_plate', sim.simx_opmode_blocking)
	return_code, force_sensor_br_handle = sim.simxGetObjectHandle(client_id, 'force_sensor_br', sim.simx_opmode_blocking)
	return_code, camera_handle = sim.simxGetObjectHandle(client_id, 'DefaultCamera', sim.simx_opmode_blocking)
	return_code, BM_Bot_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot', sim.simx_opmode_blocking)

	# Getting parent of VS 2
	returnCode, vs_2_parent_handle = sim.simxGetObjectParent( client_id, vs_2_handle, sim.simx_opmode_blocking)

	# Before removing their BM_Bot model, setting its position to (0,0) and orientation
	# return_code = sim.simxSetObjectPosition( client_id, BM_Bot_handle, -1 , [ 0, 0, 4.8029e-02], sim.simx_opmode_blocking)
	# return_code = sim.simxSetObjectOrientation( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)
	return_code = sim.pougadq( client_id, BM_Bot_handle, -1 , [ 0, 0, 4.8029e-02], sim.simx_opmode_blocking)
	return_code = sim.ghjvhjvj( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)

	# Making Vision sensors, name plate, fs_r parentless
	return_code = sim.simxSetObjectParent( client_id, vs_1_handle, -1, True, sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, vs_2_handle, -1, True, sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, nameplate_handle, -1, True, sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, force_sensor_br_handle, -1, True, sim.simx_opmode_blocking)

	# Removing their BM_Bot
	return_code = sim.simxRemoveModel( client_id, BM_Bot_handle, sim.simx_opmode_blocking)

	# Adding our BM_Bot, by default at (0,0) which is what we required
	return_code, BM_Bot_handle = sim.simxLoadModel( client_id, resource_path('BM_Bot_general.ttm'), 0, sim.simx_opmode_blocking) #Load the new model

	# Making BM_Bot_base as parent of vision_sensor_1 and name plate
	return_code, BM_Bot_base_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot_base', sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, vs_1_handle, BM_Bot_base_handle, True, sim.simx_opmode_blocking)
	return_code = sim.simxSetObjectParent( client_id, nameplate_handle, BM_Bot_base_handle, True, sim.simx_opmode_blocking)

	# Making BM_Bot as parent of fs_br
	return_code = sim.simxSetObjectParent( client_id, force_sensor_br_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

	# Making BM_Bot as parent of DefaultCamera
	return_code = sim.simxSetObjectParent( client_id, camera_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

	# Making VS 2 child of its original parent object
	return_code = sim.simxSetObjectParent( client_id, vs_2_handle, vs_2_parent_handle, True, sim.simx_opmode_blocking)


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
		return_code, dummy_handle = sim.simxGetObjectHandle(client_id, 'eval_bm', sim.simx_opmode_blocking)
		if(return_code == 0): #This means that the object exists from before
			return_code = sim.simxRemoveModel( client_id, dummy_handle, sim.simx_opmode_blocking)
		
		#cwd = os.getcwd()
		#return_code,evaluation_screen_handle=sim.simxLoadModel(client_id,cwd+'//evaluation_projector_screen.ttm',0,sim.simx_opmode_blocking) #Load the new model
		return_code, dummy_handle = sim.simxLoadModel( client_id, resource_path('task_4_dummy.ttm'), 0, sim.simx_opmode_blocking) #Load the new model
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

	return_code, disc_handle = sim.simxGetObjectHandle(client_id, 'eval_bm', sim.simx_opmode_blocking)

	if (return_code == 0):				# This means that the object exists from before
		inputBuffer = bytearray()
		return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
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
	return_code, retInts, retFloats, data_child, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
									sim.sim_scripttype_childscript,'get_required_data_child',[],[],[],inputBuffer,sim.simx_opmode_blocking)
	# print("data_child:")
	# print(data_child)

	return data_child


def get_custom_data():
	global client_id

	inputBuffer = bytearray()
	return_code, retInts, retFloats, data_custom, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
									sim.sim_scripttype_customizationscript,'get_required_data_custom',[],[],[],inputBuffer,sim.simx_opmode_blocking)

	return data_custom,return_code


def general_berry_name(argument):

	if (argument.find('lemon')==0):
		return "Lemon"
	elif(argument.find('strawberry')==0):
		return "Strawberry"
	elif(argument.find('blueberry')==0):
		return "Blueberry"
	else:
		return "None"

def unique(list1):
 
	# initialize a null list
	unique_list = []
	 
	# traverse for all elements
	for x in list1:
		# check if exists in unique_list or not
		if x not in unique_list:
			unique_list.append(x)
	
	return unique_list																															


def task_4_cardinal_main():

	global output_list_child, output_list_custom, rtf_python, eval_rtf_python, init_real_time, end_real_time, end_simulation_time

	try:
		task_4 = __import__('task_4')

	except ImportError:
		print('\n[ERROR] task_4.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_4.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_4.py threw an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()


	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Students should have opened task_4_scene.ttt

			# Loading our task 4 dummy model
			load_eval_model()

			# Check scene
			wait_for_check_scene()

			# # Replacing BM_Bot
			replacement()

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
				task_4.task_4_primary(client_id)

			except KeyboardInterrupt:
				print('\n[ERROR] Test script for Task 4 interrupted by user!')
				#end_program()			

			except Exception:
				print('\n[ERROR] Your task_4_primary() function threw an error.')
				print('Kindly debug your code.')
				traceback.print_exc(file=sys.stdout)
				print()
				end_program()
				sys.exit()

			# Getting data from child script
			output_list_child = get_child_data()
			p = ",".join(output_list_child)
			output_list_child = p.split(',')

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


			# Getting data from custom script
			output_list_custom,return_code = get_custom_data()
			
			if(return_code!=0):
				sys.exit()

			p = ",".join(output_list_custom)
			output_list_custom = p.split(',')
			# print('================================')
			# print(output_list_custom)
			# print('================================')
			
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
			print('\n[ERROR] Test script for Task 4 interrupted by user!')
			end_program()
			sys.exit()

	except Exception:
		print('\nUh oh! An unknown ERROR occured.')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		end_program()
		sys.exit()


# Main function for testing Task 4
try:
	if __name__ == '__main__':

		print("*******************************************************************")
		print("*                                                                 *")
		print("*        =================================================        *")
		print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
		print("*        =================================================        *")
		print("*                                                                 *")
		print("*    This test suite is intended to check the output of Task 4    *")
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
			
			curr_date_time=str(datetime.now())
			encrypted_team_id   = cryptocode.encrypt(str(team_id), "?8G]tzGW{T")
			encrypted_date_time = cryptocode.encrypt(curr_date_time, "?8G]tzGW{T")
			encrypted_platform  = cryptocode.encrypt(platform_uname, "?8G]tzGW{T")
			encrypted_mac       = cryptocode.encrypt(str(hex(uuid.getnode())), "?8G]tzGW{T")


			print("Keep CoppeliaSim opened with your task_4_scene")

			input("\nPress ENTER to start Task 4 evaluation  ")
			print()

			task_4_cardinal_main()

			index = 0
			if output_list_child[index] != "CI":
				print("The data from the simulation got corrupted.")
				print("Kindly rerun the exe")
				print("task_4_result.txt NOT generated")
				sys.exit()

			index = 1

			ci_temp = []
			ci_list_to_show = []
			while output_list_child[index] != "Plucked":
				ci_temp.append(output_list_child[index])
				index += 1
			
			ci_temp = unique(ci_temp)

			for i in ci_temp:
				berry_name = general_berry_name(i)
				ci_list_to_show.append(berry_name)


			index += 1
			pluck_temp = []
			pluck_list_to_show = []
			while output_list_child[index] != "Dropped":
				pluck_temp.append(output_list_child[index])
				index += 1
			
			pluck_temp = unique(pluck_temp)

			for i in pluck_temp:
				berry_name = general_berry_name(i)
				pluck_list_to_show.append(berry_name)


			index += 1
			dropped_temp = []
			dropped_list_to_show = []
			while output_list_child[index] != "Collisions":
				dropped_temp.append(output_list_child[index])
				index += 1
			
			dropped_temp = unique(dropped_temp)

			for i in dropped_temp:
				berry_name = general_berry_name(i)
				dropped_list_to_show.append(berry_name)


			index += 1
			# collision_temp = []
			collisions_list_to_show = []
			while output_list_child[index] != "Eval_Dyn":
				collisions_list_to_show.append(output_list_child[index])
				index += 1

			index += 1
			eval_all_dynamics = output_list_child[index]

			index += 2
			mass_of_arm = output_list_child[index]			
			index += 2

			# path = output_list_child[index:]
			path = []
			while output_list_child[index] != "Dyn":
				path.append(output_list_child[index])
				index += 1


			eval_no_of_joints = output_list_custom[0]
			no_of_joints      = output_list_custom[1]
			torque            = output_list_custom[2]
			force             = output_list_custom[3]
			individual_values = output_list_custom[4:]


			print("\n**** Task 4 Result ****\n")
			print("Correctly Identified : ", ci_list_to_show)
			print("Correctly Plucked    : ", pluck_list_to_show)
			print("Correctly Dropped    : ", dropped_list_to_show)
			
			# # Password = "?8G]tzGW{T"
			encrypted_ci           = cryptocode.encrypt(str(ci_list_to_show), "?8G]tzGW{T")
			encrypted_cp           = cryptocode.encrypt(str(pluck_list_to_show), "?8G]tzGW{T")
			encrypted_cd           = cryptocode.encrypt(str(dropped_list_to_show), "?8G]tzGW{T")
			encrypted_collisions   = cryptocode.encrypt(str(collisions_list_to_show), "?8G]tzGW{T")
			encrypted_mass_of_arm   = cryptocode.encrypt(str(mass_of_arm), "?8G]tzGW{T")
			encrypted_path         = cryptocode.encrypt(str(path), "?8G]tzGW{T")
			encrypted_output_list_child   =  cryptocode.encrypt(str(output_list_child), "?8G]tzGW{T")

			encrypted_eval_rtf_python      = cryptocode.encrypt( str(eval_rtf_python), "?8G]tzGW{T")
			encrypted_end_simulation_time  = cryptocode.encrypt( str(float(end_simulation_time)), "?8G]tzGW{T")
			encrypted_init_real_time       = cryptocode.encrypt( str(init_real_time), "?8G]tzGW{T")
			encrypted_end_real_time        = cryptocode.encrypt( str(end_real_time), "?8G]tzGW{T")
			encrypted_rtf_python           = cryptocode.encrypt( str(rtf_python), "?8G]tzGW{T")

			encrypted_eval_all_dynamics    = cryptocode.encrypt( str(eval_all_dynamics), "?8G]tzGW{T")
			encrypted_eval_no_of_joints    = cryptocode.encrypt( str(eval_no_of_joints), "?8G]tzGW{T")
			encrypted_no_of_joints         = cryptocode.encrypt( str(no_of_joints), "?8G]tzGW{T")
			encrypted_torque               = cryptocode.encrypt( str(torque), "?8G]tzGW{T")
			encrypted_force                = cryptocode.encrypt( str(force), "?8G]tzGW{T")
			encrypted_individual_values    = cryptocode.encrypt( str(individual_values), "?8G]tzGW{T")
			
			#Used to send data to e-Yantra Server (A google sheet)
			return_value=send_data_e_yantra_server()
			
			if(return_value==1):
				f = open("task_4_result.txt", "w")

				f.write(encrypted_team_id); f.write("\n")
				f.write(encrypted_date_time); f.write("\n")
				f.write(encrypted_platform); f.write("\n")
				f.write(encrypted_mac); f.write("\n")

				f.write(encrypted_ci); f.write("\n")
				f.write(encrypted_cp); f.write("\n")
				f.write(encrypted_cd); f.write("\n")
				f.write(encrypted_collisions); f.write("\n")
				f.write(encrypted_mass_of_arm); f.write("\n")
				f.write(encrypted_path); f.write("\n")
				f.write(encrypted_output_list_child); f.write("\n")

				f.write(encrypted_eval_rtf_python); f.write("\n")
				f.write(encrypted_end_simulation_time); f.write("\n")
				f.write(encrypted_init_real_time); f.write("\n")
				f.write(encrypted_end_real_time); f.write("\n")
				f.write(encrypted_rtf_python); f.write("\n")

				f.write(encrypted_eval_all_dynamics); f.write("\n")
				f.write(encrypted_eval_no_of_joints); f.write("\n")
				f.write(encrypted_no_of_joints); f.write("\n")
				f.write(encrypted_torque); f.write("\n")
				f.write(encrypted_force); f.write("\n")
				f.write(encrypted_individual_values); f.write("\n")

				f.close()

				print("\n-----------------------------------------------------------")
				print("\n Way to go "+str(team_id)+"! 'task_4_result.txt' generated successfully.")
				print("\n-----------------------------------------------------------")
			else:
				print('FALIED generating task_4_result.txt. Please try again.')

except KeyboardInterrupt:
	print('\n[ERROR] Test script for Task 4 interrupted by user!')
	end_program()


except Exception:
	print('\n[ERROR] An Exception occurred')
	print('Stop the CoppeliaSim simulation manually if started.\n')
	traceback.print_exc(file=sys.stdout)
	print()
	end_program()
	sys.exit()