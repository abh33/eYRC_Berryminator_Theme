'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 1C         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task_1c.py
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
import json
import cv2
import numpy as np
import string
import random
import base64
from datetime import datetime
from itertools import islice
import cryptocode
import uuid
import traceback
import math
import time
from pyzbar.pyzbar import decode
from threading import Thread

# Global variable to store task_1c_score
task_1c_score = 0

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



def run_logic(client_id):
	global task_1c_score

	# Importing the sim module for Remote API connection with CoppeliaSim
	try:
		sim = __import__('sim')
		
	except Exception:
		print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
		print('\n[WARNING] Make sure to have following files in the directory:')
		print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
		sys.exit()

	try:
		task_1b = __import__('task_1b')

	except ImportError:
		print('\n[ERROR] task_1b.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_1b.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_1b.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	try:
		task_1c = __import__('task_1c')

	except ImportError:
		print('\n[ERROR] task_1c.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_1c.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_1c.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()


	time.sleep(1.5)
	task_1c.control_logic(client_id)

	return_code, score = sim.simxGetIntegerSignal( client_id, 'score', sim.simx_opmode_blocking)
	# print("Score: ", score)  

	task_1c_score = score

	try:
		return_code = task_1b.stop_simulation(client_id)                            
		if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
			print('\nSimulation stopped correctly.')
			# Stop the Remote API connection with CoppeliaSim server
												  
		else:
			print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
			print('[ERROR] stop_simulation function is not configured correctly, check the code!')
			print('Stop the CoppeliaSim simulation manually.')
											
			print()
			input('Press enter to exit')
			sys.exit()

	except Exception:
		print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		input('Press enter to exit')
		sys.exit()


def task_1c_cardinal_main():
	# Importing the sim module for Remote API connection with CoppeliaSim
	try:
		sim = __import__('sim')
		
	except Exception:
		print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
		print('\n[WARNING] Make sure to have following files in the directory:')
		print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
		sys.exit()

	try:
		task_1b = __import__('task_1b')

	except ImportError:
		print('\n[ERROR] task_1b.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_1b.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_1b.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	try:
		task_1c = __import__('task_1c')

	except ImportError:
		print('\n[ERROR] task_1c.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_1c.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_1c.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()
	
	global task_1c_score
	test_cases_passed = []
	task_1c_score = 0

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = task_1b.init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')
			# Starting the Simulation
			try:
				#  Loading dummy 
				# sim.simxLoadModel(client_id,'dummy_for_1c.ttm', 0xFF, sim.simx_opmode_blocking)
				return_code, dummy_handle = sim.simxLoadModel(client_id, resource_path("dummy_for_1c.ttm"), 0xFF, sim.simx_opmode_blocking)
				# print(return_code, dummy_handle)
				
				# # Dummy handle
				# return_code, dummy_handle = sim.simxGetObjectHandle(client_id, "Dummmy_1C", sim.simx_opmode_blocking)
				# print(return_code)
				# print(dummy_handle)

				return_code = task_1b.start_simulation(client_id)

				if (return_code == sim.simx_return_novalue_flag):
					print('\nSimulation started correctly in CoppeliaSim.')

				else:
					print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
					print('start_simulation function is not configured correctly, check the code!')
					print()

			except Exception:
				print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
	   
		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			input('Press enter to exit')
			sys.exit()

	except Exception:
		print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		input('Press enter to exit')
		sys.exit()

	try:
		#print("Main Client_ID : "+str(client_id))
		t1 = Thread(target = run_logic,args = (client_id,))
		# t2 = Thread(target = evaluation_script,args = (client_id,))
		t1.start()
		# t2.start()
		t1.join()
		# t2.join()

		# Closing the scene
		sim.simxCloseScene( client_id, sim.simx_opmode_blocking)
		# return_code = sim.simxRemoveModel( client_id, dummy_handle, sim.simx_opmode_blocking)
		# print(return_code)
		try:

			task_1b.exit_remote_api_server(client_id)
			if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
				print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')
				# input('Press enter to exit')
				# sys.exit()
				   
			else:
				print('\n[ERROR] Failed disconnecting from Remote API server!')
				print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

		except Exception:
			print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			input('Press enter to exit')
			sys.exit()
   
	except Exception:
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		input('Press enter to exit')
		sys.exit()


	return task_1c_score


if __name__ == '__main__':

	print("*******************************************************************")
	print("*                                                                 *")
	print("*        =================================================        *")
	print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
	print("*        =================================================        *")
	print("*                                                                 *")
	print("*   This test suite is intended to check the output of Task 1C    *")
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
		
		# #remove the previously generated output csv file if exists
		# if os.path.exists(output_csv_file_name):
		# 	os.remove(output_csv_file_name)
		encrypted_team_id = cryptocode.encrypt(str(team_id), "eyrc_berryminator_2021-22")
		encrypted_date_time = cryptocode.encrypt(str(datetime.now()), "eyrc_berryminator_2021-22")
		encrypted_platform = cryptocode.encrypt(platform_uname, "eyrc_berryminator_2021-22")
		encrypted_mac = cryptocode.encrypt(str(hex(uuid.getnode())), "eyrc_berryminator_2021-22")

		print()
		input("Press ENTER to start Task 1C evaluation  ")
		print()
		# print("\nPlease select the sub-task for which you want to test the output")
		# print("1. Task 1A")
		# print("2. Task 1B")
		# print("3. Task 1C")

		# try:
		# 	task_choice = int(input('\nPlease enter your choice from the above options (1/2/3): '))

		# except ValueError:
		# 	print("\n[ERROR] The choice should be an integer!\n")
		# 	sys.exit()

		# if task_choice >= 1 and task_choice <= 3:
		# 	valid_choice_flag = 1
		# else:
		# 	valid_choice_flag = 0
		# 	print("\n[WARNING] Choice is not valid. Please enter 1 or 2 or 3\n")
		# 	sys.exit()

		valid_choice_flag = 1
		task_choice = 3
		if valid_choice_flag == 1:
			if task_choice == 3:
				task_1c_score = task_1c_cardinal_main()
				if task_1c_score == 4:
					print("\nThe robot is completing all the checkpoints !")
					print("\nTask 1C completed successfully !!")
				else:
					print("The robot is not completing one or more of the checkpoints")
					print("Please debug your code !")
				task_1c_score_scaled = task_1c_score*10
				encrypted_score = cryptocode.encrypt(str(task_1c_score_scaled), "eyrc_berryminator_2021-22")

				f = open("task_1c_result.txt", "w")
				f.write(encrypted_team_id); f.write("\n")
				f.write(encrypted_date_time); f.write("\n")
				f.write(encrypted_platform); f.write("\n")
				f.write(encrypted_mac); f.write("\n")

				# for test_case in test_cases_passed:
				# 	encrypted_test_case = cryptocode.encrypt(test_case, "eyrc_berryminator_2021-22")
				# 	f.write(encrypted_test_case); f.write("\n")
				# encrypted_score = cryptocode.encrypt(str(task_1c_score), "eyrc_berryminator_2021-22")
				f.write(encrypted_score); f.write("\n")
				f.close()
				print("\nTask 1C result text file generated.")