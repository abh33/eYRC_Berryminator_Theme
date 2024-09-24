'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 1B         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task1.py
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

def get_qr_block_handles(client_id):
	qr_blocks = ["qr_block_1", "qr_block_2", "qr_block_3", "qr_block_4", "qr_block_5", "qr_block_6"]
	qr_block_handles = {}

	# Importing the sim module for Remote API connection with CoppeliaSim
	try:
		sim = __import__('sim')
		
	except Exception:
		print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
		print('\n[WARNING] Make sure to have following files in the directory:')
		print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
		sys.exit()

	for qr_block in qr_blocks:
		try:
			return_code, handle_num = sim.simxGetObjectHandle(client_id, qr_block, sim.simx_opmode_blocking)
			if return_code == sim.simx_return_ok:
				qr_block_handles[qr_block] = handle_num
			else:
				print('\n[ERROR] ' + qr_block + ' not detected in scene. Make sure all 6 qr_blocks are present in the coppeliasim scene')
				print('Stop the CoppeliaSim simulation manually.\n')
				print()
				sys.exit()


		except Exception:
					print('\n[ERROR] Exception thrown in calling simxGetObjectHandle.')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()
	
	return qr_block_handles

def check_format(qr_codes_list):
	format_check_flag = True
	if (isinstance(qr_codes_list, list)):
		if len(qr_codes_list) == 0:
			format_check_flag = (format_check_flag and False)
			print('\n[ERROR] qr_codes_list is empty')
		else:
			for qr_code in qr_codes_list:
				if(isinstance(qr_code, list)):
					if len(qr_code) == 2:
						if (isinstance(qr_code[0], str)) and (isinstance(qr_code[1], tuple)):
							format_check_flag = (format_check_flag and True)
						else:
							format_check_flag = (format_check_flag and False)
							print('\n[ERROR] Individual elements of qr code list should be string and tuple respectively. Debug your code')
					else:
						format_check_flag = (format_check_flag and False)
						print('\n[ERROR] qr_codes detected in qr_code_list should have 2 components. Debug your code')
				else:
					format_check_flag = (format_check_flag and False)
					print('\n[ERROR] Output of detect_qr_codes should be a nested list. Debug your code')
	else:
		format_check_flag = (format_check_flag and False)
		print('\n[ERROR] Output of detect_qr_codes should be a nested list. Debug your code')
	return format_check_flag

def compare_qr_codes_list(qr_list_generated, qr_list_ideal):
	msgs_generated = []
	coords_generated = []
	msgs_ideal = []
	coords_ideal = []

	if len(qr_list_ideal) == len(qr_list_generated):
		for qr_gen in qr_list_generated:
			msgs_generated.append(qr_gen[0])
			coords_generated.append(list(qr_gen[1]))
		for qr_ideal in qr_list_ideal:
			msgs_ideal.append(qr_ideal[0])
			coords_ideal.append(qr_ideal[1])
		match_msg = set(msgs_generated).intersection(msgs_ideal)
		if len(match_msg) == len(qr_list_ideal):
			compare_flag = True
			for msgs in match_msg:
				index_1 = msgs_generated.index(msgs)
				index_2 = msgs_ideal.index(msgs)
				coord_gen = coords_generated[index_1]
				coord_ideal = coords_ideal[index_2]
				diff = [a - b for a, b in zip(coord_gen, coord_ideal)]
				if diff[0] < 5 and diff[1] < 5:
					compare_flag = (compare_flag and True)
				else:
					compare_flag = (compare_flag and False)		
	
	return compare_flag

def task_1b_cardinal_main():
	# Importing the sim module for Remote API connection with CoppeliaSim
	# sim = __import__('sim')
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

	task_1b_solution = json.load(open(resource_path('task_1b_solutions.json')))
	testcases = json.load(open(resource_path('testcases_1b.json')))
	qr_block_handles = {}
	test_cases_passed = []; task_1b_score = 0

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')
	# cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)

	try:
		client_id = task_1b.init_remote_api_server()

		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')
			print('\n============================================================================')

			# # Starting the Simulation
			# try:
			# 	return_code = task_1b.start_simulation()

			# 	if (return_code == sim.simx_return_novalue_flag):
			# 		print('\nSimulation started correctly in CoppeliaSim.')
			# 		print(client_id)

			# 	else:
			# 		print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
			# 		print('start_simulation function is not configured correctly, check the code!')
			# 		print()
			# 		sys.exit()

			# except Exception:
			# 	print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
			# 	print('Stop the CoppeliaSim simulation manually.\n')
			# 	traceback.print_exc(file=sys.stdout)
			# 	print()
			# 	sys.exit()
		
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

	for i in range(1, 11):
		testcase_name = "test_case_" + str(i)

		# Starting the Simulation
		try:
			return_code = task_1b.start_simulation(client_id)

			if (return_code == sim.simx_return_novalue_flag or return_code == sim.simx_return_ok):
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

		qr_block_handles = get_qr_block_handles(client_id)
		current_testcase = testcases[testcase_name]

		for qr_block_name in qr_block_handles.keys():
			handle = qr_block_handles[qr_block_name]
			position = current_testcase[qr_block_name]

			return_code = sim.simxSetObjectPosition(client_id, handle, -1, tuple(position), sim.simx_opmode_oneshot)
			sim.simxGetPingTime(client_id)

		# Get image array and its resolution from Vision Sensor in ComppeliaSim scene
		try:
			vision_sensor_image, image_resolution, return_code = task_1b.get_vision_sensor_image(client_id)

			if ((return_code == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(vision_sensor_image) > 0)):
				# print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

				# Get the transformed vision sensor image captured in correct format
				try:
					transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)

					if (type(transformed_image) is np.ndarray):
						# cv2.imwrite(testcase_name + ".png",transformed_image)



						qr_codes_list = task_1b.detect_qr_codes(transformed_image)
						# print(qr_codes_list)

						if check_format(qr_codes_list):
							# print(testcase_name, "hi")
							compare_flag = compare_qr_codes_list(qr_codes_list, task_1b_solution[testcase_name])

							if compare_flag:
								test_cases_passed.append("Passed")
								task_1b_score = task_1b_score + 1
								print('Your code passed successfully for test case ' + str(i))
								# print('==========================================================================')

							else:
								test_cases_passed.append("Failed")
								print('Your code failed for test case' + str(i) + '. Wrong detected values, please check your code!')
								# print('============================================================================')							

								

						# cv2.imshow('transformed image', transformed_image)
						# if cv2.waitKey(1) & 0xFF == ord('q'):
						# 	break
						# cv2.destroyAllWindows()
						

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
		
		
	# cv2.destroyAllWindows()


		for qr_block_name in qr_block_handles.keys():
			handle = qr_block_handles[qr_block_name]
			init_pos = testcases["initial_pos"]
			position = init_pos[qr_block_name]


			return_code = sim.simxSetObjectPosition(client_id, handle, -1, tuple(position), sim.simx_opmode_oneshot)
			sim.simxGetPingTime(client_id)




		
	
		# Ending the Simulation
		try:
			return_code = task_1b.stop_simulation(client_id)
			
			if (return_code == sim.simx_return_novalue_flag or return_code == sim.simx_return_ok):
				print('Simulation stopped correctly.')
				print('============================================================================')

				# # Stop the Remote API connection with CoppeliaSim server
				# try:
				# 	task_1b.exit_remote_api_server()

				# 	if (task_1b.start_simulation() == sim.simx_return_initialize_error_flag):
				# 		print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

				# 	else:
				# 		print('\n[ERROR] Failed disconnecting from Remote API server!')
				# 		print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

				# except Exception:
				# 	print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
				# 	print('Stop the CoppeliaSim simulation manually.\n')
				# 	traceback.print_exc(file=sys.stdout)
				# 	print()
				# 	sys.exit()
			
			else:
				print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
				print('[ERROR] stop_simulation function is not configured correctly, check the code!')
				print('Stop the CoppeliaSim simulation manually.')			
				# print()
				# sys.exit()

		except Exception:
			print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		time.sleep(1)

	try:
		task_1b.exit_remote_api_server(client_id)

		if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
	
	return test_cases_passed, task_1b_score

if __name__ == '__main__':

	print("*******************************************************************")
	print("*                                                                 *")
	print("*        =================================================        *")
	print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
	print("*        =================================================        *")
	print("*                                                                 *")
	print("*   This test suite is intended to check the output of Task 1B    *")
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

		test_cases_passed, task_1b_score = task_1b_cardinal_main()

		f = open("task_1b_result.txt", "w")
		f.write(encrypted_team_id); f.write("\n")
		f.write(encrypted_date_time); f.write("\n")
		f.write(encrypted_platform); f.write("\n")
		f.write(encrypted_mac); f.write("\n")

		for test_case in test_cases_passed:
			encrypted_test_case = cryptocode.encrypt(test_case, "eyrc_berryminator_2021-22")
			f.write(encrypted_test_case); f.write("\n")
		task_1b_score_scaled = task_1b_score*3
		encrypted_score = cryptocode.encrypt(str(task_1b_score), "eyrc_berryminator_2021-22")
		f.write(encrypted_score); f.write("\n")
		f.close()
		print("\nTask 1B result text file generated.")