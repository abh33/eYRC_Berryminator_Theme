'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 2A         
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task_2a.py
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

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import re
import cv2
import numpy as np
import os, sys
import traceback
import csv
import random
import math
import platform
import cryptocode
import uuid
import datetime
from pyzbar.pyzbar import decode
import psutil
from datetime import datetime

##############################################################

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

try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()

# task_1b = __import__('task_1b')
try:
	task_1b = __import__('task_1b')

except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

except Exception as e:
	print('Your task_1b.py throwed an Exception, kindly debug your code!\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

try:
	task_2a = __import__('task_2a')

except ImportError:
	print('\n[ERROR] task_2a.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_2a.py is present in this current directory.\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

except Exception as e:
	print('Your task_2a.py throwed an Exception, kindly debug your code!\n')
	traceback.print_exc(file=sys.stdout)
	sys.exit()

x_ranges = list(np.arange(-0.45, 0.5, 0.05, dtype=np.float32))
y_ranges = list(np.arange(-0.45, 0.5, 0.05, dtype=np.float32))
z_ranges = list(np.arange(0.3, 2.0, 0.05, dtype=np.float32))
x_ranges = [round(num, 2) for num in x_ranges]
y_ranges = [round(num, 2) for num in y_ranges]
z_ranges = [round(num, 2) for num in z_ranges]
berry_names = ["strawberry_1", "strawberry_2", "strawberry_3", "strawberry_4", "lemon_1", "lemon_2", "lemon_3", "lemon_4", "blueberry_1", "blueberry_2", "blueberry_3", "blueberry_4"]


def spawn_random_berries(client_id, berry_handles, vision_sensor_handle):

	number_of_berries = random.randrange(1, 6)
	random_berries = random.sample(berry_names, number_of_berries)
	x_pos = random.sample(x_ranges, number_of_berries)
	y_pos = random.sample(y_ranges, number_of_berries)
	z_pos = random.sample(z_ranges, number_of_berries)
	berry_positions_solutions = {"Strawberry": [], "Lemon" : [], "Blueberry": []}
	for num in range(number_of_berries):
		x, y, z = x_pos[num], y_pos[num], z_pos[num]
		berry_name = random_berries[num]
		berry_handle = berry_handles[berry_name]
		set_position = (x, y, z)
		return_code = sim.simxSetObjectPosition_mod_berry(client_id, berry_handle, vision_sensor_handle, set_position, sim.simx_opmode_blocking)
		if berry_name[0] == 's':
			(berry_positions_solutions['Strawberry']).append(set_position)
		elif berry_name[0] == 'l':
			(berry_positions_solutions['Lemon']).append(set_position)
		elif berry_name[0] == 'b':
			(berry_positions_solutions['Blueberry']).append(set_position)
		else:
			pass
	return berry_positions_solutions, random_berries

def unspawn_random_berries(client_id, random_berries, berry_handles):
	for berry in random_berries:
		berry_handle = berry_handles[berry]
		return_code = sim.simxSetObjectPosition_mod_berry(client_id, berry_handle, -1, (0.75, 0.75, 0.25), sim.simx_opmode_blocking)
	return return_code

def format_check_positions(berry_positions_dictionary):
	format_check_flag = True
	berry_type = list(berry_positions_dictionary.keys())

	if isinstance(berry_positions_dictionary, dict):
		if ("Strawberry" in berry_type) and ("Blueberry" in berry_type) and ("Lemon" in berry_type):
			for berry in berry_type:
				if isinstance(berry,str):
					val = berry_positions_dictionary[berry]
					if isinstance(val,list):
						for i in val:
							if isinstance(i,tuple) and len(i) == 3:
								if (isinstance(i[0],np.float32) or isinstance(i[0],float)) and (isinstance(i[1],np.float32) or isinstance(i[1],float)) and (isinstance(i[2],np.float32) or isinstance(i[2],float)):
									format_check_flag = format_check_flag and True
								else:
									format_check_flag = format_check_flag and False
									print('\n[ERROR] Wrong format of coordinates. Each coordinate value should be in float.')
							else:
								format_check_flag = format_check_flag and False
								print('\n[ERROR] Wrong format of coordinates. All 3 coordinates should be in a tuple.')
					else:
						format_check_flag = format_check_flag and False
				else:
					format_check_flag = format_check_flag and False
					print('\n[ERROR] Wrong format. Berry type should be string.')
		else:
			format_check_flag = format_check_flag and False
			print(print('\n[ERROR] All 3 fruit names should be present in dictionary'))		

	return format_check_flag

def format_check(berries_dictionary):
	format_check_flag = True
	berry_type = list(berries_dictionary.keys())

	if isinstance(berries_dictionary, dict):
		if ("Strawberry" in berry_type) and ("Blueberry" in berry_type) and ("Lemon" in berry_type):
			for berry in berry_type:
				if isinstance(berry,str):
					val = berries_dictionary[berry]
					if isinstance(val,list):
						for i in val:
							if isinstance(i,tuple) and len(i) == 3:
								if isinstance(i[0],int) and isinstance(i[1],int) and (isinstance(i[2],np.float32) or isinstance(i[2],float)):
									format_check_flag = format_check_flag and True
								else:
									format_check_flag = format_check_flag and False
									print('\n[ERROR] Wrong format of coordinates')
							else:
								format_check_flag = format_check_flag and False
								print('\n[ERROR] Wrong format of coordinates. All 3 coordinates should be in a tuple.')
					else:
						format_check_flag = format_check_flag and False
				else:
					format_check_flag = format_check_flag and False
					print('\n[ERROR] Wrong format. Berry type should be string.')
		else:
			format_check_flag = format_check_flag and False
			print(print('\n[ERROR] All 3 fruit names should be present in dictionary'))		

	return format_check_flag

def distance(coord1, coord2):
	a1, b1, c1 = coord1
	a2, b2, c2 = coord2
 
	distance = math.sqrt((a2-a1)**2 + (b2-b1)**2 + (c2-c1)**2)
	return distance

def compare_dictionaries(berry_positions_dictionary, berry_positions_solutions):
	berry_type = berry_positions_solutions.keys()
	compare = []
	for berry in berry_type:
		berry_ideal = berry_positions_solutions[berry]
		berry_detected = berry_positions_dictionary[berry]
		if len(berry_ideal) == len(berry_detected):
			temp = np.zeros((len(berry_ideal), len(berry_ideal)), dtype=float)
			for index1 in range(len(berry_ideal)):
				for index2 in range(len(berry_detected)):
					ideal_temp = berry_ideal[index1]
					detected_temp = berry_detected[index2]
					dist = distance(ideal_temp, detected_temp)
					temp[index1, index2] = dist
			matching = []
			for row_index in range(len(temp)):
				minimum = min(temp[row_index])
				column_index = list(temp[row_index]).index(minimum)
				matching.append((row_index, column_index))
			for element in matching:
				a, b = element
				ideal_val = berry_ideal[a]
				detected_val = berry_detected[b]
				# print(round(abs(ideal_val[0]-detected_val[0]), 3), round(abs(ideal_val[1]-detected_val[1]), 3), round(abs(ideal_val[2]-detected_val[2]), 3), round(distance(ideal_val, detected_val), 3))
				if (distance(ideal_val, detected_val)) < 0.015:
					compare.append(True)
				else:
					compare.append(False)
		else:
			return False
		
	if False in compare:
		return False
	else:
		return True


def get_size(bytes, suffix="B"):
	"""
	Scale bytes to its proper format
	e.g:
		1253656 => '1.20MB'
		1253656678 => '1.17GB'
	"""
	factor = 1024
	for unit in ["", "K", "M", "G", "T", "P"]:
		if bytes < factor:
			return f"{bytes:.2f}{unit}{suffix}"
		bytes /= factor

if __name__ == '__main__':

	print("*******************************************************************")
	print("*                                                                 *")
	print("*        =================================================        *")
	print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
	print("*        =================================================        *")
	print("*                                                                 *")
	print("*   This test suite is intended to check the output of Task 2A    *")
	print("*            of Berryminator (BM) Theme (eYRC 2021-22)            *")
	print("*                                                                 *")
	print("*******************************************************************")

	try:
		team_id = int(input('\nEnter your Team ID (for e.g.: "1234" or "321"): '))

	except ValueError:
		print("\n[ERROR] Enter your Team ID which is an integer!\n")
		sys.exit()

	platform_uname = platform.uname().system
	platform_node = platform.uname().node
	platform_release = platform.uname().release
	platform_version = platform.uname().version
	platform_machine = platform.uname().machine
	platform_processor = platform.uname().processor
	physical_cores = psutil.cpu_count(logical=False)
	total_cores = psutil.cpu_count(logical=True)
	cpu_frequency = psutil.cpu_freq()
	max_frequency = f"{cpu_frequency.max:.2f}"
	min_frequency = f"{cpu_frequency.min:.2f}"
	current_frequency = f"{cpu_frequency.current:.2f}"
	virtual_memory = psutil.virtual_memory()
	total_memory = f"{get_size(virtual_memory.total)}"
	available_memory = f"{get_size(virtual_memory.available)}"
	used_memory = f"{get_size(virtual_memory.used)}"
	percentage = f"{virtual_memory.percent}%"
	
	encrypted_team_id = cryptocode.encrypt(str(team_id), "berryminator_task2a_eyrc")
	encrypted_date_time = cryptocode.encrypt(str(datetime.now()), "berryminator_task2a_eyrc")
	encrypted_platform = cryptocode.encrypt(platform_uname, "berryminator_task2a_eyrc")
	encrypted_mac = cryptocode.encrypt(str(hex(uuid.getnode())), "berryminator_task2a_eyrc")
	encrypted_platform_node = cryptocode.encrypt(str(platform_node), "berryminator_task2a_eyrc")
	encrypted_platform_release = cryptocode.encrypt(str(platform_release), "berryminator_task2a_eyrc")
	encrypted_platform_version = cryptocode.encrypt(str(platform_version), "berryminator_task2a_eyrc")
	encrypted_platform_machine = cryptocode.encrypt(str(platform_machine), "berryminator_task2a_eyrc")
	encrypted_platform_processor = cryptocode.encrypt(str(platform_processor), "berryminator_task2a_eyrc")
	encrypted_physical_cores = cryptocode.encrypt(str(physical_cores), "berryminator_task2a_eyrc")
	encrypted_total_cores = cryptocode.encrypt(str(total_cores), "berryminator_task2a_eyrc")
	encrypted_max_frequency = cryptocode.encrypt(str(max_frequency), "berryminator_task2a_eyrc")
	encrypted_min_frequency = cryptocode.encrypt(str(min_frequency), "berryminator_task2a_eyrc")
	encrypted_current_frequency = cryptocode.encrypt(str(current_frequency), "berryminator_task2a_eyrc")
	encrypted_total_memory = cryptocode.encrypt(str(total_memory), "berryminator_task2a_eyrc")
	encrypted_available_memory = cryptocode.encrypt(str(available_memory), "berryminator_task2a_eyrc")
	encrypted_used_memory = cryptocode.encrypt(str(used_memory), "berryminator_task2a_eyrc")
	encrypted_percentage = cryptocode.encrypt(str(percentage), "berryminator_task2a_eyrc")
	test_cases_passed = []
	berry_pos_list = []
	berry_pos_list_sol = []


	conda_env_name = os.environ['CONDA_DEFAULT_ENV']

	expected_conda_env_name = 'BM_' + str(team_id)

	if conda_env_name == expected_conda_env_name:
		
		conda_env_name_flag = 1

	else:
		
		conda_env_name_flag = 0
		print("\n[WARNING] Conda environment name is not found as expected: BM_%s. Run this file with correct conda environment.\n" %(str(team_id)))
		sys.exit()

	if conda_env_name_flag == 1:

		berries_dictionary = {}
		berry_positions_dictionary = {}
		berry_names = ["strawberry_1", "strawberry_2", "strawberry_3", "strawberry_4", "lemon_1", "lemon_2", "lemon_3", "lemon_4", "blueberry_1", "blueberry_2", "blueberry_3", "blueberry_4"]
		berry_handles = {}
		berry_positions_solutions = {"Strawberry": [], "Lemon" : [], "Blueberry": []}
		random_berries = []
		count = 0
		score = 0

		print('\nConnection to CoppeliaSim Remote API Server initiated.')
		print('Trying to connect to Remote API Server...')
		# cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)
		# cv2.namedWindow('transformed depth image', cv2.WINDOW_AUTOSIZE)

		try:
			# Initiate Remote API connection
			client_id = task_1b.init_remote_api_server()

			if (client_id != -1):
				print('\nConnected successfully to Remote API Server in CoppeliaSim!')
				return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor', sim.simx_opmode_blocking)
				# return_code, strawberry_1 = sim.simxGetObjectHandle(client_id, 'strawberry_1', sim.simx_opmode_blocking)
				## "C:\\Users\\abhin\\Desktop\\Workspace\\Task 2A\\"
				for berry in berry_names:
					obj_name = berry.replace("_", "")
					ret_code, handle = sim.simxGetObjectHandle(client_id, obj_name, sim.simx_opmode_blocking)
					if ret_code == 8:
						if berry[0] == 's':
							return_code, berry_handle = sim.simxLoadModel(client_id, resource_path("strawberry_fruit.ttm"), 0xFF, sim.simx_opmode_blocking)
						elif berry[0] == 'l':
							return_code, berry_handle = sim.simxLoadModel(client_id, resource_path("lemon_fruit.ttm"), 0xFF, sim.simx_opmode_blocking)
						elif berry[0] == 'b':
							return_code, berry_handle = sim.simxLoadModel(client_id, resource_path("blueberry_fruit.ttm"), 0xFF, sim.simx_opmode_blocking)
						else:
							pass
					else:
						berry_handle = handle
						return_code = sim.simxSetObjectPosition_mod_berry(client_id, berry_handle, -1, (0.75, 0.75, 0.25), sim.simx_opmode_blocking)
					berry_handles[berry] = berry_handle
					# return_code, berry_pos = sim.simxGetObjectPosition(client_id, berry_handle, vision_sensor_handle, sim.simx_opmode_blocking)
					# berry_positions_initial[berry] = berry_pos
				# print(berry_handles)


				# Starting the Simulation
				try:
					return_code = task_1b.start_simulation(client_id)

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

		while count < 40:

		# Get image array and depth buffer from vision sensor in CoppeliaSim scene
			try:
				berry_positions_solutions, random_berries = spawn_random_berries(client_id, berry_handles, vision_sensor_handle)
				vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
				vision_sensor_depth_image, depth_image_resolution, return_code_2 = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)

				# print(return_code, return_code_2, len(image_resolution), len(image_resolution))

				if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
					print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

					# Get the transformed vision sensor image captured in correct format
					try:
						transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
						transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)

						if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

							berries_dictionary = task_2a.detect_berries(transformed_image, transformed_depth_image)

							if format_check(berries_dictionary) == True:
								berry_positions_dictionary = task_2a.detect_berry_positions(berries_dictionary)

								if format_check_positions(berry_positions_dictionary) == True:

									berry_pos_list.append(str(berry_positions_dictionary))
									berry_pos_list_sol.append(str(berry_positions_solutions))

									if compare_dictionaries(berry_positions_dictionary, berry_positions_solutions):
										print("=============================================================")
										print("Test case " + str(count+1)+ " was passed !")
										print("=============================================================")
										score = score + 1
										test_cases_passed.append("Passed")
									else:
										print("=============================================================")
										print("Test case " + str(count+1)+ " was failed !")
										print("=============================================================")
										test_cases_passed.append("Failed")
							# print(berry_positions_solutions)
							# print("Berry Positions Dictionary = ",berry_positions_dictionary)


							# cv2.imshow('transformed image', transformed_image)
							# # cv2.imshow('transformed depth image', transformed_depth_image)
							# # cv2.imshow('labelled image', labelled_image)
							# if cv2.waitKey(1) & 0xFF == ord('q'):
							# 	break
							

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
					# return test_cases_passed, berry_pos_list, berry_pos_list_sol
					# sys.exit()

				unspawn_random_berries(client_id, random_berries, berry_handles)
				count = count + 1

			except Exception:
				print('\n[ERROR] Your get_vision_sensor_image function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
				sys.exit()
			
			
		cv2.destroyAllWindows()

		for berry in berry_handles.keys():
			handle = berry_handles[berry]
			sim.simxRemoveModel(client_id, handle, sim.simx_opmode_blocking)

		# Ending the Simulation
		try:

			return_code = task_1b.stop_simulation(client_id)
			
			if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
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

		
		print("Your code passed " + str(test_cases_passed.count("Passed")) + "/40 random test cases !")

 

		f = open("task_2a_result.txt", "w")
		f.write(encrypted_team_id); f.write("\n")
		f.write(encrypted_date_time); f.write("\n")
		f.write(encrypted_platform); f.write("\n")
		f.write(encrypted_mac); f.write("\n")

		f.write(encrypted_platform_node); f.write("\n")
		f.write(encrypted_platform_release); f.write("\n")
		f.write(encrypted_platform_version); f.write("\n")
		f.write(encrypted_platform_machine); f.write("\n")
		f.write(encrypted_platform_processor); f.write("\n")
		f.write(encrypted_physical_cores); f.write("\n")
		f.write(encrypted_total_cores); f.write("\n")
		f.write(encrypted_max_frequency); f.write("\n")
		f.write(encrypted_min_frequency); f.write("\n")
		f.write(encrypted_current_frequency); f.write("\n")
		f.write(encrypted_total_memory); f.write("\n")
		f.write(encrypted_available_memory); f.write("\n")
		f.write(encrypted_used_memory); f.write("\n")
		f.write(encrypted_percentage); f.write("\n")

		for index in range(len(test_cases_passed)):
			encrypted_test_case = cryptocode.encrypt(test_cases_passed[index], "berryminator_task2a_eyrc")
			encrypted_berry_pos_det = cryptocode.encrypt(berry_pos_list[index], "berryminator_task2a_eyrc")
			encrypted_berry_pos_sol = cryptocode.encrypt(berry_pos_list_sol[index], "berryminator_task2a_eyrc")
			f.write(encrypted_test_case); f.write("\n")
			f.write(encrypted_berry_pos_det); f.write("\n")
			f.write(encrypted_berry_pos_sol); f.write("\n")

		f.close()
		print("\nTask 2A result text file generated.")

		f1 = open("task_2a_outputs.txt", "w")
		for index in range(len(test_cases_passed)):
			f1.write("Testcase " + str(index + 1) + ": "); f1.write(test_cases_passed[index]); f1.write("\n")
			f1.write("Detected Berry Positions: "); f1.write(berry_pos_list[index]); f1.write("\n")
			f1.write("Actual Berry Positions: "); f1.write(berry_pos_list_sol[index]); f1.write("\n\n")
		f1.close()




