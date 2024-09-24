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
# from pyzbar.pyzbar import decode

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

def check_flag_format(detected_shapes):
	
	format_check_flag = True

	if (isinstance(detected_shapes,list)):
		if len(detected_shapes) == 0:
			format_check_flag = (format_check_flag and False)
			print('\n[ERROR] detected_shapes is returning an empty list. Debug your code')
		else:
			for shape in detected_shapes:
				if (isinstance(shape,list)):
					if len(shape) == 3:
						if (isinstance(shape[0],str)) and (isinstance(shape[1],str)) and (isinstance(shape[2],tuple)):
							format_check_flag = (format_check_flag and True)
						else:
							format_check_flag = (format_check_flag and False)
							print('\n[ERROR] The values detected in detected_shapes are not represented in the required formats. Debug your code')
					else:
						print('\n[ERROR] Shapes lists detected in detected_shapes should have 3 components (color, shape and coordinates). Debug your code')
				else:
					format_check_flag = (format_check_flag and False)
					print('\n[ERROR] The values detected in detected_shapes are not represented in the required formats. Debug your code')
	else:
		format_check_flag = (format_check_flag and False)
		print('\n[ERROR] detected_shapes is not returning a list. Debug your code')
	
	return format_check_flag

def distance(coord1, coord2):
	a1, b1 = coord1
	a2, b2 = coord2
 
	distance = math.sqrt((a2-a1)**2 + (b2-b1)**2)
	return distance

def compare_outputs(ideal, generated):
	if len(ideal) == len(generated):
		temp = np.zeros((len(ideal), len(ideal)), dtype=float)
		for index1 in range(len(ideal)):
			for index2 in range(len(generated)):
				ideal_temp = ideal[index1]
				generated_temp = generated[index2]
				dist = distance(ideal_temp[2], generated_temp[2])
				temp[index1, index2] = dist
		matching = []
		for row_index in range(len(temp)):
			minimum = min(temp[row_index])
			column_index = list(temp[row_index]).index(minimum)
			matching.append((row_index, column_index))
		compare = []
		for element in matching:
			a, b = element
			ideal_val = ideal[a]
			generated_val = generated[b]
			if ideal_val[0] == generated_val[0]:
				if ideal_val[1] == generated_val[1]:
					if distance(ideal_val[2], generated_val[2]) < 8:
						compare.append(True)
					else:
						compare.append(False)
				else:
					compare.append(False)
			else:
				compare.append(False)

		if False in compare:
			return False
		else:
			return True
	else:
		return False

def task_1a_cardinal_main():
	try:
		task_1a = __import__('task_1a')

	except ImportError:
		print('\n[ERROR] task_1a.py file is not present in the current directory.')
		print('Your current directory is: ', os.getcwd())
		print('Make sure task_1a.py is present in this current directory.\n')
		sys.exit()
		
	except Exception as e:
		print('Your task_1a.py throwed an Exception, kindly debug your code!\n')
		traceback.print_exc(file=sys.stdout)
		sys.exit()

	ideal_output = json.load(open(resource_path('task_1a_solutions.json')))
	
	test_cases_passed = []
	total_score = 0

	for file_num in range(1, 31):
		img_dir_path = "all_test_images/"
		img_key = 'test_image_' + str(file_num) + '.png'
		img_file_path = img_dir_path + img_key
		img = cv2.imread(resource_path(img_file_path))

		detected_shapes = task_1a.detect_shapes(img)
		format_check_flag = check_flag_format(detected_shapes)

		if format_check_flag:
			compare_flag = compare_outputs(ideal_output[img_key], detected_shapes)

			if compare_flag:
				print('Your code passed successfully for test_image_' + str(file_num) + '.png')
				print('==========================================================================')
				test_cases_passed.append("Passed")
				total_score = total_score + 1
			else:
				print('Your code failed for test_image_' + str(file_num) + '.png. Wrong detected values, please check your code!')
				print('============================================================================')
				test_cases_passed.append("Failed")
	
	return test_cases_passed, total_score

if __name__ == '__main__':

	print("*******************************************************************")
	print("*                                                                 *")
	print("*        =================================================        *")
	print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
	print("*        =================================================        *")
	print("*                                                                 *")
	print("*   This test suite is intended to check the output of Task 1A    *")
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
		encrypted_team_id = cryptocode.encrypt(str(team_id), "eyrc_berryminator_2021-22")
		encrypted_date_time = cryptocode.encrypt(str(datetime.now()), "eyrc_berryminator_2021-22")
		encrypted_platform = cryptocode.encrypt(platform_uname, "eyrc_berryminator_2021-22")
		encrypted_mac = cryptocode.encrypt(str(hex(uuid.getnode())), "eyrc_berryminator_2021-22")
		
		test_cases_passed, task_1a_score = task_1a_cardinal_main()

		f = open("task_1a_result.txt", "w")
		f.write(encrypted_team_id); f.write("\n")
		f.write(encrypted_date_time); f.write("\n")
		f.write(encrypted_platform); f.write("\n")
		f.write(encrypted_mac); f.write("\n")

		for test_case in test_cases_passed:
			encrypted_test_case = cryptocode.encrypt(test_case, "eyrc_berryminator_2021-22")
			f.write(encrypted_test_case); f.write("\n")
		encrypted_score = cryptocode.encrypt(str(task_1a_score), "eyrc_berryminator_2021-22")
		f.write(encrypted_score); f.write("\n")
		f.close()
		print("\nTask 1A result text file generated.")