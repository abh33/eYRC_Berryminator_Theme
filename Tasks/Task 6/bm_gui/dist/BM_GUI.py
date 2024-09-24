from PyQt5 import QtCore, QtGui, QtWidgets
import sys, os
import os
import signal
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
import re
from pyzbar.pyzbar import decode
import gspread
# from test_task_6 import init_remote_api_server, get_child_data, get_custom_data
import sim
# import threading

team_id_val = None
folder_path = None
config_type = None
res_path = None
task = None
pid = None
client_id = -1

headers=[
'Date and Time',
'Platform',
'Mac',
'VALID Run?',
'T',
'CI',
'CP',
'CD',
'P',
'B',
'score',
'Correctly Identified',
'Correctly Plucked',
'Correctly Dropped',
'Correctly Dropped in CB1',
'Correctly Dropped in CB2',
'Collisions',
'Mass of Arm',
'Total simulation time',
'RTF',
'No. of joints',
'Torque',
'Force'
]

# Credentials required to communicate with Original Config Google Sheets.
# https://docs.gspread.org/en/latest/index.html
axebnfgh={
  "type": "service_account",
  "project_id": "bm-eyrc21-task4",
  "private_key_id": "fecb66f9dbadfdf30f1c953ca55fec020572f55b",
  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDNLsNg9p43FXHa\nTD/b/Tg2WtRYaeZ/yNeLOr/tA+5bagDv1xA6/C3O6UGir9FMACqJMj4ZM+wWVEz3\nfxH96nJ7GjQNKUm3iLySnkUpoUFBc6BL6YpRBqRxTWXo/MlYy9cRqVqtnZ9yLB1t\nviHzixJeCIcSE/Y2+WCDwJg0NK/l6gsTpX57SZupZqfslS/hhP3UvJtIjdfneGVy\nrdRMUI30OJkRg4uO1OkH5q4pXPIkjqCr6pawWfxLbRRbE9anXrGigbzuC1d1ATWL\nBldB2+7FBL3fNTBRBb9sKTvwpHiId50/xdTB5Bwt7eTG5KeZCf2NC1mR52tHOkba\noJBQuT5zAgMBAAECggEADceXiiowiJ/WvEO6q4TfWDx+s6Feat6Lezzq1CQbceWL\nQpA9oCMngGncj+x22z/M8Ncq9yOsNfkGDOkVGum6iSyX/6TyNR09wisBl2t4/R2i\nz6D+OmApKGLQ9qC4yk8w68K6kEZcWvKTggyZgx2n53Yd0XJTFXC8sUcnRvpIMIyJ\n0FT2hSL454bgvM8gPuMvpP8VwULN6CfVHLvgDn0UMnzoZ9OZVRkX5ZDZwzJvzRoB\nDV9se5akflbPPXY8Py82tL1EqZwBSUJKD1dc8N5yq1doyC30gfoUVWqgtP63yRSO\n1DNqIMIypcXUqygiOmyC+vd30tYKudbGWHjAC9ZsYQKBgQDudvp9dTrNwQrWDxET\nvfxezZHO8L0qkU/+pi5JZyrYaMZQlj67uUtL7i8tm8x1aaCXYXQlPLwdAQJaw9uA\nUN+0KUNBZCZue0BvKKKB1FO1eFDrn4ACNnal3TCDPPO0/noNQydeTOOc4K+Sda8o\nr8IKO1Ri4QLOEzdK8CXJgFiYbQKBgQDcRUKbnkid/XcuK7JY5nSe3L4QpX1vMzxW\nF5bDI1sQYjOvbZ6MyddXOe9g/Ie0XK0npml2KYcRLWAWBGfHBCq8aPWoMkyBrhYz\nGGZgY1StrdUBqD58M+xWnxXVzlMJCZ8b7Kr34bYU5ti9v0lYm1Y3SQqipTqvDBAA\niQyLc8qmXwKBgHxdWqgBcoiKC9+1fd+A7tR2e3uke9Nuc3zIKE2p22N/ySWQJel7\nanSQJNSxc0EstE8eyRb+hNDPjO3PRXO3OePYvTWmSJDIAMO4OoBhPsLem702suj0\nhI5ePAXFzQfwy7CM/EQm73IKkCZe79nVR+pUJN9ooDDfPq/7bImZQRtFAoGALJnB\nkO0N2x+1gB3M8P9I4dnkik5YBaokL3J7TmvcRob73CugDPXZqonKlKLNf5G8Zejm\nNczW0n2TlSGzObL4TEocY8A7iY6MZDAumpnCTaCDHuDINjH1StzPeQlb+tp94xOa\n0rJbQgQ0IseWvGBHps4dXy2BomwY2WLmrV32zJUCgYEAnMFseXqLtHyzBXXsfHwE\nxpoL09eOUzs4Ujbsr3NcRRJsGJHWs6JwrasRgVgIIymAygj8GuDe2UEwDfIhJMx/\nd/P6vvJCAgeR21F9WxdYFCMDxx9V96dD2zRWE7oJvG5qor1ow2kqTtW683+BHVzN\nXFUrpPx+tCGRc6/ICdsnOyU=\n-----END PRIVATE KEY-----\n",
  "client_email": "eyrc21-22bmtask6@bm-eyrc21-task4.iam.gserviceaccount.com",
  "client_id": "110386543062795768965",
  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
  "token_uri": "https://oauth2.googleapis.com/token",
  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/eyrc21-22bmtask6%40bm-eyrc21-task4.iam.gserviceaccount.com"
}

# Credentials required to communicate with Bonus Configuration Google Sheets.
# https://docs.gspread.org/en/latest/index.html
bnaxefgh={
  "type": "service_account",
  "project_id": "bm-eyrc21-task4",
  "private_key_id": "c78c8f19c22ea34d4f9f93951197fca5f3dfe3cb",
  "private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQDWFuJ7vQgwoGOJ\nwOGUZsPrHnebCRx2VIo+pjMhAnI91nF+zmu8m1BedoSqDjhrweurvx21YHXJKgPO\nbIoYlBE5MPmRbTAvNzfrFo4PBlLsrAs57UxTNoKMZIVMj5ejttYAbRXDfzJGRhO2\nFqamz31kqjR07IeecccjN6GMdsRTlgyVwHioA6xXKrSROwJPEdShdl+2D0jhoqjh\nAEmCTD6KYM8fMb15Z42uxSbGFuf7yoUlJR7XFdelqb6gygCgkOCe4ErtS2IAPgcc\n5zidD66lrUtJDIwTtyzMwpiO1NFDMbBIV22AjouUU81ZZCJ4SvVo7g0nvAEQef+R\nMAbje5ixAgMBAAECggEAHLKqn22ieGo3jZKmam8wGgdWfxjEkPQaJ/W9atJaIAHr\n6tSeyoSDTHZUqDsJwyAWiEoxIEwS3elc40HXbR4EBQeUnLLnP50GSOQYHNl7KjCR\nvK0bixgGfe1HlRC2ggKTQzFcNn2JbrQcFk+I/VlY9LKAH6BJOasEx6rAZATTomR+\n9JuldI3h4CecN0YuaDFyj9bPzDhHPFgFpLj/ZwPGGQvOs9RPXGmnR6bVDdf7xP+3\nAK9NhXlCyzfQTjpmzRh2mX6rUINV8QwbkX9XNoRGZvVxXKooGoA+b3kSrfS/hDh0\nEZS2zO4Osy2fvzuVbC8gUCCSeVv1mvtsW+vTsMthtQKBgQDyLcPmBprOi5rD7fYF\nsyLxnkj9LDKW3zH98LcyxhRfLNP/R2yzMTp+jHbWdM18dKQFqOQ/2Z0EKlY4DQ42\n1zAyoyAtjOvdDbU9dlN//bHYQtARPDeUqF0gVs5IHn3lmwdQCpmZDQ3U13LhDsOu\nNAIv7w3QaPuR362BrL2m/YlymwKBgQDiTru7rwmfWIe9nbEqdyl3asLbh3/ejB8w\n58H9S4497o5sBI7zH426NZ9zHu5Hkafwvj93X2t4z7qUppiV5uOs6AsoRQyozuxQ\nYQzd9K0LPYojOtvhKnmFuRRV1DiSSJ1MR2S09ek5BLrWpOx98ruhCv/WWN0Un5cn\nkWQFBRDgowKBgC1ZRIWeVv62RZTKynbmxHRaH3DJBfAae1IB8Uwgq0/nJQt4ZHJp\nlXp88wPVe2UCsnQc3CDYrv5IrzxvtWmvgeb5hN0Ctmf+WSHZNmmRJ+xHFo8jp4S+\nwjF8D8SRcSG9lfD3CywNtblVXr2l9h54vrAbI4sTHYiDSgouPpU2hLZhAoGBAKcE\nclqmaNSX8r4HFkx/2zONsDdyPfWRtidSs4FrdJPSbnvbtWBKxj9J+d5tUm+xWJ88\n2PwRiPCFZm70sKHY3+io7CsRIrGm3RLJPUTFobws87jiZIo93afKGu7pC8MyIhy8\nh99hqXjyO63T09F0BfNpebzIoojo6xZBn+5wea8/AoGBAJDr8TwsQSL1WgRqWuN6\nQ0xGmStAGM5qOcWrrJfIgeaUKQJW9CJviwODw2oYZzljXIiU+R4dytSOHOh0w1cM\nLy7KmK7wFMvBDhfpI0ZJoMjtwS8kX80W/jBVPx1tv05dRm6SAC2dvTpePGQJLXNu\nGGpGcdSsECzmqysL2ekTEuoK\n-----END PRIVATE KEY-----\n",
  "client_email": "eyrc21-22bmtask6-bonus@bm-eyrc21-task4.iam.gserviceaccount.com",
  "client_id": "108268863483605764771",
  "auth_uri": "https://accounts.google.com/o/oauth2/auth",
  "token_uri": "https://oauth2.googleapis.com/token",
  "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
  "client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/eyrc21-22bmtask6-bonus%40bm-eyrc21-task4.iam.gserviceaccount.com"
}

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

class EmittingStream(QtCore.QObject):

	textWritten = QtCore.pyqtSignal(str)

	def write(self, text):
		self.textWritten.emit(str(text))

def load_fonts_from_dir(directory):
	families = set()
	for fi in QtCore.QDir(directory).entryInfoList(["*.ttf", "*.woff", "*.woff2"]):
		_id = QtGui.QFontDatabase.addApplicationFont(fi.absoluteFilePath())
		families |= set(QtGui.QFontDatabase.applicationFontFamilies(_id))
	return families

def unique(list1):
 
	# initialize a null list
	unique_list = []
	 
	# traverse for all elements
	for x in list1:
		# check if exists in unique_list or not
		if x not in unique_list:
			unique_list.append(x)
	
	return unique_list

def IntersecOfSets(arr1, arr2):
	# Converting the arrays into sets
	s1 = set(arr1)
	s2 = set(arr2)
	# s3 = set(arr3)
	  
	# Calculates intersection of 
	# sets on s1 and s2
	set1 = s1.intersection(s2)         
	  
	# Calculates intersection of sets
	# on set1 and s3
	# result_set = set1.intersection(s3)
	  
	# Converts resulting set to list
	final_list = list(set1)
	# print(final_list)
	return final_list


def capping_lists(argument_list, required_configuration):

	bb_in_argument_list = sum(1 for i in argument_list if i == "Blueberry")
	l_in_argument_list  = sum(1 for i in argument_list if i == "Lemon")
	sb_in_argument_list = sum(1 for i in argument_list if i == "Strawberry")
	none_element_in_argument_list = sum(1 for i in argument_list if i == "None")     # Removing "None" element if present

	max_bb_allowed = int(required_configuration["Blueberry"].split("_")[0] )
	max_l_allowed  = int(required_configuration["Lemon"].split("_")[0] )
	max_sb_allowed = int(required_configuration["Strawberry"].split("_")[0] )

	if bb_in_argument_list > max_bb_allowed:
		no_of_bb_to_remove = bb_in_argument_list - max_bb_allowed

		for i in range(1, no_of_bb_to_remove+1):
			argument_list.remove("Blueberry")

	if l_in_argument_list > max_l_allowed:
		no_of_l_to_remove = l_in_argument_list - max_l_allowed

		for i in range(1, no_of_l_to_remove+1):
			argument_list.remove("Lemon")

	if sb_in_argument_list > max_sb_allowed:
		no_of_sb_to_remove = sb_in_argument_list - max_sb_allowed

		for i in range(1, no_of_sb_to_remove+1):
			argument_list.remove("Strawberry")

	if none_element_in_argument_list > 0:
		for i in range(1, none_element_in_argument_list+1):
			argument_list.remove("None")


	return argument_list

def general_berry_name(argument):

	if (argument.find('lemon')==0):
		return "Lemon"
	elif(argument.find('strawberry')==0):
		return "Strawberry"
	elif(argument.find('blueberry')==0):
		return "Blueberry"
	else:
		return "None"

def calculate_score( end_sim_time, ci_list_to_show, pluck_list_to_show, dropped_in_cb1, dropped_in_cb2, total_no_of_collisions):

	# Intializing the below variablesto zero.
	T = 0
	CI = 0
	CP = 0
	CD = 0
	P = 0
	B = 0
	score = 0

	# NOTE: Subtracting sim time while revealing the bot is left

	# The Required Configuration
	# Note: To be edited as per the given Task
	# Have modifed this a bit. B -> Blueberry

	if config_type == "Original Configuration":
		required_configuration = {"Blueberry": "1_CB1", "Lemon": "5_CB2", "Strawberry": "1_CB1"}
	elif config_type == "Bonus Configuration":
		required_configuration = {"Blueberry": "0_CB1", "Lemon": "1_CB2", "Strawberry": "7_CB1"}


	# blueberry_requriement  = required_configuration["Blueberry"].split("_")             # ex: bb_requiremnt = ['3', 'CB1']
	# lemon_requirement      = required_configuration["Lemon"].split("_")
	# strawberry_requirement = required_configuration["Strawberry"].split("_")


	# First checking whether the run is VALID one or not
	valid_run_flag = False                                                   # False = Invalid Run
	# Note: The below code won't hold if there is no requirement of any one color
	

	# Finding common berries present in CI, CP
	common_berries_in_ci_cp = IntersecOfSets(ci_list_to_show, pluck_list_to_show)
	# Is the value greater than equal to 1
	if len(common_berries_in_ci_cp) >= 1:
		# Now it should be checked with appropriate CB
		# First we need to know which CB is required for these common berries

		# Getting general names
		# general_names_list = general_berry_name(common_berries_in_ci_cp)     # Converts ['lemon_2', 'lemon_3', 'blueberry_2'] to ['Lemon', 'Lemon', 'Blueberry']

		for i in range( len(common_berries_in_ci_cp) ):

			current_berry_general_name = general_berry_name(common_berries_in_ci_cp[i])   # Converts 'lemon_2' to 'Lemon'

			if current_berry_general_name != "None":

				required_cb_for_this_berry = required_configuration[current_berry_general_name].split("_")[1]          # returns 'CB1' or 'CB2'

				# If this berry is required to be dropped in CB1
				if required_cb_for_this_berry == 'CB1':
					# Check whether this berry is actually dropped in CB1 or not
					if common_berries_in_ci_cp[i] in dropped_in_cb1:
						# This means it is dropped in CB1. This constitutes a valid run
						valid_run_flag = valid_run_flag or True                                # Boolean addition since we need atleast one berry

				# Else it is meant to be dropped in CB2
				else:
					# Check whether this berry is actually dropped in CB2 or not
					if common_berries_in_ci_cp[i] in dropped_in_cb2:
						# This means it is dropped in CB2. This constitutes a valid run
						valid_run_flag = valid_run_flag or True                                # Boolean addition

	print('\n#################################################')
	print("\n********** TASK 6 RESULT **********")

	if valid_run_flag:
		print("\nThe current run is a VALID run.")

		# Now a berry in CD should only be considered if it is present in CI and CP lists as well.
		# Because our executable finds CD independently. Therefore we need to confirm its detection and pluck.

		raw_cb1_drops_general_name = []
		raw_cb2_drops_general_name = []

		# Iterating through CB1 drops
		final_cb1_drops = []
		for i in dropped_in_cb1:
			if ( i in ci_list_to_show) and (i in pluck_list_to_show):
				berry_name = general_berry_name(i)
				final_cb1_drops.append(berry_name)
			
			raw_cb1_drops_general_name.append(general_berry_name(i))

		# Iterating through CB2 drops
		final_cb2_drops = []
		for i in dropped_in_cb2:
			if ( i in ci_list_to_show) and (i in pluck_list_to_show):
				berry_name = general_berry_name(i)
				final_cb2_drops.append(berry_name)

			raw_cb2_drops_general_name.append(general_berry_name(i))


		# So we have berries in final_cb1_drops and final_cb2_drops which are both detected and plucked
		# Now we need to check whether the berry has been dropped into the correct CB or not
		allowed_berries_in_cb1 = []                            # Example for Task 5 config: ['Blueberry']
		allowed_berries_in_cb2 = []                            # Example for Task 5 config: ['Lemon', 'Strawberry']

		for key, value in required_configuration.items():
		
			if value.split("_")[1] == "CB1":
				allowed_berries_in_cb1.append(key)
			else:
				allowed_berries_in_cb2.append(key)


		temp = final_cb1_drops
		final_cb1_drops = []
		for i in temp:
			if i in allowed_berries_in_cb1:
				final_cb1_drops.append(i)

		temp = final_cb2_drops
		final_cb2_drops = []
		for i in temp:
			if i in allowed_berries_in_cb2:
				final_cb2_drops.append(i)


		# Now concatenating to form final_cd_list
		final_cd_list = final_cb1_drops + final_cb2_drops
		

		# Now cleaning CI CP CD lists to show them on terminal
		# i.e. ['lemon_2', 'lemon_3', 'strawberry_1']    to    ['Lemon', 'Lemon', 'Strawberry']
		final_ci_list = []
		for i in ci_list_to_show:
			berry_name = general_berry_name(i)
			final_ci_list.append(berry_name)

		final_cp_list = []
		for i in pluck_list_to_show:
			berry_name = general_berry_name(i)
			final_cp_list.append(berry_name)


		# Capping CI, CP and CD lists i.e. maximum values are restricted by the required configuration
		# Also removing any "None" value in the list if any
		temp_ci_list = final_ci_list
		temp_cp_list = final_cp_list
		temp_cd_list = final_cd_list

		final_ci_list = capping_lists(temp_ci_list, required_configuration)
		final_cp_list = capping_lists(temp_cp_list, required_configuration)
		final_cd_list = capping_lists(temp_cd_list, required_configuration)

		CI = len(final_ci_list)
		CP = len(final_cp_list)
		CD = len(final_cd_list) 

		T = end_sim_time

		P = total_no_of_collisions

		# Whether to give bonus or not
		B = 0
		bonus_valid_flag = False               # True denotes bonus is valid
		# First condition: zero penalties
		if P == 0:
			bonus_valid_flag = True
			# CB1 allowed no. of berries
			cb1_allowed = [0, 0, 0]        # B L S
			# CB2 allowed no. of berries
			cb2_allowed = [0, 0, 0]        # B L S

			# Checking whether the required configuration is EXACTLY met or not

			sum_detected = 0
			sum_plucked  = 0
			sum_dropped  = 0

			# Blueberry
			no_of_blueberries =  int(required_configuration["Blueberry"].split("_")[0] )
			cb_of_blueberries =  required_configuration["Blueberry"].split("_")[1]

			sum_detected    = sum(1 for i in final_ci_list if i == "Blueberry")
			sum_plucked     = sum(1 for i in final_cp_list if i == "Blueberry")
			if cb_of_blueberries == 'CB1':
				cb1_allowed[0] = no_of_blueberries
				sum_dropped = sum(1 for i in final_cb1_drops if i == "Blueberry")
			else:
				cb2_allowed[0] = no_of_blueberries
				sum_dropped = sum(1 for i in final_cb2_drops if i == "Blueberry")

			if sum_detected == sum_plucked == sum_dropped == no_of_blueberries:
				bonus_valid_flag = bonus_valid_flag * True
			else:
				bonus_valid_flag = bonus_valid_flag * False


			sum_detected = 0
			sum_plucked  = 0
			sum_dropped  = 0

			# Lemon
			no_of_lemons =  int(required_configuration["Lemon"].split("_")[0] )
			cb_of_lemons =  required_configuration["Lemon"].split("_")[1]

			sum_detected    = sum(1 for i in final_ci_list if i == "Lemon")
			sum_plucked     = sum(1 for i in final_cp_list if i == "Lemon")
			if cb_of_lemons == 'CB1':
				cb1_allowed[1] = no_of_lemons
				sum_dropped = sum(1 for i in final_cb1_drops if i == "Lemon")
			else:
				cb2_allowed[1] = no_of_lemons
				sum_dropped = sum(1 for i in final_cb2_drops if i == "Lemon")

			if sum_detected == sum_plucked == sum_dropped == no_of_lemons:
				bonus_valid_flag = bonus_valid_flag * True
			else:
				bonus_valid_flag = bonus_valid_flag * False


			sum_detected = 0
			sum_plucked  = 0
			sum_dropped  = 0

			# Strawberry
			no_of_strawberries =  int(required_configuration["Strawberry"].split("_")[0] )
			cb_of_strawberries =  required_configuration["Strawberry"].split("_")[1]

			sum_detected    = sum(1 for i in final_ci_list if i == "Strawberry")
			sum_plucked     = sum(1 for i in final_cp_list if i == "Strawberry")
			if cb_of_strawberries == 'CB1':
				cb1_allowed[2] = no_of_strawberries
				sum_dropped = sum(1 for i in final_cb1_drops if i == "Strawberry")
			else:
				cb2_allowed[2] = no_of_strawberries
				sum_dropped = sum(1 for i in final_cb2_drops if i == "Strawberry")

			if sum_detected == sum_plucked == sum_dropped == no_of_strawberries:
				bonus_valid_flag = bonus_valid_flag * True
			else:
				bonus_valid_flag = bonus_valid_flag * False


			# Detecting extra drops in CB1 and CB2 and if found no bonus
			# Checking original drops in CB1 and CB2
			# print(cb1_allowed)
			# print(cb2_allowed)

			# CB1
			original_bb_sum_dropped = sum(1 for i in raw_cb1_drops_general_name if i == "Blueberry")
			original_l_sum_dropped  = sum(1 for i in raw_cb1_drops_general_name if i == "Lemon")
			original_sb_sum_dropped = sum(1 for i in raw_cb1_drops_general_name if i == "Strawberry")

			if original_bb_sum_dropped <= cb1_allowed[0] and original_l_sum_dropped <= cb1_allowed[1] and original_sb_sum_dropped <= cb1_allowed[2]:
				bonus_valid_flag = bonus_valid_flag * True
			else:
				bonus_valid_flag = bonus_valid_flag * False


			# CB2
			original_bb_sum_dropped = sum(1 for i in raw_cb2_drops_general_name if i == "Blueberry")
			original_l_sum_dropped  = sum(1 for i in raw_cb2_drops_general_name if i == "Lemon")
			original_sb_sum_dropped = sum(1 for i in raw_cb2_drops_general_name if i == "Strawberry")

			if original_bb_sum_dropped <= cb2_allowed[0] and original_l_sum_dropped <= cb2_allowed[1] and original_sb_sum_dropped <= cb2_allowed[2]:
				bonus_valid_flag = bonus_valid_flag * True
			else:
				bonus_valid_flag = bonus_valid_flag * False
			


		if bonus_valid_flag:
			B = 200
		else:
			B = 0

		score = (600 - T)   +   CI*10   +   CP*50   +   CD*50   -   P*30   +   B

		# print()
		# print("Correctly Identified   :", final_ci_list)
		# print("Correctly Plucked      :", final_cp_list)
		# print("Correctly Dropped      :", final_cd_list)

		# print()
		# print("Total Simulation Time  :", T)
		# print("CI                     :", CI)
		# print("CP                     :", CP)
		# print("CD                     :", CD)
		# print("P                      :", P)
		# print("B                      :", B)
		print("Task 6 Score           :", score)
		

	else:
		print("\nThe current run is an INVALID run.")
		T = 0
		CI = 0
		CP = 0
		CD = 0
		P = 0
		B = 0
		score = 0
		final_ci_list = 0
		final_cp_list = 0
		final_cd_list = 0
		final_cb1_drops = 0
		final_cb2_drops = 0
		print()
		print("Task 6 Score           :", score)


	data_to_return = [ valid_run_flag, T, CI, CP, CD, P ,B ,score, final_ci_list, final_cp_list, final_cd_list, final_cb1_drops, final_cb2_drops ]
		
	return data_to_return

def send_data_e_yantra_server(bm_task_key, data_from_calculate_score, gsheet_data):

	curr_date_time, platform_uname, collisions_list_to_show, mass_of_arm, end_simulation_time, rtf_python, no_of_joints, torque, force = gsheet_data

	submission_attempts=0

	flag_data_sent_successfully=0 # 0- Failed sending data, 1- Successfully sent data OR Master Flag in the sheet is 0 (Can be done manually ONLY).

	if config_type == "Original Configuration":
		spreadsheetname = "Real_Time_Performance_of_Teams_in_Task_6"
	elif config_type == "Bonus Configuration":
		spreadsheetname = "Bonus_Real_Time_Performance_of_Teams_in_Task_6"
	else:
		pass


	while submission_attempts<5:
		print('\n#################################################')
		print('\nAttempting to send data to e-Yantra Servers.')
		print('\nMake sure you have an active internet connection.')

		try:
			service_acc = gspread.service_account_from_dict(bm_task_key)
			g_sheet=service_acc.open(spreadsheetname)
			worksheet_list = g_sheet.worksheets()
			master_wksheet = g_sheet.worksheet('master')		
			if(master_wksheet.acell('Z1').value=='1'):

				list_of_worksheets=[]

				for obj in worksheet_list:
					list_of_worksheets.append(obj._properties['title'])


				if(str(team_id_val) not in list_of_worksheets):
					wksheet = g_sheet.add_worksheet(title=str(team_id_val), rows="10000", cols="20")
					wksheet.update('A1:W1',[headers]) #Add header row for the first time.
				else:
					wksheet = g_sheet.worksheet(str(team_id_val))


				row_num_to_write=len(wksheet.get_all_values())+1 # This will give number of rows which are filled.

				data_to_push = [curr_date_time                               , platform_uname                                , str(hex(uuid.getnode())),
								str(data_from_calculate_score[0])            , round(float(data_from_calculate_score[1]),2)  , round(float(data_from_calculate_score[2]),2),
								round(float(data_from_calculate_score[3]),2) , round(float(data_from_calculate_score[4]),2)  , round(float(data_from_calculate_score[5]),2),
								round(float(data_from_calculate_score[6]),2) , round(float(data_from_calculate_score[7]),2)  , str(data_from_calculate_score[8]),
								str(data_from_calculate_score[9])            , str(data_from_calculate_score[10])            , str(data_from_calculate_score[11]), str(data_from_calculate_score[12]),
								float(len(collisions_list_to_show))          , round(float(mass_of_arm),2)                   , round(float(end_simulation_time),2),
								rtf_python                                   , float(no_of_joints)                           , round(float(torque),2)                    ,
								round(float(force),2)
								]

				wksheet.update('A'+str(row_num_to_write)+':W'+str(row_num_to_write),[data_to_push])

				if(len(wksheet.get_all_values())+1==row_num_to_write+1): # Verifying if the row is filled or not.
					print("\nSuccessfully sent data to e-Yantra Servers.")
					print('\n#################################################')
					submission_attempts=5
					flag_data_sent_successfully=1

			else:
				submission_attempts=5
				flag_data_sent_successfully=1
				print('\nSubmission to server is not available now.')
				print('\n#################################################')
		
		except:
			submission_attempts+=1
			flag_data_sent_successfully=0
			traceback.print_exc(file=sys.stdout)
			# print()
			print('\nFAILED sending the data.')
			print('\nNumber of times remaining: ',5-submission_attempts)
			# input('\nPress any key to try again.')

	return flag_data_sent_successfully



class Ui_Berryminator_Evaluator(object):

	def openwindow(self):
		self.window = QtWidgets.QMainWindow()
		self.ui = Ui_Dialog()
		self.ui.setupUi(self.window)
		Berryminator_Evaluator.hide()
		self.window.show()

		# test_task_6_main(team_id_val, folder_path)
		# self.start_process()
		# self.process = QtCore.QProcess()
		# self.process.start("python", ['task_1a_cardinal.py', str(team_id_val), folder_path, task])

	# def start_process(self):
	#     global pid
	#     self.process = QtCore.QProcess()
	#     self.process.readyReadStandardOutput.connect(self.handle_stdout)
	#     self.process.readyReadStandardError.connect(self.handle_stderr)
	#     self.process.stateChanged.connect(self.handle_state)

	#     # os.kill(pid, signal.CTRL_C_EVENT)
	#     self.process.finished.connect(self.process_finished)  # Clean up once complete.

	#     print("Task 4 Task 4 Task 4")
	#     # print(folder_path)
	#     command = 'cmd.exe /C task_4_cardinal.exe ' + str(team_id_val) + ' "' + folder_path + '"'
	#     print(command)
	#     self.process.start(command)
	#     pid = self.process.processId()
	#     print("process pid" + str(pid))

	# def handle_stderr(self):
	#     data = self.process.readAllStandardError()
	#     stderr = bytes(data).decode("utf8")
	#     # self.message(stderr)
	#     print(stderr)

	# def handle_stdout(self):
	#     data = self.process.readAllStandardOutput()
	#     stdout = bytes(data).decode("utf8")
	#     # self.message(stdout)
	#     print(stdout)

	# def handle_state(self, state):
	#     states = {
	#         QtCore.QProcess.NotRunning: 'Not running',
	#         QtCore.QProcess.Starting: 'Starting',
	#         QtCore.QProcess.Running: 'Running',
	#     }
	#     state_name = states[state]
	#     # print(f"State changed: {state_name}")
	#     # self.message(f"State changed: {state_name}")

	# def process_finished(self):
	#     print("Process finished.")
	#     self.process = None


	def setupUi(self, Berryminator_Evaluator):
		Berryminator_Evaluator.setObjectName("Berryminator_Evaluator")
		Berryminator_Evaluator.resize(600, 550)
		sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
		sizePolicy.setHorizontalStretch(0)
		sizePolicy.setVerticalStretch(0)
		sizePolicy.setHeightForWidth(Berryminator_Evaluator.sizePolicy().hasHeightForWidth())
		Berryminator_Evaluator.setSizePolicy(sizePolicy)
		font = QtGui.QFont()
		font.setFamily("Calibri")
		Berryminator_Evaluator.setFont(font)
		Berryminator_Evaluator.setAutoFillBackground(False)
		Berryminator_Evaluator.setStyleSheet("background-color: rgb(255, 255, 255);")
		self.centralwidget = QtWidgets.QWidget(Berryminator_Evaluator)
		self.centralwidget.setObjectName("centralwidget")
		self.header_1_logo = QtWidgets.QLabel(self.centralwidget)
		self.header_1_logo.setGeometry(QtCore.QRect(10, 10, 245, 50))
		self.header_1_logo.setStyleSheet("")
		self.header_1_logo.setText("")
		self.header_1_logo.setPixmap(QtGui.QPixmap(resource_path("logo_eyantra.png")))
		self.header_1_logo.setScaledContents(True)
		self.header_1_logo.setObjectName("header_1_logo")
		self.header_2_comp = QtWidgets.QLabel(self.centralwidget)
		self.header_2_comp.setGeometry(QtCore.QRect(350, 10, 240, 50))
		# # self.header_2_comp.setFrameShape(QtWidgets.QFrame.Box)
		# self.header_2_comp.setText("")
		# self.header_2_comp.setPixmap(QtGui.QPixmap(resource_path("robotics_comp.png")))
		# self.header_2_comp.setScaledContents(True)
		# self.header_2_comp.setObjectName("header_2_comp")
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.header_2_comp.setFont(font)
		self.header_2_comp.setScaledContents(False)
		self.header_2_comp.setAlignment(QtCore.Qt.AlignCenter)
		self.header_2_comp.setWordWrap(True)
		self.header_2_comp.setIndent(0)
		self.header_2_comp.setObjectName("header_2_comp")
		self.header_3_theme_img = QtWidgets.QLabel(self.centralwidget)
		self.header_3_theme_img.setGeometry(QtCore.QRect(100, 80, 400, 190))
		self.header_3_theme_img.setFrameShape(QtWidgets.QFrame.Box)
		self.header_3_theme_img.setText("")
		self.header_3_theme_img.setPixmap(QtGui.QPixmap(resource_path("berryminator.png")))
		self.header_3_theme_img.setScaledContents(True)
		self.header_3_theme_img.setObjectName("header_3_theme_img")
		self.header_4_theme_name = QtWidgets.QLabel(self.centralwidget)
		self.header_4_theme_name.setGeometry(QtCore.QRect(150, 270, 291, 41))
		# self.header_4_theme_name.setText("")
		# self.header_4_theme_name.setPixmap(QtGui.QPixmap(resource_path("berryminator_theme.png")))
		# self.header_4_theme_name.setScaledContents(True)
		# self.header_4_theme_name.setObjectName("header_4_theme_name")
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(24)
		font.setBold(True)
		font.setWeight(75)
		self.header_4_theme_name.setFont(font)
		self.header_4_theme_name.setStyleSheet("color: rgb(255, 0, 0);")
		self.header_4_theme_name.setScaledContents(False)
		self.header_4_theme_name.setAlignment(QtCore.Qt.AlignCenter)
		self.header_4_theme_name.setWordWrap(True)
		self.header_4_theme_name.setIndent(0)
		self.header_4_theme_name.setObjectName("header_4_theme_name")
		self.folder_display = QtWidgets.QLabel(self.centralwidget)
		self.folder_display.setGeometry(QtCore.QRect(180, 440, 390, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.folder_display.setFont(font)
		# self.folder_display.setStyleSheet("color: rgb(190, 190, 190);")
		self.folder_display.setFrameShape(QtWidgets.QFrame.Box)
		self.folder_display.setText("")
		self.folder_display.setObjectName("folder_display")
		self.select_folder = QtWidgets.QPushButton(self.centralwidget)
		self.select_folder.setGeometry(QtCore.QRect(20, 440, 140, 30))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.select_folder.setFont(font)
		self.select_folder.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
		self.select_folder.setObjectName("select_folder")
		self.team_id = QtWidgets.QLineEdit(self.centralwidget)
		self.team_id.setGeometry(QtCore.QRect(285, 360, 150, 30))
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.team_id.setFont(font)
		self.team_id.setObjectName("team_id")
		self.Enter_team_id = QtWidgets.QLabel(self.centralwidget)
		self.Enter_team_id.setGeometry(QtCore.QRect(160, 360, 120, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(19)
		font.setBold(True)
		font.setWeight(75)
		self.Enter_team_id.setFont(font)
		self.Enter_team_id.setObjectName("Enter_team_id")
		self.start_evaluation = QtWidgets.QPushButton(self.centralwidget)
		self.start_evaluation.setGeometry(QtCore.QRect(220, 480, 160, 40))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.start_evaluation.setFont(font)
		self.start_evaluation.setStyleSheet("background-color: rgb(255, 0, 0);\n"
"color: rgb(255, 255, 255);")
		self.start_evaluation.setObjectName("start_evaluation")
		self.header_4_theme_name_2 = QtWidgets.QLabel(self.centralwidget)
		self.header_4_theme_name_2.setGeometry(QtCore.QRect(150, 310, 291, 41))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(30)
		font.setBold(True)
		font.setWeight(75)
		self.header_4_theme_name_2.setFont(font)
		self.header_4_theme_name_2.setStyleSheet("color: rgb(0, 0, 0);")
		self.header_4_theme_name_2.setScaledContents(False)
		self.header_4_theme_name_2.setAlignment(QtCore.Qt.AlignCenter)
		self.header_4_theme_name_2.setWordWrap(True)
		self.header_4_theme_name_2.setIndent(0)
		self.header_4_theme_name_2.setObjectName("header_4_theme_name_2")
		self.Enter_team_id_2 = QtWidgets.QLabel(self.centralwidget)
		self.Enter_team_id_2.setGeometry(QtCore.QRect(100, 400, 180, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPointSize(14)
		font.setBold(True)
		font.setWeight(75)
		self.Enter_team_id_2.setFont(font)
		self.Enter_team_id_2.setObjectName("Enter_team_id_2")
		self.comboBox = QtWidgets.QComboBox(self.centralwidget)
		self.comboBox.setGeometry(QtCore.QRect(285, 400, 180, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPointSize(12)
		font.setBold(True)
		font.setItalic(False)
		font.setWeight(75)
		font.setStrikeOut(False)
		self.comboBox.setFont(font)
		self.comboBox.setObjectName("comboBox")
		self.comboBox.addItem("")
		self.comboBox.addItem("")
		Berryminator_Evaluator.setCentralWidget(self.centralwidget)
		self.statusbar = QtWidgets.QStatusBar(Berryminator_Evaluator)
		self.statusbar.setObjectName("statusbar")
		Berryminator_Evaluator.setStatusBar(self.statusbar)

		self.retranslateUi(Berryminator_Evaluator)
		QtCore.QMetaObject.connectSlotsByName(Berryminator_Evaluator)
		self.select_folder.clicked.connect(self.pick_new)
		self.start_evaluation.clicked.connect(self.start_eval)

	def retranslateUi(self, Berryminator_Evaluator):
		_translate = QtCore.QCoreApplication.translate
		Berryminator_Evaluator.setWindowTitle(_translate("Berryminator_Evaluator", "BM_GUI"))
		self.header_2_comp.setText(_translate("Berryminator_Evaluator", "Robotics Competition 2021-22"))
		self.header_4_theme_name.setText(_translate("Berryminator_Evaluator", "Berryminator Theme"))
		# self.folder_display.setText(_translate("Berryminator_Evaluator", "Select Folder"))
		self.select_folder.setText(_translate("Berryminator_Evaluator", "Select Folder"))
		self.Enter_team_id.setText(_translate("Berryminator_Evaluator", "Enter Team Id:"))
		self.start_evaluation.setText(_translate("Berryminator_Evaluator", "Start Evaluation"))
		self.start_evaluation.setShortcut(_translate("Berryminator_Evaluator", "Return"))
		self.header_4_theme_name_2.setText(_translate("Berryminator_Evaluator", "Task 6"))
		self.Enter_team_id_2.setText(_translate("Berryminator_Evaluator", "Choose Configuration:"))
		self.comboBox.setItemText(0, _translate("Berryminator_Evaluator", "Original Configuration"))
		self.comboBox.setItemText(1, _translate("Berryminator_Evaluator", "Bonus Configuration"))

	def pick_new(self):
				dialog = QtWidgets.QFileDialog()
				folder_path = dialog.getExistingDirectory(None, "Select Folder")
				self.folder_display.setText(folder_path)

	def start_eval(self):
				global team_id_val, folder_path, config_type, res_path
				res_path = resource_path('')
				value = self.team_id.text()
				folder_path = self.folder_display.text()
				config_type = self.comboBox.currentText()

				team_id_val = None
				folder_path_valid_flag = None
				team_id_valid_flag = None
				config_exists_flag = None
				if folder_path == '':
					folder_path_valid_flag = False
				else:
					folder_path_valid_flag = True

				try:
					team_id_val = int(value)
					team_id_valid_flag = True
				except:
					team_id_valid_flag = False

				if config_type == "Original Configuration":
					if folder_path_valid_flag == True:
						if os.path.exists(folder_path + "/Theme_Config_Original.json"):
							config_exists_flag = True
						else:
							config_exists_flag = False

					else:
						config_exists_flag = False


				elif config_type == "Bonus Configuration":
					if folder_path_valid_flag == True:
						if os.path.exists(folder_path + "/Theme_Config_Bonus.json"):
							config_exists_flag = True
						else:
							config_exists_flag = False

					else:
						config_exists_flag = False

				else:
					config_exists_flag = False


				msg_box = QtWidgets.QMessageBox()
				msg_box.setWindowTitle("Error!!")
				msg_box.setIcon(QtWidgets.QMessageBox.Critical)

				try:
				
					if team_id_valid_flag == False and folder_path_valid_flag == False:
							msg_box.setText('Valid Team ID and Folder Path not selected!')
							x = msg_box.exec()
							# sys.exit()
					elif team_id_valid_flag == True and folder_path_valid_flag == False:
							msg_box.setText('Valid Folder Path not selected!')
							x = msg_box.exec()
							# sys.exit()
					elif team_id_valid_flag == False and folder_path_valid_flag == True:
							msg_box.setText('Valid Team ID not selected!')
							x = msg_box.exec()
							# sys.exit()
					elif team_id_valid_flag == True and folder_path_valid_flag == True:
							if config_exists_flag == False:
								msg_box.setText('Correct Theme Configuration file does not exist in selected folder')
								x = msg_box.exec()
							else:
								self.openwindow()
					else:
							pass             

					# self.openwindow()
				except:
					# print("hello")
					pass

class Ui_Dialog(object):
	def setupUi(self, Dialog):
		Dialog.setObjectName("Dialog")
		Dialog.resize(770, 540)


		# sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
		# sizePolicy.setHorizontalStretch(0)
		# sizePolicy.setVerticalStretch(0)
		# sizePolicy.setHeightForWidth(Dialog.sizePolicy().hasHeightForWidth())
		# Dialog.setSizePolicy(sizePolicy)
		# Dialog.setMinimumSize(QtCore.QSize(770, 540))
		# Dialog.setMaximumSize(QtCore.QSize(770, 540))

		Dialog.setStyleSheet("background-color: rgb(255, 255, 255);")
		self.header1 = QtWidgets.QLabel(Dialog)
		self.header1.setGeometry(QtCore.QRect(10, 10, 245, 50))
		self.header1.setStyleSheet("")
		self.header1.setText("")
		self.header1.setPixmap(QtGui.QPixmap(resource_path("logo_eyantra.png")))
		self.header1.setScaledContents(True)
		self.header1.setObjectName("header1")
		self.header3 = QtWidgets.QLabel(Dialog)
		self.header3.setGeometry(QtCore.QRect(520, 10, 240, 50))
		# self.header_2_comp.setFrameShape(QtWidgets.QFrame.Box)
		# self.header3.setText("")
		# self.header3.setPixmap(QtGui.QPixmap(resource_path("robotics_comp.png")))
		# self.header3.setScaledContents(True)
		# self.header3.setObjectName("header3")
		self.header3 = QtWidgets.QLabel(Dialog)
		self.header3.setGeometry(QtCore.QRect(520, 10, 240, 50))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.header3.setFont(font)
		self.header3.setScaledContents(False)
		self.header3.setAlignment(QtCore.Qt.AlignCenter)
		self.header3.setWordWrap(True)
		self.header3.setIndent(0)
		self.header3.setObjectName("header3")
		self.console_output = QtWidgets.QTextEdit(Dialog)
		self.console_output.setGeometry(QtCore.QRect(330, 240, 420, 230))
		self.console_output.setObjectName("console_output")
		self.label_output_console = QtWidgets.QLabel(Dialog)
		self.label_output_console.setGeometry(QtCore.QRect(330, 200, 151, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.label_output_console.setFont(font)
		self.label_output_console.setObjectName("label_output_console")
		self.end_process = QtWidgets.QPushButton(Dialog)
		self.end_process.setGeometry(QtCore.QRect(530, 490, 130, 30))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.end_process.setFont(font)
		self.end_process.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
		self.end_process.setObjectName("end_process")
		self.exit = QtWidgets.QPushButton(Dialog)
		self.exit.setGeometry(QtCore.QRect(680, 490, 70, 30))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.exit.setFont(font)
		self.exit.setStyleSheet("background-color: rgb(255, 255, 0);\n"
"color: rgb(0, 0, 0);")
		self.exit.setObjectName("exit")
		self.label_theme_config = QtWidgets.QLabel(Dialog)
		self.label_theme_config.setGeometry(QtCore.QRect(20, 80, 261, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.label_theme_config.setFont(font)
		self.label_theme_config.setFrameShape(QtWidgets.QFrame.Box)
		self.label_theme_config.setAlignment(QtCore.Qt.AlignCenter)
		self.label_theme_config.setObjectName("label_theme_config")
		self.label_blueberry = QtWidgets.QLabel(Dialog)
		self.label_blueberry.setGeometry(QtCore.QRect(20, 110, 101, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_blueberry.setFont(font)
		self.label_blueberry.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.label_blueberry.setStyleSheet("")
		self.label_blueberry.setFrameShape(QtWidgets.QFrame.Box)
		self.label_blueberry.setAlignment(QtCore.Qt.AlignCenter)
		self.label_blueberry.setObjectName("label_blueberry")
		self.label_strawberry = QtWidgets.QLabel(Dialog)
		self.label_strawberry.setGeometry(QtCore.QRect(20, 150, 101, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_strawberry.setFont(font)
		self.label_strawberry.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.label_strawberry.setStyleSheet("")
		self.label_strawberry.setFrameShape(QtWidgets.QFrame.Box)
		self.label_strawberry.setAlignment(QtCore.Qt.AlignCenter)
		self.label_strawberry.setObjectName("label_strawberry")
		self.label_lemon = QtWidgets.QLabel(Dialog)
		self.label_lemon.setGeometry(QtCore.QRect(20, 190, 101, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_lemon.setFont(font)
		self.label_lemon.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.label_lemon.setStyleSheet("")
		self.label_lemon.setFrameShape(QtWidgets.QFrame.Box)
		self.label_lemon.setAlignment(QtCore.Qt.AlignCenter)
		self.label_lemon.setObjectName("label_lemon")
		self.blueberry1 = QtWidgets.QLabel(Dialog)
		self.blueberry1.setGeometry(QtCore.QRect(120, 110, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.blueberry1.setFont(font)
		self.blueberry1.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.blueberry1.setStyleSheet("")
		self.blueberry1.setFrameShape(QtWidgets.QFrame.Box)
		self.blueberry1.setText("")
		# self.blueberry1.setPixmap(QtGui.QPixmap("blueberry.png"))
		self.blueberry1.setScaledContents(True)
		self.blueberry1.setAlignment(QtCore.Qt.AlignCenter)
		self.blueberry1.setObjectName("blueberry1")
		self.blueberry2 = QtWidgets.QLabel(Dialog)
		self.blueberry2.setGeometry(QtCore.QRect(160, 110, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.blueberry2.setFont(font)
		self.blueberry2.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.blueberry2.setStyleSheet("")
		self.blueberry2.setFrameShape(QtWidgets.QFrame.Box)
		self.blueberry2.setText("")
		# self.blueberry2.setPixmap(QtGui.QPixmap("blueberry.png"))
		self.blueberry2.setScaledContents(True)
		self.blueberry2.setAlignment(QtCore.Qt.AlignCenter)
		self.blueberry2.setObjectName("blueberry2")
		self.blueberry3 = QtWidgets.QLabel(Dialog)
		self.blueberry3.setGeometry(QtCore.QRect(200, 110, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.blueberry3.setFont(font)
		self.blueberry3.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.blueberry3.setStyleSheet("")
		self.blueberry3.setFrameShape(QtWidgets.QFrame.Box)
		self.blueberry3.setText("")
		# self.blueberry3.setPixmap(QtGui.QPixmap("blueberry.png"))
		self.blueberry3.setScaledContents(True)
		self.blueberry3.setAlignment(QtCore.Qt.AlignCenter)
		self.blueberry3.setObjectName("blueberry3")
		self.strawberry1 = QtWidgets.QLabel(Dialog)
		self.strawberry1.setGeometry(QtCore.QRect(120, 150, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.strawberry1.setFont(font)
		self.strawberry1.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.strawberry1.setStyleSheet("")
		self.strawberry1.setFrameShape(QtWidgets.QFrame.Box)
		self.strawberry1.setText("")
		# self.strawberry1.setPixmap(QtGui.QPixmap("strawberry.png"))
		self.strawberry1.setScaledContents(True)
		self.strawberry1.setAlignment(QtCore.Qt.AlignCenter)
		self.strawberry1.setObjectName("strawberry1")
		self.lemon1 = QtWidgets.QLabel(Dialog)
		self.lemon1.setGeometry(QtCore.QRect(120, 190, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.lemon1.setFont(font)
		self.lemon1.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.lemon1.setStyleSheet("")
		self.lemon1.setFrameShape(QtWidgets.QFrame.Box)
		self.lemon1.setText("")
		# self.lemon1.setPixmap(QtGui.QPixmap("lemon.png"))
		self.lemon1.setScaledContents(True)
		self.lemon1.setAlignment(QtCore.Qt.AlignCenter)
		self.lemon1.setObjectName("lemon1")
		self.strawberry2 = QtWidgets.QLabel(Dialog)
		self.strawberry2.setGeometry(QtCore.QRect(160, 150, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.strawberry2.setFont(font)
		self.strawberry2.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.strawberry2.setStyleSheet("")
		self.strawberry2.setFrameShape(QtWidgets.QFrame.Box)
		self.strawberry2.setText("")
		# self.strawberry2.setPixmap(QtGui.QPixmap("strawberry.png"))
		self.strawberry2.setScaledContents(True)
		self.strawberry2.setAlignment(QtCore.Qt.AlignCenter)
		self.strawberry2.setObjectName("strawberry2")
		self.strawberry3 = QtWidgets.QLabel(Dialog)
		self.strawberry3.setGeometry(QtCore.QRect(200, 150, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.strawberry3.setFont(font)
		self.strawberry3.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.strawberry3.setStyleSheet("")
		self.strawberry3.setFrameShape(QtWidgets.QFrame.Box)
		self.strawberry3.setText("")
		# self.strawberry3.setPixmap(QtGui.QPixmap("strawberry.png"))
		self.strawberry3.setScaledContents(True)
		self.strawberry3.setAlignment(QtCore.Qt.AlignCenter)
		self.strawberry3.setObjectName("strawberry3")
		self.lemon2 = QtWidgets.QLabel(Dialog)
		self.lemon2.setGeometry(QtCore.QRect(160, 190, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.lemon2.setFont(font)
		self.lemon2.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.lemon2.setStyleSheet("")
		self.lemon2.setFrameShape(QtWidgets.QFrame.Box)
		self.lemon2.setText("")
		# self.lemon2.setPixmap(QtGui.QPixmap("lemon.png"))
		self.lemon2.setScaledContents(True)
		self.lemon2.setAlignment(QtCore.Qt.AlignCenter)
		self.lemon2.setObjectName("lemon2")
		self.lemon3 = QtWidgets.QLabel(Dialog)
		self.lemon3.setGeometry(QtCore.QRect(200, 190, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.lemon3.setFont(font)
		self.lemon3.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.lemon3.setStyleSheet("")
		self.lemon3.setFrameShape(QtWidgets.QFrame.Box)
		self.lemon3.setText("")
		# self.lemon3.setPixmap(QtGui.QPixmap("lemon.png"))
		self.lemon3.setScaledContents(True)
		self.lemon3.setAlignment(QtCore.Qt.AlignCenter)
		self.lemon3.setObjectName("lemon3")
		self.blueberry_box = QtWidgets.QLabel(Dialog)
		self.blueberry_box.setGeometry(QtCore.QRect(240, 110, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.blueberry_box.setFont(font)
		self.blueberry_box.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.blueberry_box.setStyleSheet("")
		self.blueberry_box.setFrameShape(QtWidgets.QFrame.Box)
		self.blueberry_box.setAlignment(QtCore.Qt.AlignCenter)
		self.blueberry_box.setObjectName("blueberry_box")
		self.strawberry_box = QtWidgets.QLabel(Dialog)
		self.strawberry_box.setGeometry(QtCore.QRect(240, 150, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.strawberry_box.setFont(font)
		self.strawberry_box.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.strawberry_box.setStyleSheet("")
		self.strawberry_box.setFrameShape(QtWidgets.QFrame.Box)
		self.strawberry_box.setAlignment(QtCore.Qt.AlignCenter)
		self.strawberry_box.setObjectName("strawberry_box")
		self.lemon_box = QtWidgets.QLabel(Dialog)
		self.lemon_box.setGeometry(QtCore.QRect(240, 190, 41, 41))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.lemon_box.setFont(font)
		self.lemon_box.setLayoutDirection(QtCore.Qt.LeftToRight)
		self.lemon_box.setStyleSheet("")
		self.lemon_box.setFrameShape(QtWidgets.QFrame.Box)
		self.lemon_box.setAlignment(QtCore.Qt.AlignCenter)
		self.lemon_box.setObjectName("lemon_box")
		self.label_ci = QtWidgets.QLabel(Dialog)
		self.label_ci.setGeometry(QtCore.QRect(20, 320, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_ci.setFont(font)
		self.label_ci.setFrameShape(QtWidgets.QFrame.Box)
		self.label_ci.setAlignment(QtCore.Qt.AlignCenter)
		self.label_ci.setObjectName("label_ci")
		self.label_cp = QtWidgets.QLabel(Dialog)
		self.label_cp.setGeometry(QtCore.QRect(20, 350, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_cp.setFont(font)
		self.label_cp.setFrameShape(QtWidgets.QFrame.Box)
		self.label_cp.setAlignment(QtCore.Qt.AlignCenter)
		self.label_cp.setObjectName("label_cp")
		self.label_cd = QtWidgets.QLabel(Dialog)
		self.label_cd.setGeometry(QtCore.QRect(20, 380, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_cd.setFont(font)
		self.label_cd.setFrameShape(QtWidgets.QFrame.Box)
		self.label_cd.setAlignment(QtCore.Qt.AlignCenter)
		self.label_cd.setObjectName("label_cd")
		self.label_time = QtWidgets.QLabel(Dialog)
		self.label_time.setGeometry(QtCore.QRect(20, 290, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_time.setFont(font)
		self.label_time.setFrameShape(QtWidgets.QFrame.Box)
		self.label_time.setAlignment(QtCore.Qt.AlignCenter)
		self.label_time.setObjectName("label_time")
		self.label_penalty = QtWidgets.QLabel(Dialog)
		self.label_penalty.setGeometry(QtCore.QRect(20, 410, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_penalty.setFont(font)
		self.label_penalty.setFrameShape(QtWidgets.QFrame.Box)
		self.label_penalty.setAlignment(QtCore.Qt.AlignCenter)
		self.label_penalty.setObjectName("label_penalty")
		self.label_bonus = QtWidgets.QLabel(Dialog)
		self.label_bonus.setGeometry(QtCore.QRect(20, 440, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_bonus.setFont(font)
		self.label_bonus.setFrameShape(QtWidgets.QFrame.Box)
		self.label_bonus.setAlignment(QtCore.Qt.AlignCenter)
		self.label_bonus.setObjectName("label_bonus")

		self.label_valid_run = QtWidgets.QLabel(Dialog)
		self.label_valid_run.setGeometry(QtCore.QRect(20, 260, 211, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.label_valid_run.setFont(font)
		self.label_valid_run.setFrameShape(QtWidgets.QFrame.Box)
		self.label_valid_run.setAlignment(QtCore.Qt.AlignCenter)
		self.label_valid_run.setObjectName("label_valid_run")

		self.valid_run = QtWidgets.QLabel(Dialog)
		self.valid_run.setGeometry(QtCore.QRect(230, 260, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.valid_run.setFont(font)
		self.valid_run.setFrameShape(QtWidgets.QFrame.Box)
		self.valid_run.setAlignment(QtCore.Qt.AlignCenter)
		self.valid_run.setObjectName("valid_run")

		self.num_seconds = QtWidgets.QLabel(Dialog)
		self.num_seconds.setGeometry(QtCore.QRect(230, 290, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.num_seconds.setFont(font)
		self.num_seconds.setFrameShape(QtWidgets.QFrame.Box)
		self.num_seconds.setAlignment(QtCore.Qt.AlignCenter)
		self.num_seconds.setObjectName("num_seconds")
		self.num_ci = QtWidgets.QLabel(Dialog)
		self.num_ci.setGeometry(QtCore.QRect(230, 320, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.num_ci.setFont(font)
		self.num_ci.setFrameShape(QtWidgets.QFrame.Box)
		self.num_ci.setAlignment(QtCore.Qt.AlignCenter)
		self.num_ci.setObjectName("num_ci")
		self.num_cp = QtWidgets.QLabel(Dialog)
		self.num_cp.setGeometry(QtCore.QRect(230, 350, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.num_cp.setFont(font)
		self.num_cp.setFrameShape(QtWidgets.QFrame.Box)
		self.num_cp.setAlignment(QtCore.Qt.AlignCenter)
		self.num_cp.setObjectName("num_cp")
		self.num_cd = QtWidgets.QLabel(Dialog)
		self.num_cd.setGeometry(QtCore.QRect(230, 380, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.num_cd.setFont(font)
		self.num_cd.setFrameShape(QtWidgets.QFrame.Box)
		self.num_cd.setAlignment(QtCore.Qt.AlignCenter)
		self.num_cd.setObjectName("num_cd")
		self.num_penalty = QtWidgets.QLabel(Dialog)
		self.num_penalty.setGeometry(QtCore.QRect(230, 410, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.num_penalty.setFont(font)
		self.num_penalty.setFrameShape(QtWidgets.QFrame.Box)
		self.num_penalty.setAlignment(QtCore.Qt.AlignCenter)
		self.num_penalty.setObjectName("num_penalty")
		self.bonus = QtWidgets.QLabel(Dialog)
		self.bonus.setGeometry(QtCore.QRect(230, 440, 65, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(18)
		font.setBold(True)
		font.setWeight(75)
		self.bonus.setFont(font)
		self.bonus.setFrameShape(QtWidgets.QFrame.Box)
		self.bonus.setAlignment(QtCore.Qt.AlignCenter)
		self.bonus.setObjectName("bonus")
		self.label_score = QtWidgets.QLabel(Dialog)
		self.label_score.setGeometry(QtCore.QRect(160, 490, 161, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(28)
		font.setBold(True)
		font.setWeight(75)
		self.label_score.setFont(font)
		self.label_score.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.label_score.setAlignment(QtCore.Qt.AlignCenter)
		self.label_score.setObjectName("label_score")
		self.final_score = QtWidgets.QLabel(Dialog)
		self.final_score.setGeometry(QtCore.QRect(320, 490, 91, 31))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(28)
		font.setBold(True)
		font.setWeight(75)
		self.final_score.setFont(font)
		self.final_score.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.final_score.setAlignment(QtCore.Qt.AlignCenter)
		self.final_score.setObjectName("final_score")
		self.header2 = QtWidgets.QLabel(Dialog)
		self.header2.setGeometry(QtCore.QRect(330, 10, 130, 40))
		font = QtGui.QFont()
		font.setFamily("HighlandGothicFLF")
		font.setPixelSize(30)
		font.setBold(True)
		font.setUnderline(False)
		font.setWeight(75)
		font.setStrikeOut(False)
		self.header2.setFont(font)
		self.header2.setStyleSheet("color: rgb(0, 0, 0);")
		self.header2.setScaledContents(False)
		self.header2.setAlignment(QtCore.Qt.AlignCenter)
		self.header2.setWordWrap(True)
		self.header2.setIndent(0)
		self.header2.setObjectName("header2")
		self.teamid = QtWidgets.QLabel(Dialog)
		self.teamid.setGeometry(QtCore.QRect(550, 85, 80, 30))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(24)
		font.setBold(True)
		font.setWeight(75)
		self.teamid.setFont(font)
		self.teamid.setStyleSheet("color: rgb(255, 0, 0);")
		self.teamid.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.teamid.setAlignment(QtCore.Qt.AlignCenter)
		self.teamid.setObjectName("teamid")
		self.label_college_name = QtWidgets.QLabel(Dialog)
		self.label_college_name.setGeometry(QtCore.QRect(290, 130, 130, 25))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.label_college_name.setFont(font)
		self.label_college_name.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.label_college_name.setAlignment(QtCore.Qt.AlignCenter)
		self.label_college_name.setWordWrap(True)
		self.label_college_name.setObjectName("label_college_name")
		self.college_name = QtWidgets.QLabel(Dialog)
		self.college_name.setGeometry(QtCore.QRect(450, 130, 280, 70))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.college_name.setFont(font)
		self.college_name.setStyleSheet("color: rgb(255, 0, 0);")
		self.college_name.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.college_name.setTextFormat(QtCore.Qt.AutoText)
		self.college_name.setAlignment(QtCore.Qt.AlignHCenter)
		self.college_name.setWordWrap(True)
		self.college_name.setObjectName("college_name")
		self.label_teamid = QtWidgets.QLabel(Dialog)
		self.label_teamid.setGeometry(QtCore.QRect(290, 90, 130, 20))
		font = QtGui.QFont()
		font.setFamily("Calibri")
		font.setPixelSize(20)
		font.setBold(True)
		font.setWeight(75)
		self.label_teamid.setFont(font)
		self.label_teamid.setFrameShape(QtWidgets.QFrame.NoFrame)
		self.label_teamid.setAlignment(QtCore.Qt.AlignCenter)
		self.label_teamid.setWordWrap(True)
		self.label_teamid.setObjectName("label_teamid")

		sys.stdout = EmittingStream(textWritten=self.output_terminal_written)
		self.retranslateUi(Dialog)
		QtCore.QMetaObject.connectSlotsByName(Dialog)
		self.exit.clicked.connect(self.exit_btn)
		self.end_process.clicked.connect(self.end_process_btn)

		global theme_config
		if config_type == "Original Configuration":
			theme_config = json.load(open(folder_path + "/Theme_Config_Original.json"))
		elif config_type == "Bonus Configuration":
			theme_config = json.load(open(folder_path + "/Theme_Config_Bonus.json"))
		else:
			pass
		self.show_theme_config(theme_config)
		self.show_team_details(team_id_val)
		# self.show_evaluation_parameters(folder_path + "/theme_implementation_result.txt")

		print("Deleting existing theme_implementation_result.txt(if applicable)")

		if os.path.exists(folder_path + "/theme_implementation_result.txt"):
			os.remove(folder_path + "/theme_implementation_result.txt")

		self.start_process()



	def retranslateUi(self, Dialog):
		_translate = QtCore.QCoreApplication.translate
		Dialog.setWindowTitle(_translate("Dialog", "BM_GUI"))
		self.header3.setText(_translate("Dialog", "Robotics Competition 2021-22"))
		self.label_output_console.setText(_translate("Dialog", "Output Console"))
		self.end_process.setText(_translate("Dialog", "End Process"))
		self.exit.setText(_translate("Dialog", "Exit"))
		self.label_theme_config.setText(_translate("Dialog", "Theme Configuration"))
		self.label_blueberry.setText(_translate("Dialog", "Blueberry"))
		self.label_strawberry.setText(_translate("Dialog", "Strawberry"))
		self.label_lemon.setText(_translate("Dialog", "Lemon"))
		self.blueberry_box.setText(_translate("Dialog", "CB1"))
		self.strawberry_box.setText(_translate("Dialog", "CB2"))
		self.lemon_box.setText(_translate("Dialog", "CB1"))
		self.label_ci.setText(_translate("Dialog", "Correct Identification (CI)"))
		self.label_cp.setText(_translate("Dialog", "Correct Pluck (CP)"))
		self.label_cd.setText(_translate("Dialog", "Correct Deposition (CD)"))
		self.label_time.setText(_translate("Dialog", "Time (in sec) (T)"))
		self.label_penalty.setText(_translate("Dialog", "Penalties (P)"))
		self.label_bonus.setText(_translate("Dialog", "Bonus (B)"))
		self.label_valid_run.setText(_translate("Dialog", "Valid Run"))
		self.valid_run.setText(_translate("Dialog", "N/A"))
		self.num_seconds.setText(_translate("Dialog", "0"))
		self.num_ci.setText(_translate("Dialog", "0"))
		self.num_cp.setText(_translate("Dialog", "0"))
		self.num_cd.setText(_translate("Dialog", "0"))
		self.num_penalty.setText(_translate("Dialog", "0"))
		self.bonus.setText(_translate("Dialog", "0"))
		self.label_score.setText(_translate("Dialog", "Total Score:"))
		self.final_score.setText(_translate("Dialog", "0000.00"))
		self.header2.setText(_translate("Dialog", "TASK 6"))
		self.teamid.setText(_translate("Dialog", "0000"))
		self.label_college_name.setText(_translate("Dialog", "College Name:"))
		self.college_name.setText(_translate("Dialog", "Pimpri Chinchwad Education Trust\'s Pimpri Chinchwad College of Engineering"))
		self.label_teamid.setText(_translate("Dialog", "Team ID:"))


	def start_process(self):


		# thread = Thread()
		# app.aboutToQuit(thread.stop)
		# thread.start()
		
		# self.thread = QtCore.QThread()
		# self.worker = Worker()
		# self.worker.moveToThread(self.thread)

		# self.thread.started.connect(self.worker.run)
		# self.worker.finished.connect(self.thread.quit)
		# self.worker.finished.connect(self.worker.deleteLater)
		# self.thread.finished.connect(self.thread.deleteLater)
		# self.worker.progress.connect(self.reportProgress)

		# self.thread.start()

		# test_task_6_main(str(team_id_val), folder_path)
		global pid
		self.process = QtCore.QProcess()
		self.process.readyReadStandardOutput.connect(self.handle_stdout)
		self.process.readyReadStandardError.connect(self.handle_stderr)
		self.process.stateChanged.connect(self.handle_state)
		# os.kill(pid, signal.CTRL_C_EVENT)
		self.process.finished.connect(self.process_finished)  # Clean up once complete.
		# print(res_path)

		if config_type == "Original Configuration":
			command = 'test_task_6_original.exe ' + str(team_id_val) + ' "' + folder_path + '" "' + res_path + '"'
			self.process.start(command)
			# self.process.start("python", ["test_task_6_original.py", str(team_id_val), folder_path, res_path])
		elif config_type == "Bonus Configuration":
			command = 'test_task_6_bonus.exe ' + str(team_id_val) + ' "' + folder_path + '" "' + res_path + '"'
			self.process.start(command)
			# self.process.start("python", ["test_task_6_bonus.py", str(team_id_val), folder_path, res_path])
		else:
			pass

		# print("Task 5 Task 5 Task 5")
		# print(folder_path)
		# command = 'cmd.exe /C task_5_cardinal.exe ' + str(team_id_val) + ' "' + folder_path + '"'
		# command = 'test_task_6.exe ' + str(team_id_val) + ' "' + folder_path + '"'

		# print(command)
		# self.process.start(command)
		# print(os.getpid())
		# pid = self.process.processId()
		# print("process pid " + str(pid))

	def handle_stderr(self):
		data = self.process.readAllStandardError()
		stderr = bytes(data).decode("utf8")
		# self.message(stderr)
		print(stderr)
		
	def handle_stdout(self):
		data = self.process.readAllStandardOutput()
		stdout = bytes(data).decode("utf8")
		# self.message(stdout)
		print(stdout)

	def handle_state(self, state):
		states = {
			QtCore.QProcess.NotRunning: 'Not running',
			QtCore.QProcess.Starting: 'Starting',
			QtCore.QProcess.Running: 'Running',
		}
		state_name = states[state]
		# print(f"State changed: {state_name}")
		# self.message(f"State changed: {state_name}")

	def process_finished(self):
		print("Process finished.")

		if os.path.exists(folder_path + "/theme_implementation_result.txt"):
			self.show_evaluation_parameters(folder_path + "/theme_implementation_result.txt")
		else:
			print("theme_implementation_result.txt doesn't exist in specified folder")
		self.process = None

	def show_theme_config(self, theme_config):
		blueberry = theme_config["B"]
		strawberry = theme_config["S"]
		lemon = theme_config["L"]
		blueberry_num, blueberry_collection_box = int(blueberry[0]), blueberry[2:]
		strawberry_num, strawberry_collection_box = int(strawberry[0]), strawberry[2:]
		lemon_num, lemon_collection_box = int(lemon[0]), lemon[2:]

		self.blueberry_box.setText(blueberry_collection_box)
		self.strawberry_box.setText(strawberry_collection_box)
		self.lemon_box.setText(lemon_collection_box)

		self.blueberry1.setPixmap(QtGui.QPixmap(resource_path("blueberry.png")))
		self.strawberry1.setPixmap(QtGui.QPixmap(resource_path("strawberry.png")))
		self.lemon1.setPixmap(QtGui.QPixmap(resource_path("lemon.png")))

		self.blueberry2.setText("X")
		self.strawberry2.setText("X")
		self.lemon2.setText("X")

		self.blueberry3.setText(str(blueberry_num))
		self.strawberry3.setText(str(strawberry_num))
		self.lemon3.setText(str(lemon_num))

	def show_team_details(self, team_id):
		team_details_csv = open(resource_path("team_bm.csv"))
		dict_reader = list(csv.DictReader(team_details_csv))
		team_dict = {}
		for i in dict_reader:
			teamid = i["Team ID"]
			college_name = i["College Name"]
			team_dict[teamid] = college_name

		self.teamid.setText(str(team_id))
		self.college_name.setText(team_dict[str(team_id)])

	# def show_evaluation_parameters(self, result_filename):
	#     team_id, dt, ci, cp, cd, collisions, bonus, end_simulation_time, init_real_time = self.decode_result_file(result_filename)
	#     # end_simulation_time = round(float(end_simulation_time),2)
	#     # self.num_seconds.setText(str(end_simulation_time))
	#     # self.num_ci.setText(str(len(ci)))
	#     # self.num_cp.setText(str(len(cp)))
	#     # self.num_cd.setText(str(len(cd)))
	#     # self.num_penalty.setText(str(len(collisions)))
	#     # self.bonus.setText(bonus)
	#     # bonus_val = 0

	#     # if bonus == 'Y':
	#     #     bonus_val = 200
	#     # else:
	#     #     bonus_val = 0

	#     # total_score = (600 - float(end_simulation_time)) + len(ci)*10 + len(cp)*50 + len(cd)*50 - len(collisions)*30 + bonus_val
	#     total_score = round(total_score, 2)
	#     self.final_score.setText(str(total_score))

	def output_terminal_written(self, text):
		self.console_output.append(text)




	def exit_btn(self):
		sys.exit()

	def end_process_btn(self):
		# raise KeyboardInterrupt
		# self.process.write(b'0x03')
		# os.kill(self.process.processId(), signal.SIGINT)

		output_list_child, output_list_custom = self.decode_bm_logfiles()
		# print(output_list_child, output_list_custom)

		# score = len(ci)*10   +   len(cp)*50   +   len(cd)*50   -   len(collision_list)*30

		# self.valid_run.setText("True")
		# self.num_seconds.setText("600")
		# self.num_ci.setText(str(len(ci)))
		# self.num_cp.setText(str(len(cp)))
		# self.num_cd.setText(str(len(cd)))
		# self.num_penalty.setText(str(len(collision_list)))
		# self.bonus.setText("0")
		# self.final_score.setText(str(score))

		print("[ERROR] Simulation interrupted abruptly. Please stop simulation manually and shut down and restart your modified Theme_Arena.ttt to rerun your code.")


		os.kill(self.process.processId(), signal.SIGINT)

		self.create_theme_implementation_result_file(output_list_child, output_list_custom)

		# children = psutil.Process(os.getpid()).children()
		# print(children)

		# for c in children:
		# 	c.send_signal(signal.CTRL_C_EVENT)
		# psutil.wait_procs(children)
		# client_id = init_remote_api_server()
		# print("client_id " + str(client_id))

	# def result_file_data(self):

	# 	platform_uname = platform.uname().system
	# 	curr_date_time = str(datetime.now())

	# 	encrypted_team_id   = cryptocode.encrypt(str(team_id_val), "s}ZYmS{:QgMx'9Qd")
	# 	encrypted_date_time = cryptocode.encrypt(curr_date_time, "s}ZYmS{:QgMx'9Qd")
	# 	encrypted_platform  = cryptocode.encrypt(platform_uname, "s}ZYmS{:QgMx'9Qd")
	# 	encrypted_mac       = cryptocode.encrypt(str(hex(uuid.getnode())), "s}ZYmS{:QgMx'9Qd")



	def decode_bm_logfiles(self):
		f1 = open(res_path + "/bm_logfile.txt")
		f2 = open(res_path + "/bm_dropped_logfile.txt")
		f3 = open(res_path + "/bm_path_logfile.txt")

		content1 = f1.readlines()
		content2 = f2.readlines()
		content3 = f3.readlines()

		f1.close()
		f2.close()
		f3.close()

		CI = []
		CP = []
		CD_CB1 = []
		CD_CB2 = []
		Collisions = []
		Eval_Dyn = None
		Mass = None
		Path = []
		dynamics_not_enabled = []
		SimTimeToReveal = None

		eval_no_of_joints = None
		no_of_joints = None
		total_force = None
		total_torque = None
		individual_torque_values = []

		output_list_child = []
		output_list_custom = []
		
		for line in content1:
			if "Total number of joints:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				no_of_joints = line[index_1+1:index_2]
			if "Eval Number of Joints:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				eval_no_of_joints = line[index_1+1:index_2]
			elif "Total torque required:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				total_torque = line[index_1+1:index_2]
			elif "Total force required:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				total_force = round(float(line[index_1+1:index_2]), 2)
			elif "Mass:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				Mass = line[index_1+1:index_2]
			elif "Individual Values:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				individual_torque_values.append(line[index_1+1:index_2])
			elif "Eval_Dyn:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				Eval_Dyn = line[index_1+1:index_2]
			elif "SimTimeToReveal :" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				SimTimeToReveal = line[index_1+1:index_2]
			elif "Berry Detected by Team:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				CI.append(line[index_1+1:index_2])
				# print("Berry detected ", line[index_1+1:index_2])
			elif "Successfully plucked:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				CP.append(line[index_1+1:index_2])
				# print("Successfully plucked ", line[index_1+1:index_2])
			elif "Robot is colliding. Colliding pair is :" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				Collisions.append(line[index_1+1:index_2])
				# print("collision: ", line[index_1+1:index_2])
			elif "Dynamics not enabled :" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				dynamics_not_enabled.append(line[index_1+1:index_2])
			else:
				pass

		for line in content2:
			if "Successfully dropped in CB1:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				CD_CB1.append(line[index_1+1:index_2])
			elif "Successfully dropped in CB2:" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				CD_CB2.append(line[index_1+1:index_2])
			else:
				pass

		for line in content3:
			if "Path_Coord :" in line:
				index_1 = line.find(':')
				index_2 = line.find('\n')
				index_3 = line.find(',')
				Path.append(line[index_1+1:index_3])
				Path.append(line[index_3+1:index_2])

		output_list_child.append("CI")

		for item in CI:
			output_list_child.append(item)

		output_list_child.append("Plucked")

		for item in CP:
			output_list_child.append(item)

		output_list_child.append("Dropped")
		output_list_child.append("CB1")

		for item in CD_CB1:
			output_list_child.append(item)

		output_list_child.append("CB2")

		for item in CD_CB2:
			output_list_child.append(item)

		output_list_child.append("Collisions")

		for item in Collisions:
			output_list_child.append(item)

		output_list_child.append("Eval_Dyn")
		output_list_child.append(Eval_Dyn)

		output_list_child.append("Mass")
		output_list_child.append(Mass)

		output_list_child.append("Path")

		for item in Path:
			output_list_child.append(item)

		output_list_child.append("Dyn")
		for item in dynamics_not_enabled:
			output_list_child.append(item)

		output_list_child.append("EndSimTime")
		output_list_child.append("600")

		output_list_child.append("SimTimeToReveal")
		output_list_child.append(SimTimeToReveal)

		output_list_custom.append(eval_no_of_joints)
		output_list_custom.append(no_of_joints)
		output_list_custom.append(total_torque)
		output_list_custom.append(total_force)
		for item in individual_torque_values:
			output_list_custom.append(item)

		return output_list_child, output_list_custom

	def create_theme_implementation_result_file(self, output_list_child, output_list_custom):
		platform_uname = platform.uname().system
		curr_date_time      = str(datetime.now())
		encrypted_team_id   = cryptocode.encrypt(str(team_id_val), "s}ZYmS{:QgMx'9Qd")
		encrypted_date_time = cryptocode.encrypt(curr_date_time, "s}ZYmS{:QgMx'9Qd")
		encrypted_platform  = cryptocode.encrypt(platform_uname, "s}ZYmS{:QgMx'9Qd")
		encrypted_mac       = cryptocode.encrypt(str(hex(uuid.getnode())), "s}ZYmS{:QgMx'9Qd")

		index = 0
		if output_list_child[index] != "CI":
			print("The data from the simulation got corrupted.")
			print("Kindly rerun the exe")
			print("theme_implementation_result.txt NOT generated")
			sys.exit()

		index = 1

		ci_temp = []
		ci_list_to_show = []
		while output_list_child[index] != "Plucked":
			ci_temp.append(output_list_child[index])
			index += 1
		
		ci_temp = unique(ci_temp)

		for i in ci_temp:
			# berry_name = general_berry_name(i)
			berry_name = i
			ci_list_to_show.append(berry_name)


		index += 1
		pluck_temp = []
		pluck_list_to_show = []
		while output_list_child[index] != "Dropped":
			pluck_temp.append(output_list_child[index])
			index += 1
		
		pluck_temp = unique(pluck_temp)

		for i in pluck_temp:
			# berry_name = general_berry_name(i)
			berry_name = i
			pluck_list_to_show.append(berry_name)


		index += 1                                      # Reached "Dropped"
		index += 1                                      # Reached "CB1"
		dropped_temp = []
		dropped_list_to_show = []
		while output_list_child[index] != "CB2":
			dropped_temp.append(output_list_child[index])
			index += 1
		
		dropped_temp = unique(dropped_temp)

		for i in dropped_temp:
			# berry_name = general_berry_name(i)
			berry_name = i
			dropped_list_to_show.append(berry_name)

		dropped_in_cb1 = dropped_list_to_show         # Berries dropped in CB1


		index += 1                                      # Reached one ahead of CB2
		dropped_temp = []
		dropped_list_to_show = []
		while output_list_child[index] != "Collisions":
			dropped_temp.append(output_list_child[index])
			index += 1
		
		dropped_temp = unique(dropped_temp)

		for i in dropped_temp:
			# berry_name = general_berry_name(i)
			berry_name = i
			dropped_list_to_show.append(berry_name)

		dropped_in_cb2 = dropped_list_to_show         # Berries dropped in CB2


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

		index += 1
		dynamically_not_enabled_list = []
		while output_list_child[index] != "EndSimTime":
			dynamically_not_enabled_list.append(output_list_child[index])
			index += 1

		index += 1
		end_simulation_time = output_list_child[index]

		index += 1                # Reached SimTimeToReveal
		index += 1                  
		time_to_substract   = output_list_child[index]


		eval_no_of_joints = output_list_custom[0]
		no_of_joints      = output_list_custom[1]
		torque            = output_list_custom[2]
		force             = output_list_custom[3]
		individual_values = output_list_custom[4:]

		total_no_of_collisions = len(collisions_list_to_show)
		effective_time         = float(end_simulation_time)
		data_from_calculate_score = calculate_score( effective_time, ci_list_to_show, pluck_list_to_show, dropped_in_cb1, dropped_in_cb2, total_no_of_collisions)

		rtf_python = "N/A"
		eval_rtf_python = 0
		print('\nCalculated Real-Time Factor (rtf) = ', rtf_python)

		encrypted_valid_run_flag       = cryptocode.encrypt(str(data_from_calculate_score[0]), "s}ZYmS{:QgMx'9Qd")
		encrypted_T                    = cryptocode.encrypt(str(data_from_calculate_score[1]), "s}ZYmS{:QgMx'9Qd")
		encrypted_CI                   = cryptocode.encrypt(str(data_from_calculate_score[2]), "s}ZYmS{:QgMx'9Qd")
		encrypted_CP                   = cryptocode.encrypt(str(data_from_calculate_score[3]), "s}ZYmS{:QgMx'9Qd")
		encrypted_CD                   = cryptocode.encrypt(str(data_from_calculate_score[4]), "s}ZYmS{:QgMx'9Qd")
		encrypted_P                    = cryptocode.encrypt(str(data_from_calculate_score[5]), "s}ZYmS{:QgMx'9Qd")
		encrypted_B                    = cryptocode.encrypt(str(data_from_calculate_score[6]), "s}ZYmS{:QgMx'9Qd")
		encrypted_score                = cryptocode.encrypt(str(data_from_calculate_score[7]), "s}ZYmS{:QgMx'9Qd")
		encrypted_final_ci_list        = cryptocode.encrypt(str(data_from_calculate_score[8]), "s}ZYmS{:QgMx'9Qd")
		encrypted_final_cp_list        = cryptocode.encrypt(str(data_from_calculate_score[9]), "s}ZYmS{:QgMx'9Qd")
		encrypted_final_cd_list        = cryptocode.encrypt(str(data_from_calculate_score[10]), "s}ZYmS{:QgMx'9Qd")
		encrypted_final_cb1_drops      = cryptocode.encrypt(str(data_from_calculate_score[11]), "s}ZYmS{:QgMx'9Qd")
		encrypted_final_cb2_drops      = cryptocode.encrypt(str(data_from_calculate_score[12]), "s}ZYmS{:QgMx'9Qd")
		


		encrypted_raw_ci_list          = cryptocode.encrypt(str(ci_list_to_show), "s}ZYmS{:QgMx'9Qd")
		encrypted_raw_cp_list          = cryptocode.encrypt(str(pluck_list_to_show), "s}ZYmS{:QgMx'9Qd")
		encrypted_raw_dropped_in_cb1_list = cryptocode.encrypt(str(dropped_in_cb1), "s}ZYmS{:QgMx'9Qd")
		encrypted_raw_dropped_in_cb2_list = cryptocode.encrypt(str(dropped_in_cb2), "s}ZYmS{:QgMx'9Qd")
		encrypted_raw_collisions_list  = cryptocode.encrypt(str(collisions_list_to_show), "s}ZYmS{:QgMx'9Qd")
		encrypted_mass_of_arm          = cryptocode.encrypt(str(mass_of_arm), "s}ZYmS{:QgMx'9Qd")
		encrypted_path                 = cryptocode.encrypt(str(path), "s}ZYmS{:QgMx'9Qd")

		encrypted_eval_rtf_python      = cryptocode.encrypt( str(eval_rtf_python), "s}ZYmS{:QgMx'9Qd")
		encrypted_end_simulation_time  = cryptocode.encrypt( str(float(end_simulation_time)), "s}ZYmS{:QgMx'9Qd")
		encrypted_init_real_time       = cryptocode.encrypt( "N/A", "s}ZYmS{:QgMx'9Qd")
		encrypted_end_real_time        = cryptocode.encrypt( "N/A", "s}ZYmS{:QgMx'9Qd")
		encrypted_rtf_python           = cryptocode.encrypt( str(rtf_python), "s}ZYmS{:QgMx'9Qd")

		encrypted_eval_all_dynamics    = cryptocode.encrypt( str(eval_all_dynamics), "s}ZYmS{:QgMx'9Qd")
		encrypted_eval_no_of_joints    = cryptocode.encrypt( str(eval_no_of_joints), "s}ZYmS{:QgMx'9Qd")
		encrypted_no_of_joints         = cryptocode.encrypt( str(no_of_joints), "s}ZYmS{:QgMx'9Qd")
		encrypted_torque               = cryptocode.encrypt( str(torque), "s}ZYmS{:QgMx'9Qd")
		encrypted_force                = cryptocode.encrypt( str(force), "s}ZYmS{:QgMx'9Qd")
		encrypted_individual_values    = cryptocode.encrypt( str(individual_values), "s}ZYmS{:QgMx'9Qd")
		encrypted_dynamically_not_enabled_list = cryptocode.encrypt( str(dynamically_not_enabled_list), "s}ZYmS{:QgMx'9Qd")

		encrypted_time_to_substract    = cryptocode.encrypt( str(time_to_substract), "s}ZYmS{:QgMx'9Qd")

		encrypted_output_list_child   =  cryptocode.encrypt(str(output_list_child), "s}ZYmS{:QgMx'9Qd")
		encrypted_output_list_custom  =  cryptocode.encrypt(str(output_list_custom), "s}ZYmS{:QgMx'9Qd")

		if config_type == "Original Configuration":
			# bm_task_key_filename = resource_path("bm_task_6_key_original_config.json")
			bm_task_key = axebnfgh
		elif config_type == "Bonus Configuration":
			bm_task_key = bnaxefgh
		else:
			pass

		gsheet_data = [curr_date_time, platform_uname, collisions_list_to_show, mass_of_arm, end_simulation_time, rtf_python, no_of_joints, torque, force]

		return_value = send_data_e_yantra_server(bm_task_key, data_from_calculate_score, gsheet_data)
		
		
		if(return_value==1):
			f = open(folder_path + "/theme_implementation_result.txt", "w")

			f.write(encrypted_team_id); f.write("\n")
			f.write(encrypted_date_time); f.write("\n")
			f.write(encrypted_platform); f.write("\n")
			f.write(encrypted_mac); f.write("\n")

			f.write(encrypted_T); f.write("\n")
			f.write(encrypted_CI); f.write("\n")
			f.write(encrypted_CP); f.write("\n")
			f.write(encrypted_CD); f.write("\n")
			f.write(encrypted_P); f.write("\n")
			f.write(encrypted_B); f.write("\n")
			f.write(encrypted_score); f.write("\n")
			f.write(encrypted_final_ci_list); f.write("\n")
			f.write(encrypted_final_cp_list); f.write("\n")
			f.write(encrypted_final_cd_list); f.write("\n")
			f.write(encrypted_final_cb1_drops); f.write("\n")
			f.write(encrypted_final_cb2_drops); f.write("\n")

			f.write(encrypted_raw_ci_list); f.write("\n")
			f.write(encrypted_raw_cp_list); f.write("\n")
			f.write(encrypted_raw_dropped_in_cb1_list); f.write("\n")
			f.write(encrypted_raw_dropped_in_cb2_list); f.write("\n")
			f.write(encrypted_raw_collisions_list); f.write("\n")
			f.write(encrypted_mass_of_arm); f.write("\n")
			f.write(encrypted_path); f.write("\n")

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
			f.write(encrypted_dynamically_not_enabled_list); f.write("\n")

			f.write(encrypted_valid_run_flag); f.write("\n")
			f.write(encrypted_time_to_substract); f.write("\n")

			f.write(encrypted_output_list_child); f.write("\n")
			f.write(encrypted_output_list_custom); f.write("\n")

			f.close()

			print("\n-----------------------------------------------------------")
			print("\n Way to go "+str(team_id_val)+"! 'theme_implementation_result.txt' generated successfully.")
			print("\n-----------------------------------------------------------")
		else:
			print('\nFAILED generating theme_implementation_result.txt. Please follow Task 6 Instructions.')



	def show_evaluation_parameters(self, filename):
		f = open(filename, "r")
		content = f.readlines()

		# Extracting team no and score
		i = 1
		for x in content:
			if   i == 1:
				team_id = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 2:
				dt = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 3:
				platform = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 4:
				mac = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 5:
				T = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 6:
				CI = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 7:
				CP = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 8:
				CD = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 9:
				P = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 10:
				B = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 11:
				score = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 12:
			#     final_ci_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 13:
			#     final_cp_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 14:
			#     final_cd_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 15:
			#     final_cb1_drops = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 16:
			#     final_cb2_drops = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")

			# elif i == 17:
			#     raw_ci_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 18:
			#     raw_cp_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 19:
			#     raw_dropped_in_cb1_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 20:
			#     raw_dropped_in_cb2_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 21:
			#     raw_collisions_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 22:
			#     mass_of_arm = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 23:
			#     path = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")



			# elif i == 24:
			#     eval_rtf_python = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 25:
			#     end_simulation_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 26:
			#     init_real_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 27:
			#     end_real_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 28:
			#     rtf_python = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 29:
			#     eval_all_dynamics = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 30:
			#     eval_no_of_joints = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 31:
			#     no_of_joints = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 32:
			#     torque = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 33:
			#     force = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 34:
			#     individual_values = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 35:
			#     dynamically_not_enabled_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			elif i == 36:
				valid_run_flag = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 37:
			#     time_to_substract = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")    
			# elif i == 38:
			#     output_list_child = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
			# elif i == 39:
			#     output_list_custom = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")

			i += 1

		# print("Team no                         : ", team_id)
		# print("Date and Time                   : ", dt)
		# print("Platform                        : ", platform)
		# print("Mac                             : ", mac)
		# print("VALID Run?                      : ", valid_run_flag)
		# print("T                               : ", T)
		# print("CI                              : ", CI)
		# print("CP                              : ", CP)
		# print("CD                              : ", CD)
		# print("P                               : ", P)
		# print("B                               : ", B)
		# print("score                           : ", score)
		# print("Final CI List                   : ", final_ci_list)
		# print("Final CP List                   : ", final_cp_list)
		# print("Final CD List                   : ", final_cd_list)
		# print("Final CB1 Drops                 : ", final_cb1_drops)
		# print("Final CB2 Drops                 : ", final_cb2_drops)
		# print()
		# print("Raw CI List                     : ", raw_ci_list)
		# print("Raw CP List                     : ", raw_cp_list)
		# print("Raw Dropped in CB1 List         : ", raw_dropped_in_cb1_list)
		# print("Raw Dropped in CB2 List         : ", raw_dropped_in_cb2_list)
		# print("Collisions                      : ", raw_collisions_list)
		# print("Mass of Arm                     : ", mass_of_arm)
		# print("Path                            : ", path)
		# print("eval rtf                        : ", eval_rtf_python)
		# print("Total simulation time           : ", end_simulation_time)
		# print("Time to substract               : ", time_to_substract)
		# print("Init real time                  : ", init_real_time)
		# print("End real time                   : ", end_real_time)
		# print("RTF from Python                 : ", rtf_python)
		# print("Eval all dynamics               : ", eval_all_dynamics)
		# print("Eval no of joints               : ", eval_no_of_joints)
		# print("No. of joints                   : ", no_of_joints)
		# print("Torque                          : ", torque)
		# print("Force                           : ", force)
		# print("Indivdual Values                : ", individual_values)
		# print("Dyn not enabled list            : ", dynamically_not_enabled_list)
		# print("output_list_child               : ", output_list_child)
		# print("output_list_custom              : ", output_list_custom)

		T = str(round(float(T),2))
		score = str(round(float(score),2))

		self.valid_run.setText(valid_run_flag)
		self.num_seconds.setText(T)
		self.num_ci.setText(CI)
		self.num_cp.setText(CP)
		self.num_cd.setText(CD)
		self.num_penalty.setText(P)
		self.bonus.setText(B)
		self.final_score.setText(score)

	def calculate_bonus(self, CI, CP, CD, Collisions, end_simulation_time):
		blueberry = theme_config["B"][0]
		strawberry = theme_config["S"][0]
		lemon = theme_config["L"][0]
		blueberry_num, blueberry_collection_box = int(blueberry[0]), blueberry[2:]
		strawberry_num, strawberry_collection_box = int(strawberry[0]), strawberry[2:]
		lemon_num, lemon_collection_box = int(lemon[0]), lemon[2:]

		if len(Collisions) > 0:
			return 'N'
		if float(end_simulation_time) >= 600:
			return 'N'

		ci_flag = None
		cp_flag = None
		cd_flag = None

		if CI.count("Blueberry") == blueberry_num and  CI.count("Strawberry") == strawberry_num and CI.count("Lemon") == lemon_num:
			ci_flag = True
		else:
			ci_flag = False

		if CP.count("Blueberry") == blueberry_num and  CP.count("Strawberry") == strawberry_num and CP.count("Lemon") == lemon_num:
			cp_flag = True
		else:
			cp_flag = False

		if CD.count("Blueberry") == blueberry_num and  CD.count("Strawberry") == strawberry_num and CD.count("Lemon") == lemon_num:
			cd_flag = True
		else:
			cd_flag = False

		if ci_flag and cp_flag and cd_flag:
			return 'Y'
		else:
			return 'N'



if __name__ == "__main__":
	import sys
	app = QtWidgets.QApplication(sys.argv)
	Berryminator_Evaluator = QtWidgets.QMainWindow()
	ui = Ui_Berryminator_Evaluator()
	families = load_fonts_from_dir(os.fspath(resource_path('')))
	ui.setupUi(Berryminator_Evaluator)
	Berryminator_Evaluator.show()
	sys.exit(app.exec_())