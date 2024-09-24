'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 2A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2a.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os, sys
import traceback
##############################################################

try:
	import sim	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()
try:
	import task_1b
except ImportError:
	print('\n[ERROR] task_1b.py file is not present in the current directory.')
	print('Your current directory is: ', os.getcwd())
	print('Make sure task_1b.py is present in this current directory.\n')
	sys.exit()

################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def get_vision_sensor_image(client_id, vision_sensor_handle):
	
	"""
	Purpose:
	---
	This function takes the client id and handle of the vision sensor scene object as input
	arguments and returns the vision sensor's image array from the CoppeliaSim scene.

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`vision_sensor_handle`    :   [ integer ]
		the handle of the vision sensor scene object
	
	Returns:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
	
	Example call:
	---
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
	"""
	vision_sensor_image = []
	image_resolution = []
	return_code = 0

	##############	ADD YOUR CODE HERE	##############
	return_code, image_resolution, vision_sensor_image= sim.simxGetVisionSensorImage(client_id, vision_sensor_handle, 0, sim.simx_opmode_blocking)

	##################################################

	return vision_sensor_image, image_resolution, return_code

def get_vision_sensor_depth_image(client_id,objhandle):
	
	"""
	Purpose:
	---
	This function takes the client id and handle of the vision sensor scene object as input
	arguments and returns the vision sensor's depth buffer array from the CoppeliaSim scene.
	Input Arguments:
	---
	`client_id`               :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`vision_sensor_handle`    :   [ integer ]
		the handle of the vision sensor scene object
	
	Returns:
	---
	`vision_sensor_depth_image` 	:  [ list ]
		the depth buffer array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	`return_code` 			:  [ integer ]
		the return code generated from the remote API
	
	Example call:
	---
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image()
	"""
	vision_sensor_depth_image = []
	image_resolution = []
	return_code = 0

	##############	ADD YOUR CODE HERE	##############
	return_code, image_resolution, vision_sensor_depth_image= sim.simxGetVisionSensorDepthBuffer(client_id, objhandle, sim.simx_opmode_blocking)
	##################################################

	return vision_sensor_depth_image, image_resolution, return_code

def transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution):

	"""
	Purpose:
	---
	This function converts the depth buffer array received from vision sensor and converts it into
	a numpy array that can be processed by OpenCV
	This function should:
	1. First convert the vision_sensor_depth_image list to a NumPy array with data-type as float32.
	2. Since the depth image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
	the new NumPy array should then be resized to a 2-D (two dimensional) NumPy array.
	3. Flip the resultant image array about the appropriate axis. The resultant image NumPy array should be returned.
	
	Input Arguments:
	---
	`vision_sensor_depth_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get_vision_sensor_depth_image() function
	
	Returns:
	---
	`transformed_depth_image` 	:  [ numpy array ]
		the resultant transformed image array after performing above 3 steps
	
	Example call:
	---
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	
	"""
	transformed_depth_image = None

	##############	ADD YOUR CODE HERE	##############
	np1d=np.array(vision_sensor_depth_image).astype('float32')
	x=int(image_resolution[0])
	y=int(image_resolution[1])
	np2d=np.resize(np1d, (x,y))
	transformed_depth_image=np.flip(np2d, axis=0)
	##################################################
	
	return transformed_depth_image

def check_in_basket(x,y,z):
	"""
	Purpose:
	---
	This function ensures that the arm does not pick the berries already deposited in its collecting basket.
	This is done by hard-coding the values that denote the boundary of the basket, i.e, the area inside which the collected
		berries can be found.

	Input Arguments:
	x,y,z : These are the coordinates of the berries detected by the vision_sensor_2 with respect to the vision_sensor_2
	
	Returns:
	---
	False: If the berry detected is not inside the basket
	True: If the berry detected is already in the basket

	Example call:
	---
	check_in_basket(x,y,z)

	"""
	
	low_x=-0.15712356567383
	high_x=0.16787648200989
	low_y=0.018054723739624
	high_y= 0.31793212890625
	low_z= 0.00000000000
	high_z=3.28197121620178
	if ((low_x<=x<=high_x) and (low_y<=y<=high_y) and (low_z<=z<=high_z)):
		return True
	else:
		return False

	##################################################
	
def check_near_rack(x,y,z,position):
	"""
	Purpose:
	---
	This function ensures that the arm does not pick the berries that are very close to the rack.

	Input Arguments:
	x,y,z : These are the coordinates of the berries detected by the vision_sensor_2 with respect to the vision_sensor_2.
	position : This is the estimate position of the berry with respect to the bot.
	Returns:
	---
	False: If the berry detected is not inside the basket
	True: If the berry detected is already in the basket

	Example call:
	---
	check_in_basket(x,y,z,'front')
	
	"""
	if position=='front':
		if (y>=0.25631776452065):
			return False
		else:
			return True
			
	elif position=='behind':
		if (y<=-0.26881468296051):
			return False
		else:
			return True
			
	elif position=='left':
		if (x<=-0.24717375636101):
			return False
		else:
			return True
			
	elif position=='right':
		if (x>=0.27782607078552):
			return False
		else:
			return True
		
	else: #for 'center'
		return False
	


def detect_berries(transformed_image, transformed_depth_image):
	"""
	Purpose:
	---
	This function takes the transformed image and transformed depth image as input arguments and returns
	the pixel coordinates and depth values in form of a dictionary.
	
	Input Arguments:
	---
 	`transformed_image` 	:  [ numpy array ]
 		the transformed image array
 	`transformed_depth_image` 	:  [ numpy array ]
 		the transformed depth image array
	
	Returns:
	---
	`berries_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of all the berries
	
	Example call:
	---
	berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
	
	"""
	berries_dictionary = {}
	berries = ["Strawberry", "Lemon", "Blueberry"]

	##############	ADD YOUR CODE HERE	##############
	misc=[]
	berries_dictionary=dict.fromkeys(berries)
	for key in berries_dictionary:
		berries_dictionary[key]=[]
	gray = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
	_, threshold = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY_INV)
	contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	for contour in contours[1:]: 
		#vertices = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True) #finding vertices
		M = cv2.moments(contour) #finding center point
		if M['m00'] != 0.0:
			x = int(M['m10']/M['m00'])
			y = int(M['m01']/M['m00'])
		else:
			continue
		d= transformed_depth_image[y,x]		#finding depth
		#finding color
		#strawberry
		if ((40<=transformed_image[y,x][0]<=120) and (0<=transformed_image[y,x][1]<=30) and (170<=transformed_image[y,x][2]<=255)):
			berries_dictionary['Strawberry'].append((x,y,d))	
		#blueberry
		elif ((170<=transformed_image[y,x][0]<=255) and (40<=transformed_image[y,x][1]<=120) and (0<=transformed_image[y,x][2]<=30)):
			berries_dictionary['Blueberry'].append((x,y,d))		
		#lemon
		elif ((0<=transformed_image[y,x][0]<=30) and (170<=transformed_image[y,x][1]<=255) and (170<=transformed_image[y,x][2]<=255)):
			berries_dictionary['Lemon'].append((x,y,d))	
		#elif ((50<=transformed_image[y,x][0]<=150) and (50<=transformed_image[y,x][1]<=150) and (50<=transformed_image[y,x][2]<=150)):
			#misc.append([[x,y],transformed_image[y,x]])	
		else:
			continue			
	##################################################
	return berries_dictionary

def detect_berry_positions(berries_dictionary, position):
	"""
	Purpose:
	---
	This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
	berries with respect to vision sensor. It will also filter out berries which are found in  unsuitable places.
	The final output is returned in another dictionary.
	
	Input Arguments:
	---
	`berries_dictionary` 	:  [ dictionary ]
		the dictionary returned by detect_berries() function
	
	Returns:
	---
	`berry_positions_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of 3D positions of all the berries
	
	Example call:
	---
	berry_positions_dictionary = detect_berry_positions(berries_dictionary)
	
	"""
	berry_positions_dictionary = {}
	berries = ["Strawberry", "Lemon", "Blueberry"]
	##############	ADD YOUR CODE HERE	##############
	
	berry_positions_dictionary=dict.fromkeys(berries)
	for key in berry_positions_dictionary:
		berry_positions_dictionary[key]=[]
		
	for fruit in berries_dictionary['Lemon']:
		x=-((fruit[0]/512)*1.65-0.825)-0.002
		y=-((fruit[1]/512)*1.65-0.825)-0.0024
		z=fruit[2]+0.0245
		if ((check_in_basket(x, y, z)==False) and (check_near_rack(x,y,z,position)==False)):
			berry_positions_dictionary['Lemon'].append((x,y,z))
		
	for fruit in berries_dictionary['Strawberry']:
		x=-((fruit[0]/512)*1.65-0.825)-0.002
		y=-((fruit[1]/512)*1.65-0.825)-0.0024
		z=fruit[2]+0.0245
		if ((check_in_basket(x, y, z)==False) and (check_near_rack(x,y,z,position)==False)):
			berry_positions_dictionary['Strawberry'].append((x,y,z))
			
	for fruit in berries_dictionary['Blueberry']:
		x=-((fruit[0]/512)*1.65-0.825)-0.002
		y=-((fruit[1]/512)*1.65-0.825)-0.0024
		z=fruit[2]+0.0245
		if ((check_in_basket(x, y, z)==False) and (check_near_rack(x,y,z,position)==False)):
			berry_positions_dictionary['Blueberry'].append((x,y,z))
		
		
	
	##################################################
	return berry_positions_dictionary

def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
	#OBSOLETE
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the transformed_image and the dictionaries returned by detect_berries()
	and  detect_berry_positions() functions. This function is already completed for your reference
	and will be helpful for debugging purposes.

	Input Arguments:
	---
	`transformed_image` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`berries_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of all the berries

	`berry_positions_dictionary` 	:  [ dictionary ]
		the resultant dictionary with details of 3D positions of all the berries

	Returns:
	---
	`labelled_image` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	transformed_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
	"""
	labelled_image = np.array(transformed_image)
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for berry_type in berries_dictionary.keys():
		berry_details_list = berries_dictionary[berry_type]
		berry_positions_list = berry_positions_dictionary[berry_type]
		for index in range(len(berry_details_list)):
			pixel_x, pixel_y, depth_val = berry_details_list[index]
			coordinates = (pixel_x, pixel_y)
			horizontal_displacement, vertical_displacement, distance_from_sensor = berry_positions_list[index]
			horizontal_displacement, vertical_displacement, distance_from_sensor = round(horizontal_displacement, 2), round(vertical_displacement, 2), round(distance_from_sensor, 2)
			cv2.putText(labelled_image, str((horizontal_displacement, vertical_displacement, distance_from_sensor)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255), 1)
	return labelled_image

if __name__ == "__main__":
	#OBSOLETE
	berries_dictionary = {}
	berry_positions_dictionary = {}
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')
	cv2.namedWindow('transformed image', cv2.WINDOW_AUTOSIZE)
	cv2.namedWindow('transformed depth image', cv2.WINDOW_AUTOSIZE)
	try:
		# Initiate Remote API connection
		client_id = task_1b.init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')
			return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor', sim.simx_opmode_blocking)
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
	while True:
	# Get image array and depth buffer from vision sensor in CoppeliaSim scene
		try:
			vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id, vision_sensor_handle)
			vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id, vision_sensor_handle)
			if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
				print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')
				# Get the transformed vision sensor image captured in correct format
				try:
					transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
					transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
					if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):
						berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
						print("Berries Dictionary = ", berries_dictionary)
						berry_positions_dictionary = detect_berry_positions(berries_dictionary)
						print("Berry Positions Dictionary = ",berry_positions_dictionary)
						labelled_image = get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary)
						cv2.imshow('transformed image', transformed_image)
						cv2.imshow('transformed depth image', transformed_depth_image)
						cv2.imshow('labelled image', labelled_image)
						if cv2.waitKey(1) & 0xFF == ord('q'):
							break
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
	cv2.destroyAllWindows()
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



