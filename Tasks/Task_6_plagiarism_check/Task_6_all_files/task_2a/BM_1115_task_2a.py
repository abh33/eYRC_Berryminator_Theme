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

# Team ID:			[1115]
# Author List:		[Rabbi Sudheer Zacharias,D S SaiRohith,Nisarga,Ramya]
# Filename:			task_2a.py
# Functions:		
# Global variables:	
# 					[ None ]

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
#To find centroid and depth of the berries
def find_centroid(image,dict1,u,depthimg):
	coordinates=list()
	thresh=cv2.bitwise_not(image)
	contours,hierarchies=cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	for i in contours:
		M=cv2.moments(i)
		if M['m00']!=0:
			cx=int(M['m10']/M['m00'])
			cy=int(M['m01']/M['m00'])
			#cx1=round(cx,2)
			#cy1=round(cy,2)
			cz=depthimg[cy][cx]
			coordinates.append((cx,cy,cz))
	coordinates.pop()
	dict1[u]=coordinates
	return dict1
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
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id,vision_sensor_handle,0,sim.simx_opmode_oneshot_wait)
	##################################################

	return vision_sensor_image, image_resolution, return_code

def get_vision_sensor_depth_image(client_id, vision_sensor_handle):
	
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
	return_code, image_resolution, vision_sensor_depth_image=sim.simxGetVisionSensorDepthBuffer(client_id,vision_sensor_handle,sim.simx_opmode_oneshot_wait)
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
	arr=np.array(vision_sensor_depth_image)
	arr1=arr.astype('float32')
	resize_array=np.reshape(arr,(image_resolution[0],image_resolution[1]))
	transformed_depth_image = cv2.flip(resize_array, 1)
	#transformed_depth_image = resize_array
	##################################################
	
	return transformed_depth_image

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
	berries = ["Strawberry", "Blueberry", "Lemon"]

	##############	ADD YOUR CODE HERE	##############
	berries_dict={}
	font = cv2.FONT_HERSHEY_COMPLEX
	hsv=cv2.cvtColor(transformed_image,cv2.COLOR_BGR2HSV)
	lower_range_blue = np.array([90,50,70])
	upper_range_blue = np.array([128,255,255])
	lower_range_red = np.array([160,100,100])
	upper_range_red = np.array([180,255,255])
	lower_range_yellow=np.array([23,100,100])
	upper_range_yellow=np.array([35,255,255])
	blue=cv2.inRange(hsv,lower_range_blue, upper_range_blue)
	red=cv2.inRange(hsv,lower_range_red,upper_range_red)
	yellow=cv2.inRange(hsv,lower_range_yellow,upper_range_yellow)
	hasBlue = np.sum(blue)
	hasRed = np.sum(red)
	hasYellow=np.sum(yellow)

	if hasRed>0:
		#gray_r=cv2.bitwise_not(red)
		u0=berries[0]
		find_centroid(red,berries_dict,u0,transformed_depth_image)
	else:
		berries_dict[berries[0]]=None




	if hasBlue>0:
		#gray_b=cv2.bitwise_not(blue)
		u1=berries[1]
		find_centroid(blue,berries_dict,u1,transformed_depth_image)
	else:
		berries_dict[berries[1]]=None
		
		
	if hasYellow>0:
		#gray_y=cv2.bitwise_not(yellow)
		u2=berries[2]
		x3=find_centroid(yellow,berries_dict,u2,transformed_depth_image)
	else:
		berries_dict[berries[2]]=None

	berries_dictionary=berries_dict
	
	##################################################
	return berries_dictionary

def detect_berry_positions(berries_dictionary):
	"""
	Purpose:
	---
	This function takes the berries_dictionary as input arguments and calculates the 3D positions of the
	berries with respect to vision sensor. The final output is returned in another dictionary.
	
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
	berries = ["Strawberry", "Blueberry", "Lemon"]

	##############	ADD YOUR CODE HERE	##############
	if berries_dictionary[berries[0]]!=None:
		A=berries_dictionary.get(berries[0])
		C=list()
		for i in range (0,len(A)):
			p=round((A[i][0]/512)-0.5,2)
			q=round((A[i][1]/512)-0.5,2)
			r=round((2*A[i][2])+0.03,2)
			print("task 2a 1",p,q,r)
			if r>0.07:
				C.append((p,q,r))
		berry_positions_dictionary[berries[0]]=C
	
	if berries_dictionary[berries[1]]!=None:
		A=berries_dictionary.get(berries[1])
		C=list()
		for i in range (0,len(A)):
			p=round((A[i][0]/512)-0.5,2)
			q=round((A[i][1]/512)-0.5,2)
			r=round(2*A[i][2]+0.03,2)
			print("task 2a 2",p,q,r)
			if r>0.07:
				C.append((p,q,r))
		berry_positions_dictionary[berries[1]]=C
	
	if berries_dictionary[berries[2]]!=None:
		A=berries_dictionary.get(berries[2])
		C=list()
		for i in range (0,len(A)):
			p=round((A[i][0]/512)-0.5,2)
			q=round((A[i][1]/512)-0.5,2)
			r=round(2*A[i][2]+0.03,2)
			print("task 2a 3",p,q,r)
			if r>0.07:
				C.append((p,q,r))
		berry_positions_dictionary[berries[2]]=C
	
	##################################################
	return berry_positions_dictionary

def get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary):
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



