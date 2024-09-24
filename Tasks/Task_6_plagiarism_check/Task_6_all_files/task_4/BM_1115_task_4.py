'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
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
# Author List:		[ Rabbi S Zacharias, DS Sai Rohith, Nisarga, Ramya ]
# Filename:			task_4.py
# Functions:		call_open_close(int,command),send_identified_berry_data(int,string,float,float,float),visionsensorcode(int)
#					pluck_left_berry(int,dict,int),pluck_centre_berry(int,dict,int,string),pluck_right_berry(int,dict,int)
#					pluck_berry(int,string,int,dict,int),getpos(int,string,int,int),task_4_primary(int)
#					
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################


from cmath import sqrt
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()

import task_1b
import task_2a
import task_3


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)



##############################################################


def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):
	"""
	Purpose:
	---
	Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.
	
	NOTE: 
	1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
		Hence before plucking, the correct berry should be identified and sent via this function.

	2.	Accuracy of detection should be +-0.025m.

	Input Arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable

	'berry_name'		:	[ string ]
			name of the detected berry.

	'x_coor'			:	[ float ]
			x-coordinate of the centroid of the detected berry.

	'y_coor'			:	[ float ]
			y-coordinate of the centroid of the detected berry.

	'depth'			:	[ float ]
			z-coordinate of the centroid of the detected berry.

	Returns:
	---
	`return_code`		:	[ integer ]
			A remote API function return code
			https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

	Example call:
	---
	return_code=send_identified_berry_data(berry_name,x_coor,y_coor)
	
	"""

	depth=depth
	##################################################
	## You are NOT allowed to make any changes in the code below. ##
	emptybuff = bytearray()

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code


def visionsensorcode(client_id):

	"""
	Purpose:
	---
	To get Coordinates of berries with respect to vision_sensor_2

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
    Returns:
	---
		berry_positions_dictionary : [Dictionary]
			A dictionary which contains coordinates of the berries wrt vision_sensor_2
	
	Example call:
	---
		berry_positions_dictionary=visionsensorcode(client_id)

	"""




	##########################################################
	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	vision_sensor_depth_image, image_resolution, return_code = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution)
	berries_dictionary=task_2a.detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary=task_2a.detect_berry_positions(berries_dictionary)

	#print("task 4 vision sensor code",berry_positions_dictionary)

	return berry_positions_dictionary
	####################################################

def pluck_left_berry(client_id,bps,local_collector):
	"""
	Purpose:
	---
		To pluck berries which are on the left of vision_sensor_2
		This function sets the path to pluck berry which are on the left side of vision_sensor_2;
		and then drop it to the local collector specified by the input argument 'local_collector'.

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
	bps             :  [ Dictionary ]
		A dictionary which contains coordinates of the berries wrt vision_sensor_2
	

	local_collector:  [integer]
		1 for local collector 1
		2 for local collector 2

	Returns:
	---
		None
	
	Example call:
	---
		pluck_left_berry(client_id,bps,local_collector)
	"""
	#######################################################################

	#rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.645,0.162),[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(2)

	call_open_close(client_id,"open")
	#time.sleep(2)
	#print("pluck_left_berry")
	x=0
	while x<0.4:
		a=sqrt(0.05-pow((x-0.11),2))
		y=a.real+0.45
		z=(0.00926+0.051*x)/0.2945
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(x,y,z),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(0.01)
		x=x+0.01

	#time.sleep(2)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]+0.025,bps[1]+0.05,bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(2)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]+0.01,bps[1],bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(2)
	call_open_close(client_id,"close")
	time.sleep(2)

	x=0.4
	while x>0:
		a=sqrt(0.05-pow((x-0.11),2))
		y=a.real+0.50
		z=(0.00926+0.051*x)/0.2945
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(x,y,z),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(0.01)
		x=x-0.01
	#time.sleep(0.5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.25,0.162),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.219,0.04),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)


	if local_collector ==1:
		#print("pluck_left_berry     if")
		time.sleep(1)
		#call_open_close(client_id,"open")
		time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.3,0.19),[],bytearray(),sim.simx_opmode_blocking)
		#time.sleep(1)

	else:
		#print("pluck_left_berry     else")

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.319,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.319,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.40,0.319,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.3,0.219,-1),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.219,-0.5),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)

		time.sleep(2)
		call_open_close(client_id,"open")
		time.sleep(2)

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.0144,0.219,-0.46359),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.28,0.219,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.219,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.219,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.4,0.162),[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(2)
	
	###########################################################################3

def pluck_centre_berry(client_id,bps,local_collector,berry):
	"""
	Purpose:
	---
		To pluck berries which are at the same line as vision_sensor_2
		This function sets the path to pluck berry which are almost straight wrt vision_sensor_2;
		and then drop it to the local collector specified by the input argument 'local_collector'.

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
	bps             :  [ Dictionary ]
		A dictionary which contains coordinates of the berries wrt vision_sensor_2
	

	local_collector:  [integer]
		1 for local collector 1
		2 for local collector 2

	Returns:
	---
		None
	
	Example call:
	---
		pluck_centre_berry(client_id,bps,local_collector)
	"""
	####################################################

	
	rc=send_identified_berry_data(client_id,berry,bps[0],bps[1],bps[2])
	print("sending berry data",berry,bps)
	call_open_close(client_id,"open")
	#time.sleep(2)
	#bps[0]+0.006
	print("pluck_center_berry")
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]-((4*bps[0])*0.015),bps[1]+0.15,bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(2)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]-((4*bps[0])*0.009),bps[1],bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)
	call_open_close(client_id,"close")
	time.sleep(2)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1]+0.05,bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(0.5)

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1]+0.22,bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(0.5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1]+0.22,bps[2]-0.05),[],bytearray(),sim.simx_opmode_blocking)
	
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1]+0.24,bps[2]-0.05),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(0.5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.3,0.05),[],bytearray(),sim.simx_opmode_blocking)


	if local_collector ==1:
		print("pluck_center_berry     if")
		time.sleep(2)
		call_open_close(client_id,"open")
		time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.3,0.162),[],bytearray(),sim.simx_opmode_blocking)
		#time.sleep(1)

	else:
		print("pluck_center_berry     else")

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.319,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.319,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.40,0.319,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.3,0.219,-1),[],bytearray(),sim.simx_opmode_blocking)
		#time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.219,-0.5),[],bytearray(),sim.simx_opmode_blocking)
		#time.sleep(1)

		time.sleep(2)
		call_open_close(client_id,"open")
		time.sleep(2)

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.0144,0.219,-0.46359),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.28,0.219,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.219,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.40,0.319,-0.224),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.219,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0.1),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.3,0.162),[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(1)

	#########################################################
	
def pluck_right_berry(client_id,bps,local_collector):
	"""
	Purpose:
	---
		To pluck berries which are on the right of vision_sensor_2
		This function sets the path to pluck berry which are on the right side of vision_sensor_2;
		and then drop it to the local collector specified by the input argument 'local_collector'.

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
	bps             :  [ Dictionary ]
		A dictionary which contains coordinates of the berries wrt vision_sensor_2
	

	local_collector:  [integer]
		1 for local collector 1
		2 for local collector 2

	Returns:
	---
	x                : [ integer ]
		Number of berries plucked.
	
	Example call:
	---
		pluck_right_berry(client_id,bps,local_collector)
	"""
	####################################################

	call_open_close(client_id,"open")
	#time.sleep(2)

	print("pluck_right_berry")
	x=0
	while x>-0.4:
		a=sqrt(0.06-pow((x+0.08),2))
		y=a.real+0.45
		z=(0.009206-0.051*x)/0.2495
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],[x,y,z],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(0.01)
		x=x-0.01


	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]+0.02,bps[1]+0.2,bps[2]+0.02),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1]+0.1,bps[2]+0.02),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1],bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)
	#rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0],bps[1],bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(5)


	call_open_close(client_id,"close")
	time.sleep(2)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(bps[0]+0.01,bps[1]+0.05,bps[2]),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)


	x=-0.4
	while x<0:
		a=sqrt(0.06-pow((x+0.08),2))
		y=a.real+0.50
		z=(0.009206-0.051*x)/0.2495
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],[x,y,z],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(0.01)
		x=x+0.01

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.25,0.162),[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(1)

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.219,0.04),[],bytearray(),sim.simx_opmode_blocking)
	
	if local_collector ==1:
		print("pluck_right_berry     if")
		time.sleep(2)
		call_open_close(client_id,"open")
		time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.3,0.162),[],bytearray(),sim.simx_opmode_blocking)
		#time.sleep(2)

	else:
		print("pluck_right_berry     else")

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.319,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.319,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.4,0.319,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.3,0.219,-1),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(2)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.219,-0.5),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)

		time.sleep(2)
		call_open_close(client_id,"open")
		time.sleep(3)

		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.0144,0.219,-0.46359),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.28,0.219,-0.624),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.45,0.219,-0.324),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.65,0.219,-0.173),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.425,0.319,0),[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(1)

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(-0.08,0.4,0.162),[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(2)
		
	
	

	#rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.645,0.162),[],bytearray(),sim.simx_opmode_blocking)	
	##################################################

def pluck_berry(client_id,berry,count,berry_positions_dictionary,local_collector):

	"""
	Purpose:
	---
		To pluck the desired berry

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
	berry           :  [ string ]
		Name og the berry to be plucked.
		"Blueberry","Strawberry" or "Lemon"

	count           :  [ integer ]
		Number of berries to be plucked.
	
	berry_positions_dictionary:
		Dictionary containing berry positions
	
	local collector :  [ integer ]
		1 for local collector1
		2 for local collector2

	Returns:
	---
	x               : [ integer ]
		Number of berries plucked

	Example call:
	---
	x=pluck_berry(client_id,berry,count,berry_positions_dictionary,local_collector)

	"""
	####################################################################################
	i=0
	cff=1.5
	a=berry_positions_dictionary.get(berry)
	print("berry pos dic in task 4 pluck berry", berry_positions_dictionary)
	print("aaaa",a)
	print(count)
	x=len(a)
	print(x)
	x=min(x,count)
	print(x)
	
	for c in range(0,x):
		b=list(a[c])
		#print(c,"   ",b)
		b[0]=cff*b[0]
		b[1]=cff*b[1]
		b[2]=(2*b[2])+0.05
		#send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
		if b[0]>=0.15:
			#pluck_left_berry(client_id,b,local_collector)
			#send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
			pluck_centre_berry(client_id,b,local_collector,berry)
			task_3.correction(client_id,90)
			
		if b[0]<=-0.15:
			#pluck_right_berry(client_id,b,local_collector)
			#send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
			pluck_centre_berry(client_id,b,local_collector,berry)
			task_3.correction(client_id,90)
			
		if b[0]<0.15 and b[0]>-0.15:
			pluck_centre_berry(client_id,b,local_collector,berry)
			#send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
			task_3.correction(client_id,90)
			
		
		c=c+1

	return x

	'''

	if len(a)<=count:                  # 0.35, 0.15
		for c in range(0,len(a)):
			b=list(a[c])
			#print(c,"   ",b)
			b[0]=cff*b[0]
			b[1]=cff*b[1]
			b[2]=1.25*b[2]
			send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
			if b[0]>=0.15:
				pluck_left_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			if b[0]<=-0.15:
				pluck_right_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			if b[0]<0.15 and b[0]>-0.15:
				pluck_centre_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			
			c=c+1

		return len(a)
    
	if len(a)>count:
		for c in range(0,count):
			
			b=list(a[c])
			#print(c,"   ",b)
			b[0]=cff*b[0]
			b[1]=cff*b[1]
			b[2]=1.25*b[2]
			send_identified_berry_data(client_id,berry,b[0],b[1],b[2])
			if b[0]>=0.15:
				pluck_left_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			if b[0]<=-0.15:
						
				pluck_right_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			if b[0]<0.15 and b[0]>-0.15:
				
				pluck_centre_berry(client_id,b,local_collector)
				task_3.correction(client_id,90)
				
			
			c=c+1

		return count
'''
	
	#######################################################3


def getpos(client_id,berry,count,lc):
	"""
	Purpose:
		To check if the berry is on left,right or at centre of the vertical rack.
		Then, move the bot 1 step left/right accordingly and call the function to pluck berry.
		After plucking, move the bot to original position.

	Input arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API
	
	berry           :  [ string ]
		Name og the berry to be plucked.
		"Blueberry","Strawberry" or "Lemon"

	count           :  [ integer ]
		Number of berries to be plucked.
	
	lc              :  [ integer ]
		1 for local collector1
		2 for local collector2

	Returns:
	---
	x               : [ integer ]
		Number of berries plucked

	Example call:
	---
	x=getpos(client_id,berry,count,lc)

	"""
	############################################################
	bpd_1=visionsensorcode(client_id)
	a1=bpd_1.get(berry)
	b=list(a1[0])
	#b0=b[0]*1.5
	#b1=b[1]*1.5
	#b2=b[2]*2
	b[0]=1.5*b[0]
	
	
	#send_identified_berry_data(client_id,berry,b0,b1,b2)
	if b[0]>0.24:
		task_3.shortest_path(client_id,[(-1,0)],6,2.5)
		berry_positions_dictionary=visionsensorcode(client_id)
		wheel_joints=task_3.init_setup(client_id)
		task_3.hault(client_id,wheel_joints)
		x=pluck_berry(client_id,berry,count,berry_positions_dictionary,lc)
		task_3.shortest_path(client_id,[(1,0)],8,2.5)

	elif b[0]<-0.24:
		task_3.shortest_path(client_id,[(1,0)],2,2)
		berry_positions_dictionary=visionsensorcode(client_id)
		berry_positions_dictionary=visionsensorcode(client_id)
		wheel_joints=task_3.init_setup(client_id)
		x=pluck_berry(client_id,berry,count,berry_positions_dictionary,lc)
		task_3.shortest_path(client_id,[(-1,0)],8,2.5)

	else:
		berry_positions_dictionary=visionsensorcode(client_id)
		berry_positions_dictionary=visionsensorcode(client_id)
		wheel_joints=task_3.init_setup(client_id)
		x=pluck_berry(client_id,berry,count,berry_positions_dictionary,lc)
	print("getpos",x)
	return x
	######################################################

def task_4_primary(client_id):

	"""
	This function was used in task_4.

	"""
	
	#target_points=[(3,3),(4,3),(4,4)]
	#task_3.task_3_primary(client_id,target_points)

	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	call_open_close(client_id,"open")

	return_code, vision_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = task_2a.get_vision_sensor_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	vision_sensor_depth_image, image_resolution, return_code = task_2a.get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_depth_image = task_2a.transform_vision_sensor_depth_image(vision_sensor_depth_image, image_resolution)
	berries_dictionary  =task_2a.detect_berries(transformed_image, transformed_depth_image)

	berry_positions_dictionary=task_2a.detect_berry_positions(berries_dictionary)
	print(berry_positions_dictionary)
	a1=berry_positions_dictionary.get('Blueberry')
	a2=berry_positions_dictionary.get('Lemon')
	a3=berry_positions_dictionary.get('Strawberry')
	b1=list(a1[0])
	b2=list(a2[0])
	b3=list(a3[0])
	print(b3)

	cff=1.5

	b1[0]=cff*b1[0]
	b1[1]=cff*b1[1]

	b2[0]=cff*b2[0]
	b2[1]=cff*b2[1]

	b3[0]=cff*b3[0]
	b3[1]=cff*b3[1]

	#send_identified_berry_data(client_id,'Blueberry',b1[0],b1[1],b1[2])
	#STRAWBERRY
	
	if b3[0]>=0.14:
		pluck_left_berry(client_id,b3)
	
	if b3[0]<=-0.14:
		pluck_right_berry(client_id,b3)

	if b3[0]<0.14 and b3[0]>-0.14:
		pluck_centre_berry(client_id,b3)

    #LEMON
	if b2[0]>=0.14:
		pluck_left_berry(client_id,b2)
	
	if b2[0]<=-0.14:
		pluck_right_berry(client_id,b2)

	if b2[0]<0.14 and b2[0]>-0.14:
		pluck_centre_berry(client_id,b2)

	#BLUEBERRY

	if b1[0]>=0.14:
		pluck_left_berry(client_id,b1)
	
	if b1[0]<=-0.14:
		pluck_right_berry(client_id,b1)

	if b1[0]<0.14 and b1[0]>-0.14:
		pluck_centre_berry(client_id,b1)

	#coll_box=[(6,4),(6,7)]
	#task_3.task_3_primary(client_id,coll_box)



if __name__ == "__main__":


	##################################################
	## You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = task_1b.init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

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

	try:

		task_4_primary(client_id)
		time.sleep(1)        

		try:
			return_code = task_1b.stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
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

	except Exception:
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()