'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 3 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ 1115 ]
# Author List:		[ Rabbi Sudheer Zacharias , D S Sai Rohith , Ramya , Nisarga  ]
# Filename:			task_3.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

from tokenize import Double
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import task_1a
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



################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################
def hault(client_id,wheel_joints):
        r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0],0,sim.simx_opmode_oneshot)
        r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1],0,sim.simx_opmode_oneshot)
        r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2],0,sim.simx_opmode_oneshot)
        r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3],0,sim.simx_opmode_oneshot) 

##############################################################


def init_remote_api_server():

	"""
	Purpose:
	---
	This function should first close any open connections and then start
	communication thread with server i.e. CoppeliaSim.
	
	Input Arguments:
	---
	None
	
	Returns:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable
	
	Example call:
	---
	client_id = init_remote_api_server()
	
	"""

	client_id = -1

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(-1)
	client_id = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
	##################################################

	return client_id


def start_simulation(client_id):

	"""
	Purpose:
	---
	This function should first start the simulation if the connection to server
	i.e. CoppeliaSim was successful and then wait for last command sent to arrive
	at CoppeliaSim server end.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the start running simulation remote API
	
	Example call:
	---
	return_code = start_simulation()
	
	"""
	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	return_code=sim.simxStartSimulation(client_id,sim.simx_opmode_oneshot)
	##################################################

	return return_code


def get_vision_sensor_image(client_id):
	
	"""
	Purpose:
	---
	This function should first get the handle of the Vision Sensor object from the scene.
	After that it should get the Vision Sensor's image array from the CoppeliaSim scene.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
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


	return_code = 0
	res,v0=sim.simxGetObjectHandle(client_id,'vision_sensor_1',sim.simx_opmode_oneshot_wait)
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, v0 ,0,sim.simx_opmode_oneshot_wait)
	##############	ADD YOUR CODE HERE	##############
 	
	
	##################################################

	return vision_sensor_image, image_resolution, return_code


def transform_vision_sensor_image(vision_sensor_image, image_resolution):

	"""
	Purpose:
	---
	This function should:
	1. First convert the vision_sensor_image list to a NumPy array with data-type as uint8.
	2. Since the image returned from Vision Sensor is in the form of a 1-D (one dimensional) array,
	the new NumPy array should then be resized to a 3-D (three dimensional) NumPy array.
	3. Change the color of the new image array from BGR to RGB.
	4. Flip the resultant image array about the X-axis.
	The resultant image NumPy array should be returned.
	
	Input Arguments:
	---
	`vision_sensor_image` 	:  [ list ]
		the image array returned from the get vision sensor image remote API
	`image_resolution` 		:  [ list ]
		the image resolution returned from the get vision sensor image remote API
	
	Returns:
	---
	`transformed_image` 	:  [ numpy array ]
		the resultant transformed image array after performing above 4 steps
	
	Example call:
	---
	transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
	
	"""


	
	
 	
	##############	ADD YOUR CODE HERE	##############
	transformed_image = None
	arr=np.array(vision_sensor_image)
	arr1=np.uint8(arr)
	resize_array=np.reshape(arr1,(image_resolution[0],image_resolution[1],3))
	rgb = cv2.cvtColor(resize_array, cv2.COLOR_BGR2RGB)
	transformed_image = cv2.flip(rgb, 1)
	#cv2.imwrite('test.png',transformed_image)
 
	##################################################
	
	return transformed_image


def stop_simulation(client_id):
	"""
	Purpose:
	---
	This function should stop the running simulation in CoppeliaSim server.
	NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
		  It is already written in the main function.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	`return_code` 	:  [ integer ]
		the return code generated from the stop running simulation remote API
	
	Example call:
	---
	return_code = stop_simulation()
	
	"""

	return_code = -2

	##############	ADD YOUR CODE HERE	##############
	return_code = sim.simxStopSimulation(client_id,sim.simx_opmode_oneshot)
	##################################################

	return return_code


def exit_remote_api_server(client_id):
	
	"""
	Purpose:
	---
	This function should wait for the last command sent to arrive at the Coppeliasim server
	before closing the connection and then end the communication thread with server
	i.e. CoppeliaSim using simxFinish Remote API.
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	None
	
	Example call:
	---
	exit_remote_api_server()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	sim.simxFinish(client_id)
	##################################################


def detect_qr_codes(transformed_image):
	
	"""
	Purpose:
	---
	This function receives the transformed image from the vision sensor and detects qr codes in the image

	Input Arguments:
	---
	`transformed_image` 	:  [ numpy array ]
		the transformed image array
	
	Returns:
	---
	None
	
	Example call:
	---
	detect_qr_codes()
	
	"""

	##############	ADD YOUR CODE HERE	##############
	qr_codes=list()
	result = decode(transformed_image)
	for code in result:
		codeData = code.data.decode()
		x,y,w,h=code.rect
		qr_codes.append(codeData)
	##################################################
	
	return qr_codes


def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):

	"""
	Purpose:
	---
	This function takes desired forward/back, left/right, rotational velocites of the bot as input arguments.
	It should then convert these desired velocities into individual joint velocities(4 joints) and actuate the joints
	accordingly.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints

	`forw_back_vel'     :   [ float ]
		Desired forward/back velocity of the bot

	`left_right_vel'    :   [ float ]
		Desired left/back velocity of the bot
	
	`rot_vel'           :   [ float ]
		Desired rotational velocity of the bot
	
	Returns:
	---
	None
	
	Example call:
	---
	set_bot_movement(client_id, wheel_joints, 0.5, 0, 0)
	
	"""
	if(abs(forw_back_vel)>0 and abs(left_right_vel)==0) :
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0] , forw_back_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1] , forw_back_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2] , forw_back_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3] , forw_back_vel, sim.simx_opmode_oneshot)
	if(abs(forw_back_vel)>0 and abs(left_right_vel)>0):
		if(left_right_vel>0):
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0] , 0, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1] , forw_back_vel, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2] , forw_back_vel, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3] , 0, sim.simx_opmode_oneshot)
		else:
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0] , forw_back_vel, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1] , 0, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2] , 0, sim.simx_opmode_oneshot)
			r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3] , forw_back_vel, sim.simx_opmode_oneshot)
	if (abs(left_right_vel)>0 and abs(forw_back_vel)==0):
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0] , left_right_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1] , -1*left_right_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2] , -1*left_right_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3] , left_right_vel, sim.simx_opmode_oneshot)	
	if (abs(rot_vel)>0 and abs(forw_back_vel)==0):
     	
      
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[1] , -1*rot_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[2] , rot_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[3] , -1*rot_vel, sim.simx_opmode_oneshot)
		r1=sim.simxSetJointTargetVelocity(client_id,wheel_joints[0] , rot_vel, sim.simx_opmode_oneshot)

	##################################################


def init_setup(client_id):
	
	"""
	Purpose:
	---
	This function will get the object handles of all the four joints in the bot, store them in a list
	and return the list

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'wheel_joints`      :   [ list]
		Python list containing joint object handles of individual joints
	
	Example call:
	---
	init setup(client_id)
	
	"""

	##############	ADD YOUR CODE HERE	##############
	wheel_joints=list()
	res,fl=sim.simxGetObjectHandle(client_id,'rollingJoint_fl',sim.simx_opmode_oneshot_wait)
	res,fr=sim.simxGetObjectHandle(client_id,'rollingJoint_fr',sim.simx_opmode_oneshot_wait)
	res,rl=sim.simxGetObjectHandle(client_id,'rollingJoint_rl',sim.simx_opmode_oneshot_wait)
	res,rr=sim.simxGetObjectHandle(client_id,'rollingJoint_rr',sim.simx_opmode_oneshot_wait)

	wheel_joints.append(fl)
	wheel_joints.append(fr)
	wheel_joints.append(rl)
	wheel_joints.append(rr)	

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,0.1),[],bytearray(),sim.simx_opmode_blocking)
	##################################################

	return wheel_joints


def encoders(client_id):

	"""
	Purpose:
	---
	This function will get the `combined_joint_position` string signal from CoppeliaSim, decode it
	and return a list which contains the total joint position of all joints    

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	
	Returns:
	---
	'joints_position`      :   [ list]
		Python list containing the total joint position of all joints
	
	Example call:
	---
	encoders(client_id)
	
	"""

	return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")
	

	#if len(joints_position)>1:
	for index,joint_val in enumerate(joints_position):
		#print("In encoders",joints_position)
		joints_position[index]=float(joint_val)
	#else:
		#joint_positions=[0.0,0.0,0.0,0.0]


	return joints_position
def nav_logic(client_id,moves,QR_code,vf,vs):

	
    #vf=8
    #vs=2.8
    vf=vf
    vs=vs
	#moves[]={diagonal left, diagonal right, vertical, horizontal}
    

    if(moves[0]>0):
        a=0
        count=0
        QR=list()
        while count<=moves[0]:
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            wheel_joints=init_setup(client_id)
            if(count<=moves[0]-1):
                set_bot_movement(client_id,wheel_joints,vf,vf,0)
               # rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            else:
                set_bot_movement(client_id,wheel_joints,vs,vs,0)   
                #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
 
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)
     
    elif moves[0]<0:
        a=0
        count=0
        QR=list()
        while count<=abs(moves[0]):
            wheel_joints=init_setup(client_id)
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)

            if(count<=abs(moves[0])-1):
                set_bot_movement(client_id,wheel_joints,-vf,vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,-vs,vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)           
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)
        
  
    if(moves[1]>0):
        a=0
        count=0
        QR=list()
        while count<=moves[1]:
            wheel_joints=init_setup(client_id)
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            if(count<=moves[1]-1):
                set_bot_movement(client_id,wheel_joints,vf,-vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,vs,-vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)                        
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)
  
    elif moves[1]<0:
        a=0
        count=0
        QR=list()
        while count<=abs(moves[1]):
            wheel_joints=init_setup(client_id)
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            if(count<=abs(moves[1])-1):
                set_bot_movement(client_id,wheel_joints,-vf,-vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,-vs,-vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)         
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)  
   
    if(moves[2]>0):
        a=0
        count=0
        QR=list()
        while count<=moves[2]:

            #jp=encoders(client_id)
            #print("encoder error",jp)
            wheel_joints=init_setup(client_id)
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            if(count<=moves[2]-1):
                set_bot_movement(client_id,wheel_joints,vf,0,0)
            else:
                set_bot_movement(client_id,wheel_joints,vs,0,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)           
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                    
                a=i[0]                                      
        #print(QR)
 
    elif moves[2]<0:
        a=0
        count=0
        QR=list()       
        
		
        while count<=abs(moves[2]):
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            wheel_joints=init_setup(client_id)
            if(count<=abs(moves[2])-1):
                set_bot_movement(client_id,wheel_joints,-vf,0,0)
            else:
                set_bot_movement(client_id,wheel_joints,-vs,0,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
			
            i=detect_qr_codes(transformed_image)           
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)
     
    if(moves[3]>0):
        a=0
        count=0
        QR=list()
        while count<=moves[3]:
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
            wheel_joints=init_setup(client_id)
            if(count<=moves[3]-1):
                set_bot_movement(client_id,wheel_joints,0,vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,0,vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)
            
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)


    elif moves[3]<0:
        a=0
        count=0
        QR=list()
        #joint_positions=[0.0,0.0,0.0,0.0]
        while count<=abs(moves[3]):
            #rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setdummy',[],(0,0.2,-0.05),[],bytearray(),sim.simx_opmode_blocking)
        #while abs(joint_positions[0])<12.5:
            wheel_joints=init_setup(client_id)
            if(count<=abs(moves[3])-1):
                #joint_positions=encoders(client_id)
                #v=(12.5-joint_positions[0])
                set_bot_movement(client_id,wheel_joints,0,-vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,0,-vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            #joint_positions=encoders(client_id)
            #print(joint_positions)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)            
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])
                a=i[0]                                      
        #print(QR)
    #cv2.imwrite('test.png', transformed_image)	
	

    hault(client_id,wheel_joints)

'''
    elif moves[3]<0:
       a=0
       count=0
        QR=list()
        while count<=abs(moves[3]):
            wheel_joints=init_setup(client_id)
            if(count<=abs(moves[3])-1):
                set_bot_movement(client_id,wheel_joints,0,-vf,0)
            else:
                set_bot_movement(client_id,wheel_joints,0,-vs,0)    
            vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
            transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
            i=detect_qr_codes(transformed_image)            
            if len(i)>0:
                if i[0]!=a:
                    count=count+1
                    QR.append(i[0])`
                a=i[0]                                      
        print(QR)
    #cv2.imwrite('test.png', transformed_image)
'''
def rotation(client_id,i,angle):   # i is 1 for clock wise rotation and ois -1 for anticlockwise rotation


	rc,jph=sim.simxGetObjectHandle(client_id,"rolling_joint_fl",sim.simx_opmode_blocking)
	sim.simxSetJointPosition(client_id,jph,0.0,sim.simx_opmode_oneshot)
	#print(encoders(client_id))
	joint_positions=[0.0,0.0,0.0,0.0]
	joint_positions=encoders(client_id)
	initial_position=joint_positions[0]


	ev=12.4*angle/90     #12.5      #encoder value
	#print("outside while ",joint_positions)
	#print("outside while ",initial_position)


	if (i>0):
		while (joint_positions[0])<(initial_position)+ev:
			wheel_joints=init_setup(client_id)
			joint_positions=encoders(client_id)
			#print("i>0",joint_positions)
			v=(i*abs(abs(initial_position+ev)-abs(joint_positions[0])))*1.5
			set_bot_movement(client_id,wheel_joints,0,0,v+i*0.05)

	elif(i<0):
		while (joint_positions[0])>(initial_position-ev):
			wheel_joints=init_setup(client_id)
			joint_positions=encoders(client_id)
			#print("i<0",joint_positions)
			v=(i*abs((abs(initial_position-ev)-abs(joint_positions[0]))))*1.5
			set_bot_movement(client_id,wheel_joints,0,0,v+i*0.05)
				

def correction(client_id,a):
	vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
	transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
	angel = task_1a.detect_shapes(transformed_image)
	#print("snd",angel)
	#print(angel)
	angle=a
	#x=45-angel
	#x=45.0/x
	#rotation(client_id,x)
	x=(angle/2)-angel
	x=(angle/2)/x

	while abs(x)>1.001:

		vision_sensor_image,image_resolution,rs=get_vision_sensor_image(client_id)
		transformed_image=transform_vision_sensor_image(vision_sensor_image,image_resolution)
		angel = task_1a.detect_shapes(transformed_image)
		#print("insidewhile",angel)
		#print(angel)
	
		x=(angle/2)-angel
		x=(angle/2)/x
		joint_positions=[]
		wheel_joints=init_setup(client_id)
		joint_positions=encoders(client_id)
		#print("i<0",joint_positions)
		v=((abs(x)-1)*x/abs(x))+(0.025*x/abs(x))
		set_bot_movement(client_id,wheel_joints,0,0,v)

def shortest_path(client_id,target_points,vf,vs):
    a=0
    b=0
    QR_codes=list()
    for i in target_points:
        FW=LR=LD=RD=0
        a1=i[0]-a
        b1=i[1]-b
        if (abs(a1)>=abs(b1)):
            if(a1>0 and b1>0)or(a1<0 and b1<0):
                LD=b1
            else:
                RD=b1
                
            if a1>=0:
                LR=abs(a1)-abs(b1)
            elif a1<0:
                LR=-1*(abs(a1)-abs(b1))
            FW=0         
        elif (abs(a1)<abs(b1) and a1!=b1):
            if((a1>0 and b1>0)or(a1<0 and b1<0)):
                LD=a1
            else:
                RD=-1*a1
            if b1>0:
                FW=abs(b1)-abs(a1)
            elif b1<0:
                FW=-1*(abs(b1)-abs(a1)) 
         
        moves=list()
        moves.append(RD)
        moves.append(LD)
        moves.append(FW)
        moves.append(LR)
        #print(moves)
        nav_logic(client_id,moves,QR_codes,vf,vs)
        
        a=i[0]
        b=i[1]
        
def task_3_primary(client_id, target_points):
	
	"""
	Purpose:
	---
	
	# NOTE:This is the only function that is called from the main function and from the executable.
	
	Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic. 
	The bot should traverses all the target navigational co-ordinates.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`target_points`     : [ list ]
		List of tuples where tuples are the target navigational co-ordinates.
	
	Returns:
	---
	
	Example call:
	---
	target_points(client_id, target_points)
	
	"""
	
	

	
	



if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(0,0)]    # You can give any number of different co-ordinates
	target_points1 = [(0,2)]
	target_points1 = [(0,-4)]

	##################################################
	## NOTE: You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = start_simulation(client_id)

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

		task_3_primary(client_id, target_points)
		time.sleep(1)        

		try:
			return_code = stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					exit_remote_api_server(client_id)
					if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
		print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()