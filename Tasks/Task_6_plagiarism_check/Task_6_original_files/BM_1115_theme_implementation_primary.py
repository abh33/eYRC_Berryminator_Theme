'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement the Berryminator(BM) Theme (eYRC 2021-22).
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
# Author List:		[Rabbi S Zacharias, Ramya , Nisarga B,D S SaiRohith]
# Filename:			theme_implementation.py
# Functions:		updation_room_entry(list),enter_room(int,int,list),exit_room(int,int,list),drop_fruit(int,int)
#					decode_json(),fruit_count(int),room_priority_logic(dict,dict,dict,dict,dict,dict),CB_1_CB_2_max_update(dict,dict)
#					update_CB1_CB2(dict,dict,dict,dict),update_room_fruit_count(dict,dict,dict,dict,dict,dict,int)
#					pluck_logic(int,dict,dict),theme_implementation_primary(int,list)
# 						
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################


from cmath import sqrt
from http import client
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
import json
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
import task_4


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################


##############################################################
def updation_room_entry(rooms_entry):

	"""
	Purpose:
	---
	This function takes the rooms_entry list of tuples and arranges it in order wise.
	Room 1  :- top left
	Room 2  :- top right
	Room 3  :- botton right
	Room 4  :- botton left

	arranges as per the rulebook 

	Input Arguments:
	---
	`rooms_entry`    :   [ list of tuples ]
		entry coordinates of all 4 rooms

	
	Returns:
	---
	`new_rooms_entry` 	:  [ list of tuples ]
		updated rooms_entry order wise
	
	
	Example call:
	---
	new_rooms_entry=updation_room_entry(rooms_entry)
	"""
	################################################################################
	### UPDTATION OF ROOM Entry COORDINATES to shifted Origin ###
	#rooms_entry = [(3,2),(6,5),(3,6),(8,3)]
	#print(rooms_entry) 
	new_rooms_entry=[(0,0),(0,0),(0,0),(0,0)]
	for i in range (0,4):
		a=rooms_entry[i][0]
		b=rooms_entry[i][1]
		rooms_entry[i]=(a-4,b-4)
		#print(i)
		
	#print(rooms_entry)

	for i in range (0,4):
		a=rooms_entry[i]
		if(a[0]<0 and a[1]>0):
			new_rooms_entry[0]=a
		elif(a[0]>0 and a[1]>0):
			new_rooms_entry[1]=a
		elif(a[0]>0 and a[1]<0):
			new_rooms_entry[2]=a
		elif(a[0]<0 and a[1]<0):
			new_rooms_entry[3]=a

	#print(new_rooms_entry)
	#return new_rooms_entry   # returns room entries with shifted origin and also arranged room number wise

	for i in range (0,4):
		for j in range (0,i):
			a=new_rooms_entry[i][0]
			b=new_rooms_entry[i][1]
			new_rooms_entry[i]=(-b,a)

	#print(new_rooms_entry)
	return new_rooms_entry

############################################################################


def enter_room(client_id,room_number,new_rooms_entry):

	"""
	Purpose:
	---
	This function takes the client id ,room number and the updated room entry 
	list of tuples and navigates to the room specified by the room number from home position accordingly.

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`room _number`    :   [ integer ]
		Room 1 = 1
		Room 2 = 2
		Room 3 = 3
		Room 4 = 4
	`new_rooms_entry`    :   [list of tuples ]
		updated room entries order wise

	Returns:
	---
	no value
	
	Example call:
	---
	enter_room(client_id,3,new_rooms_entry)
	"""
################################################################################3
######## Setting Up #####
	vf=8
	vs=3.5
	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)
######## Setting Up #####



	for i in range (0,(room_number-1)):
		task_3.rotation(client_id,1,90)
	task_3.correction(client_id,90)

	if new_rooms_entry[room_number-1]==(-4,1):
		task_3.shortest_path(client_id,[(-3,0)],7,3)
		task_3.shortest_path(client_id,[(-1,1)],vf,vs)
		task_3.shortest_path(client_id,[(0,1)],vf,vs)
		task_3.shortest_path(client_id,[(1,1)],vf,vs)
		task_3.hault(client_id,wheel_joints)

	if new_rooms_entry[room_number-1]==(-2,1):
		task_3.shortest_path(client_id,[(-3,0)],7,vs)
		task_3.shortest_path(client_id,[(1,1)],7.5,2.5)
		task_3.shortest_path(client_id,[(0,1)],vf,vs)
		task_3.shortest_path(client_id,[(-1,1)],vf,vs)
		task_3.hault(client_id,wheel_joints)

	if new_rooms_entry[room_number-1]==(-1,2):
		task_3.shortest_path(client_id,[(0,3)],7,vs)
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(-1,1)],vf,5)
		task_3.shortest_path(client_id,[(0,1)],vf,3)
		task_3.shortest_path(client_id,[(1,1)],vf,2.5)
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)

##############################################################################################


def exit_room(client_id,room_number,new_rooms_entry):
	"""
	Purpose:
	---
	This function takes the client id ,room number and the updated room entry 
	list of tuples and exits from the room specified by the room number and comes to hault at home position.

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`room _number`    :   [ integer ]
		Room 1 = 1
		Room 2 = 2
		Room 3 = 3
		Room 4 = 4
	`new_rooms_entry`    :   [list of tuples ]
		updated room entries order wise

	Returns:
	---
	no value
	
	Example call:
	---
	enter_room(client_id,3,new_rooms_entry)
	"""

	###################################################################################

######## Setting Up #####
	vf=8.5
	vs=3.5
	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)
######## Setting Up #####

	if new_rooms_entry[room_number-1]==(-4,1):
		task_3.shortest_path(client_id,[(-1,-1)],vf,vs)
		task_3.shortest_path(client_id,[(0,-1)],vf,vs)
		task_3.shortest_path(client_id,[(1,-1)],vf,vs)
		task_3.shortest_path(client_id,[(3,0)],7,2.5)
		task_3.hault(client_id,wheel_joints)

	if new_rooms_entry[room_number-1]==(-2,1):
		task_3.shortest_path(client_id,[(1,-1)],vf,vs)
		task_3.shortest_path(client_id,[(0,-2)],7.5,vs)
		#task_3.shortest_path(client_id,[(-1,-1)],vf,vs)
		task_3.shortest_path(client_id,[(2,0)],7,3)
		task_3.hault(client_id,wheel_joints)

	if new_rooms_entry[room_number-1]==(-1,2):

		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(1,1)],vf,vs)
		task_3.shortest_path(client_id,[(0,2)],vf,vs)
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,-2)],7,3)
		task_3.hault(client_id,wheel_joints)



	for i in range (0,room_number-1):
		task_3.rotation(client_id,-1,90)
	#task_3.correction(client_id,90)	
	
	#######################################################################	


def drop_fruit(client_id,flag):

	"""
	Purpose:
	---
	This function takes the client id and flag returned from CB_1_CB_2_max_update() function 
	and drops the plucked fruits from local collectors to  collector box 1 and collector box 2
	

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`flag`    :   [ integer ]
		if flag = -1 then drops maximum number of fruits from local collector 1 to collector box 1 
		and less fruits from local collector 2 into collection box 2
		if flag = 1 then drops maximum number of fruits from local collector 1 to collector box 2
		and less fruits from local collector 2 into collection box 1
	
	Returns:
	---
	no value
	
	Example call:
	---
	drop_fruit(client_id,-1)
	"""

	#############################################################################

	vf=8
	vs=3
	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)

	task_3.shortest_path(client_id,[(0,4)],6,vs)
	task_3.shortest_path(client_id,[(0,3)],5,2)


	if (flag>0):

		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,2)],7,vs)

		task_3.hault(client_id,wheel_joints)
		task_3.correction(client_id,90)
		task_3.rotation(client_id,-1,0.25)

		#drop in CB2 from local collector 1
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket',sim.sim_scripttype_childscript,'setdummy',[],[-0.058,0.055,0.21],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(5)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,-2)],8.5,2)
		task_3.shortest_path(client_id,[(0,-2)],7.5,2)
		#task_3.hault(client_id,wheel_joints)
		#task_3.correction(client_id,90)
		#task_3.shortest_path(client_id,[(0,-2)],vf,vs)
		task_3.hault(client_id,wheel_joints)
		#task_3.shortest_path(client_id,[(0,-1)],vf,vs)
		#task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,1)
		#drop in CB1 from LC2
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket_2',sim.sim_scripttype_childscript,'setdummy',[],[-0.036,0.054,-0.62],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(10)	
		
	elif(flag<0):
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,2)],vf,vs)

		task_3.hault(client_id,wheel_joints)
		task_3.correction(client_id,90)

		#drop in CB1 from local collector 1
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket',sim.sim_scripttype_childscript,'setdummy',[],[-0.058,0.055,0.21],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(5)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,-4)],vf,vs)
		#task_3.hault(client_id,wheel_joints)
		#task_3.correction(client_id,90)
		#task_3.shortest_path(client_id,[(0,-2)],vf,vs)
		task_3.hault(client_id,wheel_joints)
		#task_3.shortest_path(client_id,[(0,-1)],vf,vs)
		#task_3.hault(client_id,wheel_joints)
		task_3.correction(client_id,50)
		#drop in CB2 from LC2
		rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket_2',sim.sim_scripttype_childscript,'setdummy',[],[-0.036,0.054,-0.62],[],bytearray(),sim.simx_opmode_blocking)
		time.sleep(10)	

	##############################################################################


def decode_json():
	"""
	Purpose:
	---
	This function decodes the JSON file in the folder of task 6
	if exe changes the JSON file values then it decodes accordingly

	Input Arguments:
	---
	no parameter
	
	Returns:
	---
	`CB_1` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 1
	`CB_2` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 2
	
	
	Example call:
	---
	decode_json()
	"""

	###################################################################################
	f=open('Theme_Config.json')
	data=json.load(f)
	CB_1=dict()
	CB_2=dict()

	blueberry=data['B']
	#print(len(blueberry))
	if len(blueberry)==2:
		blueberry=data['B']
	else:
		blueberry=[data['B']]

	for i in blueberry:
		#print(i)
		a=i.split('_')
		#print(a)
		for j in a:
			if a[1]=='CB1':
				CB_1["bb"]=a[0]
			if a[1]=='CB2':
				CB_2["bb"]=a[0]


	lemon=data['L']
	if len(lemon)==2:
		lemon=data['L']
	else:
		lemon=[data['L']]
	for p in lemon:
		b=p.split('_')
		for j in b:
			if b[1]=='CB1':
				CB_1["lmn"]=b[0]
			if b[1]=='CB2':
				CB_2["lmn"]=b[0]
				

	strawberry=data['S']
	if len(strawberry)==2:
		strawberry=data['S']
	else:
		strawberry=[data['S']]
	for k in strawberry:
		s=k.split('_')
		for j in s:
			if s[1]=='CB1':
				CB_1["sb"]=s[0]
			if s[1]=='CB2':
				CB_2["sb"]=s[0]
			

	if CB_1.get('bb')==None:
		CB_1['bb']=0

	if CB_1.get('lmn')==None:
		CB_1['lmn']=0

	if CB_1.get('sb')==None:
		CB_1['sb']=0

	if CB_2.get('bb')==None:
		CB_2['bb']=0

	if CB_2.get('lmn')==None:
		CB_2['lmn']=0

	if CB_2.get('sb')==None:
		CB_2['sb']=0

	#print("CB_1: ",CB_1)
	#print("CB_2: ",CB_2)
	return CB_1,CB_2

################################################################################################


def fruit_count(client_id):     # blueberry , Lemon, strawberry
	"""
	Purpose:
	---
	This function when called gazez the arena and checks which room has what fruits and their respective count.
	This can be used to get the importance of a room ( room priority) when the JSON file values are changed.

	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	
	Returns:
	---
	`room_1` 	:  [ dictionary ]
		Checks room 1 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_2` 	:  [ dictionary ]
		Checks room 2 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_3` 	:  [ dictionary ]
		Checks room 3 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_4` 	:  [ dictionary ]
		Checks room 4 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	
	Example call:
	---
	room_1,room_2,room_3,room_4=fruit_count(client_id)
	"""
	########################################################################
	room_fruit_count=[(0,0,0),(0,0,0),(0,0,0),(0,0,0)]
	vf=7
	vs=2
	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)


	#Traverse and check how many rooms have how many fruits by going to common points
	#From home to check and come back to home again
 ###################################################
	task_3.rotation(client_id,1,55)
	berry_positions_dictionary2=task_4.visionsensorcode(client_id) #checks room 2
	a1=berry_positions_dictionary2.get('Blueberry')
	a2=berry_positions_dictionary2.get('Lemon')
	a3=berry_positions_dictionary2.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[1]=(b1,b2,b3)
	room_2=dict()
	room_2["bb_2"]=len(a1)
	room_2["sb_2"]=len(a3)
	room_2["lmn_2"]=len(a2)


	task_3.rotation(client_id,1,90)
	berry_positions_dictionary3=task_4.visionsensorcode(client_id) #checks room 3
	a1=berry_positions_dictionary3.get('Blueberry')
	a2=berry_positions_dictionary3.get('Lemon')
	a3=berry_positions_dictionary3.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[2]=(b1,b2,b3)
	room_3=dict()
	room_3["bb_3"]=len(a1)
	room_3["sb_3"]=len(a3)
	room_3["lmn_3"]=len(a2)


	task_3.rotation(client_id,1,90)
	berry_positions_dictionary4=task_4.visionsensorcode(client_id) #checks room 4
	a1=berry_positions_dictionary4.get('Blueberry')
	a2=berry_positions_dictionary4.get('Lemon')
	a3=berry_positions_dictionary4.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[3]=(b1,b2,b3)
	room_4=dict()
	room_4["bb_4"]=len(a1)
	room_4["sb_4"]=len(a3)
	room_4["lmn_4"]=len(a2)

	task_3.rotation(client_id,1,90)
	berry_positions_dictionary1=task_4.visionsensorcode(client_id) #checks room 1
	a1=berry_positions_dictionary1.get('Blueberry')
	a2=berry_positions_dictionary1.get('Lemon')
	a3=berry_positions_dictionary1.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[0]=(b1,b2,b3)
	room_1=dict()
	room_1["bb_1"]=len(a1)
	room_1["sb_1"]=len(a3)
	room_1["lmn_1"]=len(a2)

	task_3.rotation(client_id,1,35)
	task_3.correction(client_id,90)

	return room_1,room_2,room_3,room_4



	'''
	task_3.shortest_path(client_id,[(-3,0)],vf,vs)   # go to see room 1 and 2
	task_3.correction(client_id,90)
	#room_fruit_count[0]=task_4.visionsensorcode(client_id) #checks room 1
	berry_positions_dictionary1=task_4.visionsensorcode(client_id) #checks room 1
	a1=berry_positions_dictionary1.get('Blueberry')
	a2=berry_positions_dictionary1.get('Lemon')
	a3=berry_positions_dictionary1.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[0]=(b1,b2,b3)
	room_1=dict()
	room_1["bb_1"]=len(a1)
	room_1["sb_1"]=len(a3)
	room_1["lmn_1"]=len(a2)
	

	
	time.sleep(1)

	task_3.rotation(client_id,-1,180)
	#task_3.rotation(client_id,-1,90)
	task_3.correction(client_id,50)
	#room_fruit_count[3]=task_4.visionsensorcode(client_id) #checks room 4
	berry_positions_dictionary4=task_4.visionsensorcode(client_id) #checks room 4
	a1=berry_positions_dictionary4.get('Blueberry')
	a2=berry_positions_dictionary4.get('Lemon')
	a3=berry_positions_dictionary4.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[3]=(b1,b2,b3)
	room_4=dict()
	room_4["bb_4"]=len(a1)
	room_4["sb_4"]=len(a3)
	room_4["lmn_4"]=len(a2)
	

	task_3.correction(client_id,90)

	task_3.shortest_path(client_id,[(-3,0)],vf,vs)
	task_3.shortest_path(client_id,[(-3,0)],vf,vs)
	#room_fruit_count[2]=task_4.visionsensorcode(client_id) #checks room 3
	berry_positions_dictionary3=task_4.visionsensorcode(client_id) #checks room 3
	a1=berry_positions_dictionary3.get('Blueberry')
	a2=berry_positions_dictionary3.get('Lemon')
	a3=berry_positions_dictionary3.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[2]=(b1,b2,b3)
	room_3=dict()
	room_3["bb_3"]=len(a1)
	room_3["sb_3"]=len(a3)
	room_3["lmn_3"]=len(a2)
	


	#task_3.rotation(client_id,-1,90)
	task_3.rotation(client_id,-1,180)
	task_3.correction(client_id,50)

	#room_fruit_count[1]=task_4.visionsensorcode(client_id) #checks room 2
	berry_positions_dictionary2=task_4.visionsensorcode(client_id) #checks room 2
	a1=berry_positions_dictionary2.get('Blueberry')
	a2=berry_positions_dictionary2.get('Lemon')
	a3=berry_positions_dictionary2.get('Strawberry')
	b1=len(a1)
	b2=len(a2)
	b3=len(a3)
	room_fruit_count[1]=(b1,b2,b3)
	room_2=dict()
	room_2["bb_2"]=len(a1)
	room_2["sb_2"]=len(a3)
	room_2["lmn_2"]=len(a2)
	task_3.correction(client_id,90)
	task_3.shortest_path(client_id,[(-3,0)],vf,vs) # home position

'''
	#return room_fruit_count
	
	#################################################################



###### Takes CB_1,CB_2,room_1,room_2,room_2,room_4 and returns 1,2,3,4 based on room priority #######
def room_priority_logic(CB_1,CB_2,room_1,room_2,room_3,room_4):
	
	"""
	Purpose:
	---
	This function takes CB_1 and CB_2 to get the information on type of fruit 
	and number of fruits to be collection box 1 and collection box 2.
	It also take the information of what rooms have how many fruits and their types.
	By using this it tells us which room is the best to enter instead of going into random rooms.
	
	Input Arguments:
	---
 	`CB_1` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 1
	`CB_2` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 2
	`room_1` 	:  [ dictionary ]
		Checks room 1 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_2` 	:  [ dictionary ]
		Checks room 2 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_3` 	:  [ dictionary ]
		Checks room 3 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_4` 	:  [ dictionary ]
		Checks room 4 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	
	Returns:
	---
	`1` 	:  [ Integer]
		if room 1 is the best room to enter 
	`2` 	:  [ Integer]
		if room 2 is the best room to enter 
	`3` 	:  [ Integer]
		if room 3 is the best room to enter 
	`4` 	:  [ Integer]
		if room 4 is the best room to enter 
	`-1` 	:  [ Integer]
		If error 
	
	Example call:
	---
	x=room_priority_logic(CB_1,CB_2,room_1,room_2,room_3,room_4)
	
	"""

	####################################################################3
	
	##check difference
	diff_r1_cb1_bb=int(CB_1.get("bb"))-int(room_1.get("bb_1"))
	diff_r1_cb1_lmn=int(CB_1.get("lmn"))-int(room_1.get("lmn_1"))
	diff_r1_cb1_sb=int(CB_1.get("sb"))-int(room_1.get("sb_1"))
	if(diff_r1_cb1_bb<0):
		diff_r1_cb2_bb=int(CB_2.get("bb"))-int(abs(diff_r1_cb1_bb))
	else:
		diff_r1_cb2_bb=int(CB_2.get("bb"))
	if(diff_r1_cb1_lmn<0):
		diff_r1_cb2_lmn=int(CB_2.get("lmn"))-int(abs(diff_r1_cb1_lmn))
	else:
		diff_r1_cb2_lmn=int(CB_2.get("lmn"))
	if(diff_r1_cb1_sb<0):
		diff_r1_cb2_sb=int(CB_2.get("sb"))-int(abs(diff_r1_cb1_sb))
	else:
		diff_r1_cb2_sb=int(CB_2.get("sb"))

	#diff_r1_cb2_bb=int(CB_1.get("bb"))-int(room_1.get("bb_1"))
	#diff_r1_cb2_lmn=int(CB_1.get("lmn"))-int(room_1.get("lmn_1"))
	#diff_r1_cb2_sb=int(CB_1.get("sb"))-int(room_1.get("sb_1"))

	netdiff_r1=diff_r1_cb1_bb+diff_r1_cb1_lmn+diff_r1_cb1_sb+diff_r1_cb2_bb+diff_r1_cb2_lmn+diff_r1_cb2_sb

	#print("nett_diff_r1",netdiff_r1)

	#room_2_berry_difference

	

	##check difference
	
	diff_r2_cb1_bb=int(CB_1.get("bb"))-int(room_2.get("bb_2"))
	diff_r2_cb1_lmn=int(CB_1.get("lmn"))-int(room_2.get("lmn_2"))
	diff_r2_cb1_sb=int(CB_1.get("sb"))-int(room_2.get("sb_2"))
	if(diff_r2_cb1_bb<0):
		diff_r2_cb2_bb=int(CB_2.get("bb"))-int(abs(diff_r2_cb1_bb))
	else:
		diff_r2_cb2_bb=int(CB_2.get("bb"))
	if(diff_r2_cb1_lmn<0):
		diff_r2_cb2_lmn=int(CB_2.get("lmn"))-int(abs(diff_r2_cb1_lmn))
	else:
		diff_r2_cb2_lmn=int(CB_2.get("lmn"))
	if(diff_r2_cb1_sb<0):
		diff_r2_cb2_sb=int(CB_2.get("sb"))-int(abs(diff_r2_cb1_sb))
	else:
		diff_r2_cb2_sb=int(CB_2.get("sb"))

	#diff_r1_cb2_bb=int(CB_1.get("bb"))-int(room_1.get("bb_1"))
	#diff_r1_cb2_lmn=int(CB_1.get("lmn"))-int(room_1.get("lmn_1"))
	#diff_r1_cb2_sb=int(CB_1.get("sb"))-int(room_1.get("sb_1"))

	netdiff_r2=diff_r2_cb1_bb+diff_r2_cb1_lmn+diff_r2_cb1_sb+diff_r2_cb2_bb+diff_r2_cb2_lmn+diff_r2_cb2_sb

	#print("nett_diff_r2",netdiff_r2)

	#room3_berry_differnce

	

	

	##check difference
	
	diff_r3_cb1_bb=int(CB_1.get("bb"))-int(room_3.get("bb_3"))
	diff_r3_cb1_lmn=int(CB_1.get("lmn"))-int(room_3.get("lmn_3"))
	diff_r3_cb1_sb=int(CB_1.get("sb"))-int(room_3.get("sb_3"))
	if(diff_r3_cb1_bb<0):
		diff_r3_cb2_bb=int(CB_2.get("bb"))-int(abs(diff_r3_cb1_bb))
	else:
		diff_r3_cb2_bb=int(CB_2.get("bb"))
	if(diff_r3_cb1_lmn<0):
		diff_r3_cb2_lmn=int(CB_2.get("lmn"))-int(abs(diff_r3_cb1_lmn))
	else:
		diff_r3_cb2_lmn=int(CB_2.get("lmn"))
	if(diff_r3_cb1_sb<0):
		diff_r3_cb2_sb=int(CB_2.get("sb"))-int(abs(diff_r3_cb1_sb))
	else:
		diff_r3_cb2_sb=int(CB_2.get("sb"))

	#diff_r1_cb2_bb=int(CB_1.get("bb"))-int(room_1.get("bb_1"))
	#diff_r1_cb2_lmn=int(CB_1.get("lmn"))-int(room_1.get("lmn_1"))
	#diff_r1_cb2_sb=int(CB_1.get("sb"))-int(room_1.get("sb_1"))

	netdiff_r3=diff_r3_cb1_bb+diff_r3_cb1_lmn+diff_r3_cb1_sb+diff_r3_cb2_bb+diff_r3_cb2_lmn+diff_r3_cb2_sb
	#print("nett_diff_r3",netdiff_r3)

	#room4_berry_difference
	

	##check difference
	
	diff_r4_cb1_bb=int(CB_1.get("bb"))-int(room_4.get("bb_4"))
	diff_r4_cb1_lmn=int(CB_1.get("lmn"))-int(room_4.get("lmn_4"))
	diff_r4_cb1_sb=int(CB_1.get("sb"))-int(room_4.get("sb_4"))
	if(diff_r4_cb1_bb<0):
		diff_r4_cb2_bb=int(CB_2.get("bb"))-int(abs(diff_r4_cb1_bb))
	else:
		diff_r4_cb2_bb=int(CB_2.get("bb"))
	if(diff_r4_cb1_lmn<0):
		diff_r4_cb2_lmn=int(CB_2.get("lmn"))-int(abs(diff_r4_cb1_lmn))
	else:
		diff_r4_cb2_lmn=int(CB_2.get("lmn"))
	if(diff_r4_cb1_sb<0):
		diff_r4_cb2_sb=int(CB_2.get("sb"))-int(abs(diff_r4_cb1_sb))
	else:
		diff_r4_cb2_sb=int(CB_2.get("sb"))

	#diff_r1_cb2_bb=int(CB_1.get("bb"))-int(room_1.get("bb_1"))
	#diff_r1_cb2_lmn=int(CB_1.get("lmn"))-int(room_1.get("lmn_1"))
	#diff_r1_cb2_sb=int(CB_1.get("sb"))-int(room_1.get("sb_1"))

	netdiff_r4=diff_r4_cb1_bb+diff_r4_cb1_lmn+diff_r4_cb1_sb+diff_r4_cb2_bb+diff_r4_cb2_lmn+diff_r4_cb2_sb
	#print("nett_diff_r4",netdiff_r4)


	#print("error")

	if(netdiff_r1<=netdiff_r2 and netdiff_r1<=netdiff_r3 and netdiff_r1<=netdiff_r4):
		#print("goto room1")
		return 1

	elif(netdiff_r2<=netdiff_r1 and netdiff_r2<=netdiff_r3 and netdiff_r2<=netdiff_r4):
		#print("goto room2")
		return 2

	elif(netdiff_r3<=netdiff_r2 and netdiff_r3<=netdiff_r1 and netdiff_r3<=netdiff_r4):
		#print("goto room3")
		return 3

	elif(netdiff_r4<=netdiff_r2 and netdiff_r4<=netdiff_r3 and netdiff_r4<=netdiff_r1):
		#print("goto room4")
		return 4



	else:
		return -1

	##############################################################3

###### Updates CB_1 and CB_2 for max in local collector 1 #####
def CB_1_CB_2_max_update(CB_1,CB_2):

	"""
	Purpose:
	---
	This function takes the CB_1 and CB_2 dictionaries returned fron decode_json function 
	and returns updated CB_1 and CB_2 to drop the maximum fruits into local collector 1 as it is nearer
	to the fruits so saves time
	it returns a flag value to tell if the CB_1 and CB_2 have been updated or not
	
	Input Arguments:
	---

	`CB_1` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 1
	`CB_2` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 2
	
	Returns:
	---
	`CB_1` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 1
	`CB_2` 	:  [ dictionary ]
		a dictionary which has information of what fruits to drop in collector 2
	`flag` 	:  [ integer ]
		if -1 the no updation is done
		if 1 the CB_1 and CB_2 va;ues have been interchanged
	
	
	Example call:
	---
	CB_1,CB_2,flag=CB_1_CB_2_max_update(CB_1,CB_2)
	
	"""

	########################################################################

	temp=dict()
	flag=-1
	a1=int(CB_1.get("bb"))
	a2=int(CB_1.get("lmn"))
	a3=int(CB_1.get("sb"))
	b1=int(CB_2.get("bb"))
	b2=int(CB_2.get("lmn"))
	b3=int(CB_2.get("sb"))

	a=a1+a2+a3
	b=b1+b2+b3
	if(b>a):
		temp=CB_1
		CB_1=CB_2
		CB_2=temp
		flag=1

	return CB_1,CB_2,flag

	###########################################################################

##### Takes CB_1,CB_2,CB_1p,CB_2p and returns updates CB_1,CB_2 #####
def update_CB1_CB2(CB_1,CB_2,CB_1p,CB_2p):
	"""
	Purpose:
	---
	This function takes the CB_1 and CB_2 dictionaries and also CB_1p and CB_2p dictionaries and updated the CB_1 and CB_2
	CB_1 and CB_2 have information on how many total fruits to pluck
	CB_1p and CB_2p have information on how many fruits have been plucked after entering and exiting a room.
	So if some fruits have been removed or plucked from a room the the number of required berries shud be updated
	
	Input Arguments:
	---
 	`CB_1` 	:  [ dictionary ]
		a dictionary which has information of what fruits to pluck and also the fruit type 
	`CB_2` 	:  [ dictionary ]
		a dictionary which has information of what fruits to pluck and also the fruit type
	`CB_1p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 1 after exiting a room
	`CB_2p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 2 after exiting a room

	CB_1 , CB_2 are collection box 1 and 2 respectively
	CB_1p , CB_2p are collection box 1 and 2 plucked respectively
	
	Returns:
	---
	`CB_1` 	:  [ dictionary ]
		an updated dictionary which contains the number of 
		fruits left to be plucked and dropped in collection box 1
	`CB_2` 	:  [ dictionary ]
		an updated dictionary which contains the number of 
		fruits left to be plucked and dropped in collection box 2
	
	Example call:
	---
	CB_1,CB_2=update_CB1_CB2(CB_1,CB_2,CB_1p,CB_2p)
	
	"""
	###############################################################################

	a1=int(CB_1.get("bb"))-int(CB_1p.get("bb"))
	a2=int(CB_1.get("lmn"))-int(CB_1p.get("lmn"))
	a3=int(CB_1.get("sb"))-int(CB_1p.get("sb"))
	b1=int(CB_2.get("bb"))-int(CB_2p.get("bb"))
	b2=int(CB_2.get("lmn"))-int(CB_2p.get("lmn"))
	b3=int(CB_2.get("sb"))-int(CB_2p.get("sb"))


	CB_1["bb"]=a1
	CB_2["bb"]=b1

	CB_1["lmn"]=a2
	CB_2["lmn"]=b2

	CB_1["sb"]=a3
	CB_2["sb"]=b3

	return CB_1,CB_2

	###############################################################################

def update_room_fruit_count(room_1,room_2,room_3,room_4,CB_1p,CB_2p,k):

	"""
	Purpose:
	---
	This function takes the number of fruits in a room 
	and also a dictionary which has number of fruits plucked from room k 
	and updates the room fruit count accordinly and returs 
	updated room details i.e. room fruits count and types of fruits in that room.
	
	Input Arguments:
	---
 	`room_1` 	:  [ dictionary ]
		Checks room 1 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_2` 	:  [ dictionary ]
		Checks room 2 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_3` 	:  [ dictionary ]
		Checks room 3 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`room_4` 	:  [ dictionary ]
		Checks room 4 and stores the fruit and its positon so that count of a
		 particular fruit can be used for room entry priority 
	`CB_1p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 1 after exiting a room
	`CB_2p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 2 after exiting a room
	`k` 	:  [ Integer]
	 1 for room 1 , 2 for room 2 .........
	
	
	Returns:
	---
	`room_1` 	:  [ dictionary ]
		updated fruit details in room 1 
	`room_2` 	:  [ dictionary ]
		updated fruit details in room 2
	`room_3` 	:  [ dictionary ]
		updated fruit details in room 3 
	`room_4` 	:  [ dictionary ]
		updated fruit details in room 4  
	
	Example call:
	---
	room_1,room_2,room_3,room_4=update_room_fruit_count(room_1,room_2,room_3,room_4,CB_1p,CB_2p,k)
	
	"""
	################################################################################3

	a1=int(CB_1p.get("bb"))
	a2=int(CB_1p.get("lmn"))
	a3=int(CB_1p.get("sb"))

	b1=int(CB_2p.get("bb"))
	b2=int(CB_2p.get("lmn"))
	b3=int(CB_2p.get("sb"))



	
	if k==1:
		room_1["bb_1"]=int(room_1.get("bb_1"))-a1-b1
		room_1["lmn_1"]=int(room_1.get("lmn_1"))-a2-b2
		room_1["sb_1"]=int(room_1.get("sb_1"))-a3-b3


	if k==2:
		room_2["bb_2"]=int(room_2.get("bb_2"))-a1-b1
		room_2["lmn_2"]=int(room_2.get("lmn_2"))-a2-b2
		room_2["sb_2"]=int(room_2.get("sb_2"))-a3-b3	
		
	if k==3:
		room_3["bb_3"]=int(room_3.get("bb_3"))-a1-b1
		room_3["lmn_3"]=int(room_3.get("lmn_3"))-a2-b2
		room_3["sb_3"]=int(room_3.get("sb_3"))-a3-b3
	
	if k==4:
		room_4["bb_4"]=int(room_4.get("bb_4"))-a1-b1
		room_4["lmn_4"]=int(room_4.get("lmn_4"))-a2-b2
		room_4["sb_4"]=int(room_4.get("sb_4"))-a3-b3

	return room_1,room_2,room_3,room_4

	

	################################################################################3


##### takes CB_1 and CB_2 and plucks and returns CB_1p,CB_2p to tell howmany plucked   ###### 
def pluck_logic(client_id,CB_1,CB_2):
	"""
	Purpose:
	---
	This function takes CB_1 and CB_2 which has information on how many fruits and what
	fruits to be plucked and plucks those fruits accordingly.
	It then returns the information on howmany fruits have been plucked
	
	Input Arguments:
	---
 	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`CB_1` 	:  [ dictionary ]
		an updated dictionary which contains the number of 
		fruits left to be plucked and dropped in collection box 1
	`CB_2` 	:  [ dictionary ]
		an updated dictionary which contains the number of 
		fruits left to be plucked and dropped in collection box 2
	
	Returns:
	---
	`CB_1p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 1 after exiting a room
	`CB_2p` 	:  [ dictionary ]
		a dictionary which has information of what fruits have been plucked to be dropped in 
		collection box 2 after exiting a room
	
	Example call:
	---
	CB_1p,CB_2p = pluck_logic(client_id,CB_1,CB_2)
	
	"""
	############################################################################3
	CB_1p=dict()
	CB_2p=dict()

	berry_positions_dictionary=task_4.visionsensorcode(client_id)
	print("pluck logic ",berry_positions_dictionary)

	a1=int(CB_1.get("bb"))
	a2=int(CB_1.get("lmn"))
	a3=int(CB_1.get("sb"))
	b1=int(CB_2.get("bb"))
	b2=int(CB_2.get("lmn"))
	b3=int(CB_2.get("sb"))

	if a1>0:
		#a1=task_4.pluck_berry(client_id,'Blueberry',a1,berry_positions_dictionary,1)
		a1=task_4.getpos(client_id,'Blueberry',a1,1)
	if a2>0:
		#a2=task_4.pluck_berry(client_id,'Lemon',a2,berry_positions_dictionary,1)
		a2=task_4.getpos(client_id,'Lemon',a2,1)
	if a3>0:
		#a3=task_4.pluck_berry(client_id,'Strawberry',a3,berry_positions_dictionary,1)
		a3=task_4.getpos(client_id,'Strawberry',a3,1)

	if b1>0:
		#b1=task_4.pluck_berry(client_id,'Blueberry',b1,berry_positions_dictionary,2)
		b1=task_4.getpos(client_id,'Blueberry',b1,2)
	if b2>0:
		#b2=task_4.pluck_berry(client_id,'Lemon',b2,berry_positions_dictionary,2)
		b2=task_4.getpos(client_id,'Lemon',b2,2)
	if b3>0:
		#b3=task_4.pluck_berry(client_id,'Strawberry',b3,berry_positions_dictionary,2)
		b3=task_4.getpos(client_id,'Strawberry',b3,2)

	
	CB_1p["bb"]=a1
	CB_2p["bb"]=b1

	CB_1p["lmn"]=a2
	CB_2p["lmn"]=b2

	CB_1p["sb"]=a3
	CB_2p["sb"]=b3

	return CB_1p,CB_2p

	##############################################################

def theme_implementation_primary( client_id, rooms_entry):
	"""
	Purpose:
	---
	This is the theme implementation function which takes cliet id and also the room 
	entry list of tuples produced by the exe file.
	This function will be called in the main function.
	All the defined functions will be called here.
	
	Input Arguments:
	---
	`client_id`    :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()
	`rooms_entry`    :   [ list of tuples ]
		entry coordinates of all 4 rooms
	
	Returns:
	---
	no value
	
	Example call:
	---
	theme_implementation_primary(client_id, rooms_entry)
	
	"""
	
	new_rooms_entry=updation_room_entry(rooms_entry)
	CB_1p={"bb":0,"lmn":0,"sb":0}
	CB_2p={"bb":0,"lmn":0,"sb":0}
	CB_1,CB_2=decode_json()
	print("line 1,CB_1,CB_2= ",CB_1,CB_2)


	room_1=dict()
	room_2=dict()
	room_3=dict()
	room_4=dict()
	room_1["bb_1"]=2
	room_1["sb_1"]=2
	room_1["lmn_1"]=2
	room_4["bb_4"]=2
	room_4["sb_4"]=2
	room_4["lmn_4"]=2
	room_3["bb_3"]=2
	room_3["sb_3"]=2
	room_3["lmn_3"]=2
	room_2["bb_2"]=2
	room_2["sb_2"]=2
	room_2["lmn_2"]=2

	room_1,room_2,room_3,room_4=fruit_count(client_id)
	print(" line 2 room details= ",room_1,room_2,room_3,room_4)

	CB_1,CB_2,flag=CB_1_CB_2_max_update(CB_1,CB_2)
	print(" line 3 updated CB!CB@ and flag= ",CB_1,CB_2,flag)


	while ( (int(CB_1.get("bb")))>0 or (int(CB_1.get("lmn")))>0 or (int(CB_1.get("sb")))>0 or (int(CB_2.get("bb")))>0 or (int(CB_2.get("lmn")))>0 or (int(CB_2.get("sb")))>0 ):
		k=room_priority_logic(CB_1,CB_2,room_1,room_2,room_3,room_4)
		print("line4  room priority",k)
		enter_room(client_id,k,new_rooms_entry)
		CB_1p,CB_2p=pluck_logic(client_id,CB_1,CB_2)
		print("line 5 CB_1p,CB_2p",CB_1p,CB_2p)
		CB_1,CB_2=update_CB1_CB2(CB_1,CB_2,CB_1p,CB_2p)
		print("line 6 CB_1,CB_2 updated",CB_1,CB_2)
		room_1,room_2,room_3,room_4=update_room_fruit_count(room_1,room_2,room_3,room_4,CB_1p,CB_2p,k)
		print(" line 7 room details= ",room_1,room_2,room_3,room_4)
		exit_room(client_id,k,new_rooms_entry)

	#drop_fruit(client_id,flag)
	drop_fruit(client_id,1)

	'''

	#room_fruit_count=fruit_count(client_id)
	#print(room_fruit_count)
	print(rooms_entry)
	new_rooms_entry=updation_room_entry(rooms_entry)
	print(new_rooms_entry)

	res,v0=sim.simxGetObjectHandle(client_id,'Revolute_joint_left_cb1',sim.simx_opmode_oneshot_wait)
	rc=sim.simxSetJointTargetPosition(client_id,v0,0,sim.simx_opmode_streaming )
	#number returnCode=simxSetJointTargetPosition(number clientID,number jointHandle,number targetPosition,number simx_opmode_oneshot_wait)
	returnCode,position=sim.simxGetJointPosition(client_id,v0,sim.simx_opmode_streaming)
	print(position)

	
	while(position<90):
		rc=sim.simxSetJointPosition(client_id,v0,position,sim.simx_opmode_buffer )
		returnCode,position1=sim.simxGetJointPosition(client_id,v0,sim.simx_opmode_streaming)
		print(position1)
		print(" ")
		position=position+0.5
		print(position)
		time.sleep(0.1)

'''
	'''
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_1',sim.sim_scripttype_childscript,'setdummy',[],[-0.048,0.0074,0.0532],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	#rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_2',sim.sim_scripttype_childscript,'setdummy',[],[-0.051,0.196,-0.47],[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(5)	
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_1',sim.sim_scripttype_childscript,'setdummy',[],[-0.048,0.307,0.0532],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_1',sim.sim_scripttype_childscript,'setdummy',[],[-0.048,0.407,0.0532],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_1',sim.sim_scripttype_childscript,'setdummy',[],[-0.048,0.0,0.0532],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_1',sim.sim_scripttype_childscript,'setdummy',[],[-0.048,0.207,0.0532],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	#rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'local_collector_2',sim.sim_scripttype_childscript,'setdummy',[],[-0.051,-0.0039,-0.47],[],bytearray(),sim.simx_opmode_blocking)
	#time.sleep(5)
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket',sim.sim_scripttype_childscript,'setdummy',[],[-0.058,0.055,0.21],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)
	'''

	'''

	enter_room(client_id,1,new_rooms_entry)
	#plucking logic
	exit_room(client_id,1,new_rooms_entry)
	enter_room(client_id,2,new_rooms_entry)
	exit_room(client_id,2,new_rooms_entry)
	enter_room(client_id,3,new_rooms_entry)
	exit_room(client_id,3,new_rooms_entry)
	enter_room(client_id,4,new_rooms_entry)
	exit_room(client_id,4,new_rooms_entry)

	'''


	
	'''
	wheel_joints=task_3.init_setup(client_id)
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)

	#Traverse and check how many rooms have how many fruits by going to common points
	#From home to check and come back to home again
 ###################################################
	task_3.shortest_path(client_id,[(-3,0)])   # go to see room 1 and 2
	task_3.correction(client_id,130)
	berry_positions_dictionary_room1=task_4.visionsensorcode(client_id) #checks room 1
	time.sleep(2)

	task_3.rotation(client_id,-1,90)
	task_3.rotation(client_id,-1,90)
	task_3.correction(client_id,50)
	berry_positions_dictionary_room4=task_4.visionsensorcode(client_id) #checks room 4
	task_3.correction(client_id,90)

	task_3.shortest_path(client_id,[(-3,0)])
	task_3.shortest_path(client_id,[(-3,0)])
	berry_positions_dictionary_room3=task_4.visionsensorcode(client_id) #checks room 3

	task_3.rotation(client_id,-1,90)
	task_3.rotation(client_id,-1,90)
	task_3.correction(client_id,50)

	berry_positions_dictionary_room2=task_4.visionsensorcode(client_id) #checks room 2
	task_3.correction(client_id,90)
	task_3.shortest_path(client_id,[(-3,0)]) # home position


	print("Room 1 details", berry_positions_dictionary_room1)
	print("Room 2 details", berry_positions_dictionary_room2)
	print("Room 3 details", berry_positions_dictionary_room3)
	print("Room 4 details", berry_positions_dictionary_room4)

	print(error)
	'''
 ###################################################
'''
	if (rooms_entry[0]==(0,5) or rooms_entry[1]==(0,5) or rooms_entry[2]==(0,5) or rooms_entry[3]==(0,5)):

		task_3.shortest_path(client_id,[(-3,0)])
		task_3.shortest_path(client_id,[(-1,1)])
		task_3.shortest_path(client_id,[(0,2)])
		task_3.hault(client_id,wheel_joints)


		# pluck the fruits algorithm
		################################################
		task_4.call_open_close(client_id,"open")
		time.sleep(2)

		
		#lemon=berry_positions_dictionary.get('Lemon')
		#strawberry=berry_positions_dictionary.get('Strawberry')
		
		#task_4.pluck_berry(client_id,'Lemon',2,berry_positions_dictionary)
		#task_4.pluck_berry(client_id,'Strawberry',2,berry_positions_dictionary)


		#### for 2 Blue berry in lC2
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',2,berry_positions_dictionary,2)

		##### for 2 lemons in LC1

		task_3.shortest_path(client_id,[(1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		lemon=berry_positions_dictionary.get('Lemon')
		task_4.pluck_berry(client_id,'Lemon',2,berry_positions_dictionary,1)

		#### for 1 strawberry in LC1
		

		################################################

		#task_3.shortest_path(client_id,[(-1,0)])
		task_3.shortest_path(client_id,[(-1,-1)])
		task_3.shortest_path(client_id,[(0,-1)])
		task_3.shortest_path(client_id,[(1,-1)])
		task_3.shortest_path(client_id,[(3,0)])
		task_3.hault(client_id,wheel_joints)

	if (rooms_entry[0]==(2,5) or rooms_entry[1]==(2,5) or rooms_entry[2]==(2,5) or rooms_entry[3]==(2,5)):

		task_3.shortest_path(client_id,[(-3,0)])
		task_3.shortest_path(client_id,[(1,1)])
		task_3.shortest_path(client_id,[(0,1)])
		task_3.shortest_path(client_id,[(-1,1)])
		task_3.hault(client_id,wheel_joints)


		# pluck the fruits algorithm
		################################################
		task_4.call_open_close(client_id,"open")
		time.sleep(2)

		
		#lemon=berry_positions_dictionary.get('Lemon')
		#strawberry=berry_positions_dictionary.get('Strawberry')
		
		#task_4.pluck_berry(client_id,'Lemon',2,berry_positions_dictionary)
		#task_4.pluck_berry(client_id,'Strawberry',2,berry_positions_dictionary)


		#### for 1 strawberry in lC1
		
		##### for 2 lemons in LC1

		#task_3.shortest_path(client_id,[(-1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		lemon=berry_positions_dictionary.get('Lemon')
		task_4.pluck_berry(client_id,'Lemon',2,berry_positions_dictionary,1)

		#### for 2 blueberry in LC2
		task_3.shortest_path(client_id,[(-1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',2,berry_positions_dictionary,2)



		################################################

		task_3.shortest_path(client_id,[(1,0)])
		task_3.shortest_path(client_id,[(1,-1)])
		task_3.shortest_path(client_id,[(0,-2)])
		task_3.shortest_path(client_id,[(2,0)])
		#task_3.shortest_path(client_id,[(3,0)])
		task_3.hault(client_id,wheel_joints)
	
	if (rooms_entry[0]==(3,6) or rooms_entry[1]==(3,6) or rooms_entry[2]==(3,6) or rooms_entry[3]==(3,6))  :

		task_3.shortest_path(client_id,[(0,3)])
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id)
		task_3.shortest_path(client_id,[(-1,1)])
		task_3.shortest_path(client_id,[(0,1)])
		task_3.shortest_path(client_id,[(1,1)])
		task_3.hault(client_id,wheel_joints)
		
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		#task_3.shortest_path(client_id,[(1,0)])

		#task_3.hault(client_id,wheel_joints)
		


		# pluck the fruits algorithm
		################################################
		task_4.call_open_close(client_id,"open")
		time.sleep(2)


		##### for 2 lemons in LC1

	
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		lemon=berry_positions_dictionary.get('Lemon')
		task_4.pluck_berry(client_id,'Lemon',2,berry_positions_dictionary,1)


		#### for 1 strawberry in lC1
			

		#### for 2 blueberry in LC2
		task_3.shortest_path(client_id,[(-1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',2,berry_positions_dictionary,2)

		################################################

		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(1,1)])
		task_3.shortest_path(client_id,[(0,3)])
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,-2)])
		#task_3.shortest_path(client_id,[(-4,0)])
		task_3.hault(client_id,wheel_joints)

	if (rooms_entry[0]==(5,8) or rooms_entry[1]==(5,8) or rooms_entry[2]==(5,8) or rooms_entry[3]==(5,8)):

		task_3.shortest_path(client_id,[(0,3)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(-1,1)])
		task_3.shortest_path(client_id,[(0,2)])
		
		task_3.hault(client_id,wheel_joints)
		task_3.correction(client_id,90)


		# pluck the fruits algorithm
		################################################


		task_4.call_open_close(client_id,"open")
		time.sleep(2)

	

		
		#### for 1 Blue berry in lC2
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',1,berry_positions_dictionary,2)


		task_3.shortest_path(client_id,[(1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		strawberry=berry_positions_dictionary.get('Strawberry')
		task_4.pluck_berry(client_id,'Strawberry',1,berry_positions_dictionary,1)

	
		###############################################

		task_3.shortest_path(client_id,[(-1,-1)])
		task_3.shortest_path(client_id,[(0,-2)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,3)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)

	if (rooms_entry[0]==(5,6) or rooms_entry[1]==(5,6) or rooms_entry[2]==(5,6) or rooms_entry[3]==(5,6)):

		task_3.shortest_path(client_id,[(0,3)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(1,1)])
		task_3.shortest_path(client_id,[(0,1)])
		task_3.shortest_path(client_id,[(-1,1)])
		#task_3.shortest_path(client_id,[(-1,0)])
		task_3.hault(client_id,wheel_joints)
		task_3.correction(client_id,90)


		# pluck the fruits algorithm
		################################################


		task_4.call_open_close(client_id,"open")
		time.sleep(2)

		
		#task_3.shortest_path(client_id,[(1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		strawberry=berry_positions_dictionary.get('Strawberry')
		task_4.pluck_berry(client_id,'Strawberry',1,berry_positions_dictionary,1)

	
		task_3.shortest_path(client_id,[(-1,0)])
		
		#### for 1 Blue berry in lC2

		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',1,berry_positions_dictionary,2)

	
		###############################################

		task_3.shortest_path(client_id,[(1,0)])
		task_3.shortest_path(client_id,[(1,-1)])
		task_3.shortest_path(client_id,[(0,-2)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(0,3)])
		task_3.shortest_path(client_id,[(0,2)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)

	if (rooms_entry[0]==(6,5) or rooms_entry[1]==(6,5) or rooms_entry[2]==(6,5) or rooms_entry[3]==(6,5)):

		task_3.shortest_path(client_id,[(3,0)])
		task_3.shortest_path(client_id,[(-1,1)])
		task_3.shortest_path(client_id,[(0,1)])
		task_3.shortest_path(client_id,[(1,1)])
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
		#task_3.shortest_path(client_id,[(-1,0)])
		task_3.hault(client_id,wheel_joints)


	

		# pluck the fruits algorithm
		################################################
		task_4.call_open_close(client_id,"open")
		time.sleep(2)

		#task_3.shortest_path(client_id,[(1,0)])
		task_3.hault(client_id,wheel_joints)
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		strawberry=berry_positions_dictionary.get('Strawberry')
		task_4.pluck_berry(client_id,'Strawberry',1,berry_positions_dictionary,1)

		task_3.shortest_path(client_id,[(-1,0)])

		#### for 1 Blue berry in lC2
		berry_positions_dictionary=task_4.visionsensorcode(client_id)
		blueberry=berry_positions_dictionary.get('Blueberry')
		task_4.pluck_berry(client_id,'Blueberry',1,berry_positions_dictionary,2)



		################################################
		task_3.shortest_path(client_id,[(1,0)])
		task_3.rotation(client_id,-1,90)
		task_3.correction(client_id,90)
		task_3.shortest_path(client_id,[(-1,-1)])
		task_3.shortest_path(client_id,[(0,-2)])
		task_3.shortest_path(client_id,[(-2,0)])
		task_3.shortest_path(client_id,[(0,4)])
		task_3.shortest_path(client_id,[(0,3)])
		task_3.hault(client_id,wheel_joints)
		task_3.rotation(client_id,1,90)
		task_3.correction(client_id,90)
	




	

	task_3.shortest_path(client_id,[(0,2)])
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)



#drop in CB2 from local collector 1
	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket',sim.sim_scripttype_childscript,'setdummy',[],[-0.058,0.055,0.21],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)



	task_3.shortest_path(client_id,[(0,-2)])
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)

	
	task_3.shortest_path(client_id,[(0,-1)])
	task_3.hault(client_id,wheel_joints)
	task_3.shortest_path(client_id,[(0,-1)])
	task_3.hault(client_id,wheel_joints)
	task_3.correction(client_id,90)

#drop in CB1 from LC2

	rc,oi,of,os,ob= sim.simxCallScriptFunction(client_id,'basket_2',sim.sim_scripttype_childscript,'setdummy',[],[-0.036,0.054,-0.62],[],bytearray(),sim.simx_opmode_blocking)
	time.sleep(5)	
	
'''

	

if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [(3,6), (5,6), (8,3), (2,3)]     # example list of tuples

	###############################################################
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

		# Running student's logic
		theme_implementation_primary(client_id, rooms_entry)

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
		print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	except KeyboardInterrupt:
		print('\n[ERROR] Script interrupted by user!')