import sim
import time
import os,sys
import math
import traceback
import task_4
import random

def resource_path(relative_path):
	""" Get absolute path to resource, works for dev and for PyInstaller """
	try:
		# PyInstaller creates a temp folder and stores path in _MEIPASS
		base_path = sys._MEIPASS
	except Exception:
		base_path = os.environ.get("_MEIPASS2",os.path.abspath("."))

	return os.path.join(base_path, relative_path)


def create_room(client_id,room_num, room_values, room_entrance_num,arena_base):

	wall_names=['wall_type_1.ttm','wall_type_1.ttm','wall_type_2.ttm','wall_type_3.ttm','wall_type_2.ttm','wall_type_3.ttm']
	wall_handles=[]

	for wall_name in wall_names:
		return_code=1 # Intializing to a non-zero value
		while return_code!=0: # Polling till the model doesn't get added. It is imperative to do this when we are importing a lot of models. 
			return_code, wall_handle = sim.simxLoadModel(client_id, resource_path(wall_name), 0, sim.simx_opmode_blocking) #Load the new model			
		
		return_code=sim.simxSetObjectParent(client_id,wall_handle,arena_base,True,sim.simx_opmode_blocking)
		wall_handles.append(wall_handle)

	if room_num%2 == 1:
		sim.ghjvhjvj(client_id,wall_handles[0],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking) #For setting orientation
		sim.ghjvhjvj(client_id,wall_handles[2],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
		sim.ghjvhjvj(client_id,wall_handles[3],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
		for index,handle in enumerate(wall_handles):
			sim.pougadq(client_id,handle,-1,room_values[room_num-1][index],sim.simx_opmode_blocking) # For setting position

	else:
		sim.ghjvhjvj(client_id,wall_handles[1],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
		sim.ghjvhjvj(client_id,wall_handles[4],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
		sim.ghjvhjvj(client_id,wall_handles[5],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)		

		for index,handle in enumerate(wall_handles):
			sim.pougadq(client_id,handle,-1,room_values[room_num-1][index],sim.simx_opmode_blocking)

	return_code=1 # Intializing to a non-zero value	
	while return_code!=0: # Polling till the model doesn't get deleted. It is imperative to do this when we are importing a lot of models.
		return_code=sim.simxRemoveModel(client_id,wall_handles[room_entrance_num],sim.simx_opmode_blocking)

def init_rooms():

	return_code,qr_plane  = sim.simxGetObjectHandle(client_id, 'QR_Plane', sim.simx_opmode_blocking)

	if(return_code!=0):
		print('Unable to find QR_Plane object in the scene. Kindly download the scene file again.')
		#TODO: Call end_program() here
		sys.exit() #TODO: May not be required if end_program() is called.

	############## Delete any walls if loaded previously or already present in the scene. #######################

	children_of_QR_Plane=[]
	child_object_handle=0 #If child_object_handle=-1, there is no child at the given index
	index_child=0

	#To determine if there is at least 1 child of QR_Plane 
	return_code=1
	while return_code!=0:
		return_code,child_object_handle=sim.simxGetObjectChild(client_id,qr_plane,index_child,sim.simx_opmode_blocking)
	
	index_child=+1

	if(child_object_handle!=-1):
		children_of_QR_Plane.append(child_object_handle)
		while child_object_handle!=-1:
			return_code=1
			while return_code!=0:
				return_code,child_object_handle=sim.simxGetObjectChild(client_id,qr_plane,index_child,sim.simx_opmode_blocking)
			children_of_QR_Plane.append(child_object_handle)
			index_child=+1

	return_code=1 # Intializing to a non-zero value	
	for child_object_handle in children_of_QR_Plane: #Iterating through all the children of QR plane.
		return_code=1 # Intializing to a non-zero value	
		while return_code!=0: # Polling till the object(NOT MODEL) doesn't get deleted. It is imperative to do this when we are importing a lot of models.
			return_code=sim.simxRemoveObject(client_id,child_object_handle,sim.simx_opmode_blocking)

	###############################################################################################################

	#Intializing room variable. 
	room = []
	for i in range(4): #There are a total of 4 rooms.
		room.append([]) #Appending an empty list for each room.
		for j in range(6):
			room[i].append([0, 0, 0]) #Appending a list (x,y,z) coordinate for each wall. (Each room has 6 walls) 

	room[0][0] = [-2.45+2.087-0.045, 1.65+3.064-0.9-0.045, 0.052]
	room[0][1] = [-1.55+2.087-0.045-0.045, 2.45+3.064-0.9, 0.052]
	room[0][2] = [-0.75+2.087-0.045, 2.00+3.064-0.9+0.045, 0.052]
	room[0][3] = [-0.75+2.087-0.045, 1.15+3.064-0.9+0.045, 0.052]
	room[0][4] = [-1.2+2.087+0.045-0.045, 0.75+3.064-0.9, 0.052]
	room[0][5] = [-2.05+2.087+0.045-0.045, 0.75+3.064-0.9, 0.052]
	
	
	room[1][0] = [room[0][1][0]+3.29,room[0][1][1],room[0][1][2]]
	room[1][1] = [room[0][2][0]+3.29,room[0][0][1],room[0][0][2]]
	room[1][2] = [room[0][4][0]+3.29,room[0][4][1],room[0][4][2]]
	room[1][3] = [room[0][5][0]+3.29,room[0][5][1],room[0][5][2]]
	room[1][4] = [room[0][0][0]+3.29,room[0][2][1],room[0][2][2]]
	room[1][5] = [room[0][0][0]+3.29,room[0][3][1],room[0][3][2]]
	
	room[2][0] = [room[1][1][0],room[1][1][1]-3.2553,room[1][1][2]]
	room[2][1] = [room[1][0][0],room[1][2][1]-3.2553,room[1][2][2]]
	room[2][2] = [room[1][4][0],room[1][4][1]-3.2553,room[1][4][2]]
	room[2][3] = [room[1][5][0],room[1][5][1]-3.2553,room[1][5][2]]
	room[2][4] = [room[1][2][0],room[1][0][1]-3.2553,room[1][2][2]]
	room[2][5] = [room[1][3][0],room[1][0][1]-3.2553,room[1][3][2]]
	
	room[3][0] = [room[0][1][0],room[0][5][1]-3.2553,room[0][5][2]]
	room[3][1] = [room[0][0][0],room[0][0][1]-3.2553,room[0][0][2]]
	room[3][2] = [room[0][4][0],room[0][1][1]-3.2553,room[0][4][2]]
	room[3][3] = [room[0][5][0],room[0][1][1]-3.2553,room[0][5][2]]
	room[3][4] = [room[0][2][0],room[0][2][1]-3.2553,room[0][2][2]]
	room[3][5] = [room[0][3][0],room[0][3][1]-3.2553,room[0][3][2]]
	
	deleted_walls=[] #Store the deleted wall of room 1-4 in the same order.

	#Creating the required rooms
	rand_num=0
	possible_walls_to_remove=[3,4,5] #Based on the current scene, any of the 3rd, 4th or 5th index of `wall_handles` list needs to be removed for this room.
	wall_to_remove=random.choice(possible_walls_to_remove) #Randomly choose a wall from the above list
	deleted_walls.append(wall_to_remove) #Store the deleted wall in a list
	create_room(client_id,1, room, wall_to_remove,qr_plane) #Calling create_room functions with the required args
	
	possible_walls_to_remove=[3,4,5]
	wall_to_remove=random.choice(possible_walls_to_remove)
	deleted_walls.append(wall_to_remove)
	create_room(client_id,2, room, wall_to_remove,qr_plane)
	
	possible_walls_to_remove=[2,4,5]
	wall_to_remove=random.choice(possible_walls_to_remove)
	deleted_walls.append(wall_to_remove)
	create_room(client_id,3, room, wall_to_remove,qr_plane)
	
	possible_walls_to_remove=[2,4,5]
	wall_to_remove=random.choice(possible_walls_to_remove)
	deleted_walls.append(wall_to_remove)
	create_room(client_id,4, room, wall_to_remove,qr_plane)

	print(deleted_walls)

	qr_codes_for_room_entry=[] #List of tuples to be passed to student's function.

	# It is not possible to generalize the code for this. Hence we have to manually assign for each wall of each room.
	#Checking conditions for Room 1
	if(deleted_walls[0]==3):
		qr_codes_for_room_entry.append((3,6))
	elif(deleted_walls[0]==4):
		qr_codes_for_room_entry.append((2,5))
	elif(deleted_walls[0]==5):
		qr_codes_for_room_entry.append((0,5))

	#Checking conditions for Room 2
	if(deleted_walls[1]==3):
		qr_codes_for_room_entry.append((6,5))
	elif(deleted_walls[1]==4):
		qr_codes_for_room_entry.append((5,8))
	elif(deleted_walls[1]==5):
		qr_codes_for_room_entry.append((5,6))

	#Checking conditions for Room 3
	if(deleted_walls[2]==2):
		qr_codes_for_room_entry.append((5,2))
	elif(deleted_walls[2]==4):
		qr_codes_for_room_entry.append((8,3))
	elif(deleted_walls[2]==5):
		qr_codes_for_room_entry.append((6,3))

	#Checking conditions for Room 4
	if(deleted_walls[3]==2):
		qr_codes_for_room_entry.append((2,3))
	elif(deleted_walls[3]==4):
		qr_codes_for_room_entry.append((3,2))
	elif(deleted_walls[3]==5):
		qr_codes_for_room_entry.append((3,0))

	print(qr_codes_for_room_entry)

client_id = task_4.init_remote_api_server()

return_code = task_4.start_simulation(client_id)

try:
	init_rooms()
	time.sleep(15)

except:
	traceback.print_exc(file=sys.stdout)
	return_code = task_4.stop_simulation(client_id)
	task_4.exit_remote_api_server(client_id)

return_code = task_4.stop_simulation(client_id)
task_4.exit_remote_api_server(client_id)