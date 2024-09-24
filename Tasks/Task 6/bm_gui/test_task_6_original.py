'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*                                                         
*  This script is intended to check the output of Task 6 Original Configuration        
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			test_task_6.py
*  Created:				
*  Last Modified:		
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode
import platform
import csv
import string
import random
from datetime import datetime
from itertools import islice
import cryptocode
import uuid
import random
import gspread
import json

team_id = sys.argv[1]
folder_path = sys.argv[2]
res_path = sys.argv[3]
sys.path.append(folder_path)


global output_list_child, output_list_custom, rtf_python, init_real_time, end_real_time, end_simulation_time
eval_rtf_python = 0


# Global variable "client_id" for storing ID of starting the CoppeliaSim Remote connection
# NOTE: DO NOT change the value of this "client_id" variable here
client_id = -1


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

# Headers to be added in the 1st row of the Google Sheet.
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

# Credentials required to communicate with Google Sheets.
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



def send_data_e_yantra_server():

    submission_attempts=0

    flag_data_sent_successfully=0 # 0- Failed sending data, 1- Successfully sent data OR Master Flag in the sheet is 0 (Can be done manually ONLY). 

    while submission_attempts<5:
        print('\n#################################################', flush=True)
        print('\nAttempting to send data to e-Yantra Servers.', flush=True)
        print('\nMake sure you have an active internet connection.', flush=True)

        try:
            service_acc = gspread.service_account_from_dict(axebnfgh)
            g_sheet=service_acc.open('Real_Time_Performance_of_Teams_in_Task_6')
            worksheet_list = g_sheet.worksheets()
            master_wksheet = g_sheet.worksheet('master')		
            if(master_wksheet.acell('Z1').value=='1'):

                list_of_worksheets=[]

                for obj in worksheet_list:
                    list_of_worksheets.append(obj._properties['title'])


                if(str(team_id) not in list_of_worksheets):
                    wksheet = g_sheet.add_worksheet(title=str(team_id), rows="10000", cols="20")
                    wksheet.update('A1:W1',[headers]) #Add header row for the first time.
                else:
                    wksheet = g_sheet.worksheet(str(team_id))


                row_num_to_write=len(wksheet.get_all_values())+1 # This will give number of rows which are filled.

                data_to_push = [curr_date_time                               , platform_uname                                , str(hex(uuid.getnode())),
                                str(data_from_calculate_score[0])            , round(float(data_from_calculate_score[1]),2)  , round(float(data_from_calculate_score[2]),2),
                                round(float(data_from_calculate_score[3]),2) , round(float(data_from_calculate_score[4]),2)  , round(float(data_from_calculate_score[5]),2),
                                round(float(data_from_calculate_score[6]),2) , round(float(data_from_calculate_score[7]),2)  , str(data_from_calculate_score[8]),
                                str(data_from_calculate_score[9])            , str(data_from_calculate_score[10])            , str(data_from_calculate_score[11]), str(data_from_calculate_score[12]),
                                float(len(collisions_list_to_show))          , round(float(mass_of_arm),2)                   , round(float(end_simulation_time),2),
                                round(float(rtf_python),2)                   , float(no_of_joints)                           , round(float(torque),2)                    ,
                                round(float(force),2)
                                ]

                wksheet.update('A'+str(row_num_to_write)+':W'+str(row_num_to_write),[data_to_push])

                if(len(wksheet.get_all_values())+1==row_num_to_write+1): # Verifying if the row is filled or not.
                    print("\nSuccessfully sent data to e-Yantra Servers.", flush=True)
                    print('\n#################################################', flush=True)
                    submission_attempts=5
                    flag_data_sent_successfully=1

            else:
                submission_attempts=5
                flag_data_sent_successfully=0
                print('\nSubmission to server is not available now.', flush=True)
                print('\n#################################################', flush=True)
        
        except:
            submission_attempts+=1
            flag_data_sent_successfully=0
            # traceback.print_exc(file=sys.stdout)
            # print()
            print('\nFAILED sending the data.', flush=True)
            print('\nNumber of times remaining: ',5-submission_attempts, flush=True)
            # input('\nPress any key to try again.')

    return flag_data_sent_successfully

# Importing the sim module for Remote API connection with CoppeliaSim
try:
    sim = __import__('sim')
    
except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!', flush=True)
    print('\n[WARNING] Make sure to have following files in the directory:', flush=True)
    print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n', flush=True)
    sys.exit()


def init_remote_api_server():
    """
    Purpose:
    ---
    This function should first close any open connections and then start
    communication thread with server i.e. CoppeliaSim.

    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
    The evaluation script will handle that condition.

    Input Arguments:
    ---
    None

    Returns:
    ---
    `client_id`     :  [ integer ]
        the client_id generated from start connection remote API, it should be stored in a global variable

    Example call:
    ---
    client_id = init_remote_api_server()

    NOTE: This function will be automatically called by evaluation script before starting the simulation.
    """

    global client_id
    ##############  ADD YOUR CODE HERE  ##############

    sim.simxFinish(-1)  # just in case, close all opened connections
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to CoppeliaSim
    sim.simxGetPingTime(client_id)

    ##################################################

    return client_id


def start_simulation():
    """
    Purpose:
    ---
    This function should first start the simulation if the connection to server
    i.e. CoppeliaSim was successful and then wait for last command sent to arrive
    at CoppeliaSim server end.

    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
    The evaluation script will handle that condition.

    Input Arguments:
    ---
    None

    Returns:
    ---
    `return_code`   :  [ integer ]
        the return code generated from the start running simulation remote API

    Example call:
    ---
    return_code = start_simulation()

    NOTE: This function will be automatically called by evaluation at the start of simulation.
    """

    global client_id

    ##############  ADD YOUR CODE HERE  ##############

    # return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
    if client_id != -1:
        return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

    # Making sure that last command sent out had time to arrive
    sim.simxGetPingTime(client_id)

    ##################################################

    return return_code


def stop_simulation():

    """
    Purpose:
    ---
    This function should stop the running simulation in CoppeliaSim server.

    NOTE: In this Task, do not call the exit_remote_api_server function in case of failed connection to the server.
    The evaluation script will handle that condition.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    `return_code`   :  [ integer ]
        the return code generated from the stop running simulation remote API
    
    Example call:
    ---
    return_code = stop_simulation()
    
    """

    global client_id

    ##############  ADD YOUR CODE HERE  ##############
 
    return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait)
    sim.simxGetPingTime(client_id)

    ##################################################

    return return_code


def exit_remote_api_server():

    """
    Purpose:
    ---
    This function should wait for the last command sent to arrive at the Coppeliasim server
    before closing the connection and then end the communication thread with server
    i.e. CoppeliaSim using simxFinish Remote API.

    Input Arguments:
    ---
    None

    Returns:
    ---
    None

    Example call:
    ---
    exit_remote_api_server()

    """

    global client_id

    ##############  ADD YOUR CODE HERE  ##############

    sim.simxGetPingTime(client_id)
    sim.simxFinish(client_id)

    ##################################################


# This function will replace the BM_Bot in the scene
def replacement():
    global client_id

    # Getting object handles
    return_code, vs_1_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
    return_code, vs_2_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2', sim.simx_opmode_blocking)
    return_code, nameplate_handle = sim.simxGetObjectHandle(client_id, 'name_plate', sim.simx_opmode_blocking)
    return_code, force_sensor_br_handle = sim.simxGetObjectHandle(client_id, 'force_sensor_br', sim.simx_opmode_blocking)
    # return_code, camera_handle = sim.simxGetObjectHandle(client_id, 'DefaultCamera', sim.simx_opmode_blocking)
    return_code, BM_Bot_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot', sim.simx_opmode_blocking)
    return_code_basket, basket_handle = sim.simxGetObjectHandle(client_id, 'basket', sim.simx_opmode_blocking)
    return_code, camera_revolute_joint_handle = sim.simxGetObjectHandle(client_id, 'rj_for_camera', sim.simx_opmode_blocking)

    # Getting parent of VS 2 and basket
    returnCode, vs_2_parent_handle = sim.simxGetObjectParent( client_id, vs_2_handle, sim.simx_opmode_blocking)
    # returnCode, basket_parent_handle = sim.simxGetObjectParent( client_id, basket_handle, sim.simx_opmode_blocking)

    # If basket exists
    if return_code_basket == 0:
        # Checking whether there is a force_sensor_bbasket present or not
        return_code, force_sensor_bbasket_handle = sim.simxGetObjectHandle(client_id, 'force_sensor_bbasket', sim.simx_opmode_blocking)
        if return_code == 0:         # There exists the FS
            basket_tree_topmost_object_handle = force_sensor_bbasket_handle
        else:                        # FS doesn't exist
            basket_tree_topmost_object_handle = basket_handle


    # Before removing their BM_Bot model, setting its position to (2.132,2.132) and orientation
    # return_code = sim.simxSetObjectPosition( client_id, BM_Bot_handle, -1 , [ 0, 0, 4.8029e-02], sim.simx_opmode_blocking)
    # return_code = sim.simxSetObjectOrientation( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)
    return_code = sim.pougadq( client_id, BM_Bot_handle, -1 , [ 2.132, 2.132, 4.8029e-02], sim.simx_opmode_blocking)
    return_code = sim.nbhjhesq( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)

    # Making Vision sensors, name plate, fs_r parentless, basket tree parentless
    return_code = sim.simxSetObjectParent( client_id, vs_1_handle, -1, True, sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, vs_2_handle, -1, True, sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, nameplate_handle, -1, True, sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, force_sensor_br_handle, -1, True, sim.simx_opmode_blocking)
    if return_code_basket == 0:
        return_code = sim.simxSetObjectParent( client_id, basket_tree_topmost_object_handle, -1, True, sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, camera_revolute_joint_handle, -1, True, sim.simx_opmode_blocking)


    # Removing their BM_Bot
    return_code = sim.simxRemoveModel( client_id, BM_Bot_handle, sim.simx_opmode_blocking)

    # Adding our BM_Bot, by default at (2.132,2.132) which is what we required
    return_code, BM_Bot_handle = sim.simxLoadModel( client_id, resource_path('BM_Bot_general.ttm'), 0, sim.simx_opmode_blocking) #Load the new model

    # Setting its position to (2.132,2.132) and orientation
    # return_code = sim.simxSetObjectPosition( client_id, BM_Bot_handle, -1 , [ 0, 0, 4.8029e-02], sim.simx_opmode_blocking)
    # return_code = sim.simxSetObjectOrientation( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)
    return_code = sim.pougadq( client_id, BM_Bot_handle, -1 , [ 2.132, 2.132, 4.8029e-02], sim.simx_opmode_blocking)
    return_code = sim.nbhjhesq( client_id, BM_Bot_handle, -1 , [ -math.pi/2, 0, -math.pi/2], sim.simx_opmode_blocking)


    # Making BM_Bot_base as parent of vision_sensor_1 and name plate
    return_code, BM_Bot_base_handle = sim.simxGetObjectHandle(client_id, 'BM_Bot_base', sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, vs_1_handle, BM_Bot_base_handle, True, sim.simx_opmode_blocking)
    return_code = sim.simxSetObjectParent( client_id, nameplate_handle, BM_Bot_base_handle, True, sim.simx_opmode_blocking)

    # Making BM_Bot as parent of fs_br
    return_code = sim.simxSetObjectParent( client_id, force_sensor_br_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

    # # Making BM_Bot as parent of DefaultCamera
    # return_code = sim.simxSetObjectParent( client_id, camera_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

    # Making BM_Bot as parent of Basket tree
    if return_code_basket == 0:
        return_code = sim.simxSetObjectParent( client_id, basket_tree_topmost_object_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

    # Making BM_Bot as parent of camera revolute joint
    return_code = sim.simxSetObjectParent( client_id, camera_revolute_joint_handle, BM_Bot_handle, True, sim.simx_opmode_blocking)

    # Making VS 2 child of its original parent object
    return_code = sim.simxSetObjectParent( client_id, vs_2_handle, vs_2_parent_handle, True, sim.simx_opmode_blocking)


def wait_for_check_scene():
    global client_id

    return_code_signal = 1
    flag_scene_all_ok = '0'
    init_timeout = time.time()
    end_timeout = init_timeout
    timeout = end_timeout - init_timeout

    # inputBuffer = bytearray()
    # return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc',\
    # 								sim.sim_scripttype_customizationscript,'arm_check',[],[],[],inputBuffer,sim.simx_opmode_blocking)


    # We will wait till we get the signal value. If the correct flag_scene_all_ok signal value is not obtained,
    #  or the timer expires the code will NOT proceed further.
    while(return_code_signal != 0 or flag_scene_all_ok != '1'):
        return_code_signal,flag_scene_all_ok = sim.simxGetStringSignal(client_id, 'gfh36801nc', sim.simx_opmode_blocking)
        end_timeout = time.time()
        timeout = end_timeout - init_timeout
        flag_scene_all_ok = str(flag_scene_all_ok, 'utf-8')
        # print('flag_scene_all_ok',flag_scene_all_ok)
        # print('return_code_signal',return_code_signal)
        if(timeout > 10):
            print('\n[ERROR] Scene could not be checked. Check CoppeliaSim status bar.', flush=True)
            print('Exiting evaluation', flush=True)
            end_program()
            sys.exit()
            break


def wait_for_bot_reveal():
    global client_id

    return_code_signal = 1
    flag_scene_all_ok = '0'
    init_timeout = time.time()
    end_timeout = init_timeout
    timeout = end_timeout - init_timeout

    # inputBuffer = bytearray()
    # return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'Disc',\
    # 								sim.sim_scripttype_customizationscript,'arm_check',[],[],[],inputBuffer,sim.simx_opmode_blocking)


    # We will wait till we get the signal value. If the correct flag_scene_all_ok signal value is not obtained,
    #  or the timer expires the code will NOT proceed further.
    while(return_code_signal != 0 or flag_scene_all_ok != '1'):
        return_code_signal,flag_scene_all_ok = sim.simxGetStringSignal(client_id, 'start_student_run', sim.simx_opmode_blocking)
        end_timeout = time.time()
        timeout = end_timeout - init_timeout
        flag_scene_all_ok = str(flag_scene_all_ok, 'utf-8')
        # print('flag_scene_all_ok',flag_scene_all_ok)
        # print('return_code_signal',return_code_signal)
        if(timeout > 60):
            print('\n[ERROR] Robot could not be revealed.', flush=True)
            print('Exiting evaluation', flush=True)
            end_program()
            sys.exit()
            break


def load_eval_model():
    global client_id

    # Remove model if previously loaded.
    try:
        return_code, dummy_handle = sim.simxGetObjectHandle(client_id, 'eval_bm', sim.simx_opmode_blocking)
        if(return_code == 0): #This means that the object exists from before
            return_code = sim.simxRemoveModel( client_id, dummy_handle, sim.simx_opmode_blocking)
        
        #cwd = os.getcwd()
        #return_code,evaluation_screen_handle=sim.simxLoadModel(client_id,cwd+'//evaluation_projector_screen.ttm',0,sim.simx_opmode_blocking) #Load the new model
        return_code, dummy_handle = sim.simxLoadModel( client_id, resource_path('task_6_dummy.ttm'), 0, sim.simx_opmode_blocking) #Load the new model
        if(return_code != 0):
            # print('[ERROR] Evaluation script failed to load. Please try again.')
            print('\n[ERROR] Evaluation failed to start. Please try again.', flush=True)
            end_program()
            sys.exit()
        else:
            # print('Evaluation script loaded successfully.')
            print('\nEvaluation started successfully.', flush=True)
    except Exception:
        end_program()


# End the program
# Will stop the simulation, call organize_screen_end, clear string signals, exit the server
def end_program():


    return_code = stop_simulation()

    return_code, disc_handle = sim.simxGetObjectHandle(client_id, 'eval_bm', sim.simx_opmode_blocking)

    if (return_code == 0):				# This means that the object exists from before
        inputBuffer = bytearray()
        return_code, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
                                    sim.sim_scripttype_customizationscript,'organize_screen_end',[],[],[],inputBuffer,sim.simx_opmode_blocking)


    # This is used to clear all the signals.
    return_code = sim.simxClearStringSignal(client_id, '', sim.simx_opmode_oneshot)

    try:
        exit_remote_api_server()

        if (start_simulation() == sim.simx_return_initialize_error_flag):
            print('\nDisconnected successfully from Remote API Server in CoppeliaSim!', flush=True)

        else:
            print('\n[ERROR] Failed disconnecting from Remote API server!', flush=True)
            # print('[ERROR] exit_remote_api_server function in task_2a.py is not configured correctly, check the code!')

    except Exception:
        # print('\n[ERROR] Your exit_remote_api_server function in task_2a.py throwed an Exception. Kindly debug your code!')
        print('\n[ERROR] The connection to Remote API Server did not end successfully!', flush=True)
        print('Stop the CoppeliaSim simulation manually if required.\n', flush=True)
        # traceback.print_exc(file=sys.stdout)
        print(flush=True)


def get_child_data():
    global client_id

    inputBuffer = bytearray()
    return_code, retInts, retFloats, data_child, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
                                    sim.sim_scripttype_childscript,'get_required_data_child',[],[],[],inputBuffer,sim.simx_opmode_blocking)
    # print("data_child:")
    # print(data_child)

    return data_child


def get_custom_data():
    global client_id

    inputBuffer = bytearray()
    return_code, retInts, retFloats, data_custom, retBuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',\
                                    sim.sim_scripttype_customizationscript,'get_required_data_custom',[],[],[],inputBuffer,sim.simx_opmode_blocking)

    # print("data_custom:")
    # print(data_custom)

    return data_custom,return_code


def general_berry_name(argument):

    if (argument.find('lemon')==0):
        return "Lemon"
    elif(argument.find('strawberry')==0):
        return "Strawberry"
    elif(argument.find('blueberry')==0):
        return "Blueberry"
    else:
        return "None"

def unique(list1):
 
    # initialize a null list
    unique_list = []
     
    # traverse for all elements
    for x in list1:
        # check if exists in unique_list or not
        if x not in unique_list:
            unique_list.append(x)
    
    return unique_list	


def create_room(room_num, room_values, room_entrance_num,arena_base):
    global client_id

    # print("create_room() started")
    wall_names=['wall_type_1.ttm','wall_type_1.ttm','wall_type_2.ttm','wall_type_3.ttm','wall_type_2.ttm','wall_type_3.ttm']
    wall_handles=[]

    for wall_name in wall_names:
        return_code=1 # Intializing to a non-zero value
        while return_code!=0: # Polling till the model doesn't get added. It is imperative to do this when we are importing a lot of models. 
            return_code, wall_handle = sim.simxLoadModel(client_id, resource_path(wall_name), 0, sim.simx_opmode_blocking) #Load the new model			
            # print("Loop 5")
        
        return_code=sim.simxSetObjectParent(client_id,wall_handle,arena_base,True,sim.simx_opmode_blocking)
        wall_handles.append(wall_handle)
        # print("Loop 6")

    if room_num%2 == 1:
        sim.nbhjhesq(client_id,wall_handles[0],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking) #For setting orientation
        sim.nbhjhesq(client_id,wall_handles[2],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
        sim.nbhjhesq(client_id,wall_handles[3],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
        for index,handle in enumerate(wall_handles):
            sim.pougadq(client_id,handle,-1,room_values[room_num-1][index],sim.simx_opmode_blocking) # For setting position

    else:
        sim.nbhjhesq(client_id,wall_handles[1],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
        sim.nbhjhesq(client_id,wall_handles[4],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)
        sim.nbhjhesq(client_id,wall_handles[5],-1, [0, 0, math.pi*0.5],sim.simx_opmode_blocking)		

        for index,handle in enumerate(wall_handles):
            sim.pougadq(client_id,handle,-1,room_values[room_num-1][index],sim.simx_opmode_blocking)

    return_code=1 # Intializing to a non-zero value	
    while return_code!=0: # Polling till the model doesn't get deleted. It is imperative to do this when we are importing a lot of models.
        return_code=sim.simxRemoveModel(client_id,wall_handles[room_entrance_num],sim.simx_opmode_blocking)
                                                                                                                        

def init_rooms():
    global client_id

    return_code,qr_plane  = sim.simxGetObjectHandle(client_id, 'QR_Plane', sim.simx_opmode_blocking)

    if(return_code!=0):
        print('Unable to find QR_Plane object in the scene. Kindly download the scene file again.', flush=True)
        #TODO: Call end_program() here
        end_program()
        sys.exit() #TODO: May not be required if end_program() is called.

    ############## Delete any walls if loaded previously or already present in the scene. #######################

    children_of_QR_Plane=[]
    child_object_handle=0 #If child_object_handle=-1, there is no child at the given index
    index_child=0

    #To determine if there is at least 1 child of QR_Plane 
    return_code=1
    while return_code!=0:
        return_code,child_object_handle=sim.simxGetObjectChild(client_id,qr_plane,index_child,sim.simx_opmode_blocking)
        # print("Loop 1")
    
    index_child=+1

    if(child_object_handle!=-1):
        children_of_QR_Plane.append(child_object_handle)
        while child_object_handle!=-1:
            return_code=1
            while return_code!=0:
                return_code,child_object_handle=sim.simxGetObjectChild(client_id,qr_plane,index_child,sim.simx_opmode_blocking)
                # print("Loop 2")
            children_of_QR_Plane.append(child_object_handle)
            index_child += 1
            # print("Loop 3 ", child_object_handle, index_child)

    # print(children_of_QR_Plane)
    if len(children_of_QR_Plane) > 0:
        children_of_QR_Plane.pop()         # Removing last element bcoz it is equal to -1
    return_code=1 # Intializing to a non-zero value	
    for child_object_handle in children_of_QR_Plane: #Iterating through all the children of QR plane.
        return_code=1
        while return_code!=0: # Polling till the object(NOT MODEL) doesn't get deleted. It is imperative to do this when we are importing a lot of models.
            return_code=sim.simxRemoveObject(client_id,child_object_handle,sim.simx_opmode_blocking)
            # print("Loop 4", return_code, child_object_handle)

    ###############################################################################################

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
    create_room(1, room, wall_to_remove,qr_plane) #Calling create_room functions with the required args
    
    possible_walls_to_remove=[3,4,5]
    wall_to_remove=random.choice(possible_walls_to_remove)
    deleted_walls.append(wall_to_remove)
    create_room(2, room, wall_to_remove,qr_plane)
    
    possible_walls_to_remove=[2,4,5]
    wall_to_remove=random.choice(possible_walls_to_remove)
    deleted_walls.append(wall_to_remove)
    create_room(3, room, wall_to_remove,qr_plane)
    
    possible_walls_to_remove=[2,4,5]
    wall_to_remove=random.choice(possible_walls_to_remove)
    deleted_walls.append(wall_to_remove)
    create_room(4, room, wall_to_remove,qr_plane)

    # print(deleted_walls)

    # Mapping wall generation to rooms entry
    
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

    # print(qr_codes_for_room_entry)

    return qr_codes_for_room_entry


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
    required_configuration = {
                                "Blueberry": "1_CB1", 
                                "Lemon": "5_CB2", 
                                "Strawberry": "1_CB1"
                            }

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

    print('\n#################################################', flush=True)
    print("\n********** TASK 6 RESULT **********", flush=True)

    if valid_run_flag:
        print("\nThe current run is a VALID run.", flush=True)

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
        print("Task 6 Score           :", score, flush=True)
        

    else:
        print("\nThe current run is an INVALID run.", flush=True)
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
        print(flush=True)
        print("Task 6 Score           :", score, flush=True)


    data_to_return = [ valid_run_flag, T, CI, CP, CD, P ,B ,score, final_ci_list, final_cp_list, final_cd_list, final_cb1_drops, final_cb2_drops ]
        
    return data_to_return



def task_6_cardinal_main():

    global output_list_child, output_list_custom, rtf_python, eval_rtf_python, init_real_time, end_real_time, end_simulation_time

    try:
        theme_implemenation = __import__('theme_implementation')

    except ImportError:
        print('\n[ERROR] theme_implementation.py file is not present in the current directory.', flush=True)
        print('Your current directory is: ', folder_path, flush=True)
        print('Make sure theme_implementation.py is present in this current directory.\n', flush=True)
        sys.exit()
        
    except Exception as e:
        print('Your theme_implementation.py threw an Exception, kindly debug your code!\n', flush=True)
        traceback.print_exc(file=sys.stdout)
        sys.exit()


    # Initiate the Remote API connection with CoppeliaSim server
    print('\nConnection to CoppeliaSim Remote API Server initiated.', flush=True)
    print('Trying to connect to Remote API Server...', flush=True)

    try:
        client_id = init_remote_api_server()
        if (client_id != -1):
            print('\nConnected successfully to Remote API Server in CoppeliaSim!', flush=True)

            #Send folder path to coppeliasim
            return_code_signal = sim.simxSetStringSignal(client_id, 'get_folder_name_child', res_path, sim.simx_opmode_oneshot)
            return_code_signal = sim.simxSetStringSignal(client_id, 'get_folder_name_customisation', res_path, sim.simx_opmode_oneshot)

            # Students should have opened Theme_Arena.ttt

            # rooms_entry = [(0,5), (5,6), (5,0), (2,3)] 
            try:
                # Generating walls and getting rooms entry co-ordinate
                # time.sleep(3.)
                rooms_entry = init_rooms()
                print("\nWalls generated successfully.", flush=True)
                print("Room entry QR code co-ordinates (R1-R2-R3-R4): ", rooms_entry, flush=True)

            except Exception:
                print('\n[ERROR] Could not generate walls.', flush=True)
                # traceback.print_exc(file=sys.stdout)
                print(flush=True)
                end_program()
                sys.exit()

            # Loading our task 6 dummy model
            load_eval_model()

            # Check scene
            wait_for_check_scene()

            # Replacing BM_Bot
            replacement()


            init_real_time = time.time()

            #  Start simulation
            return_code = start_simulation()

            if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
                print('\nSimulation started correctly in CoppeliaSim.', flush=True)

            else:
                print('\n[ERROR] Failed starting the simulation in CoppeliaSim!', flush=True)
                end_program()
                sys.exit()

            # Wait for bot to reveal
            wait_for_bot_reveal()
            

            try:
                # Running student's logic for four random co-ordinates
                theme_implemenation.theme_implementation_primary(client_id, rooms_entry)

            except KeyboardInterrupt:
                print('\n[ERROR] Test script for Task 6 interrupted by user!', flush=True)
                #end_program()			

            except Exception:
                print('\n[ERROR] Your theme_implementation_primary(client_id, rooms_entry) function threw an error.', flush=True)
                print('Kindly debug your code.', flush=True)
                traceback.print_exc(file=sys.stdout)
                print(flush=True)
                end_program()
                sys.exit()

            # Getting data from child script
            output_list_child = get_child_data()
            p = ",".join(output_list_child)
            output_list_child = p.split(',')
            # print('================================')
            # print(output_list_child)
            # print('================================')

            #  Stop simulation
            return_code = stop_simulation()
            end_real_time = time.time()

            # # RTF calculation
            # # NOTE: This method is not used. Instead end sim time is received from child data. The calculation is done in main function.
            # end_simulation_time = 0; rtf_python = 0
            

            # return_code, end_simulation_time = sim.simxGetStringSignal(client_id, 'time', sim.simx_opmode_blocking)

            # rtf_python = float("{0:.5f}".format(float(end_simulation_time)/(end_real_time - init_real_time)))
            # print('\nCalculated Real-Time Factor (rtf) = ', rtf_python)
            # if rtf_python >= 0.8:
            #     eval_rtf_python = 1
            # else:
            #     eval_rtf_python = 0


            # Getting data from custom script
            output_list_custom,return_code = get_custom_data()
            
            if(return_code!=0):
                sys.exit()

            p = ",".join(output_list_custom)
            output_list_custom = p.split(',')
            # print('================================')
            # print(output_list_custom)
            # print('================================')
            
            end_program()

       
        else:
            print('\n[ERROR] Failed connecting to Remote API server!', flush=True)
            print('[WARNING] Make sure the CoppeliaSim software is running and', flush=True)
            print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.', flush=True)
            # print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
            print(flush=True)
            # input('Press enter to exit')
            # sys.exit()

    except KeyboardInterrupt:
            print('\n[ERROR] Test script for Task 6 interrupted by user!', flush=True)
            end_program()
            # sys.exit()

    except Exception:
        print('\nUh oh! An unknown ERROR occured.', flush=True)
        print('Stop the CoppeliaSim simulation manually if started.\n', flush=True)
        traceback.print_exc(file=sys.stdout)
        print(flush=True)
        end_program()
        sys.exit()


# Main function for testing Task 6
try:
    if __name__ == '__main__':

        # print("*******************************************************************")
        # print("*                                                                 *")
        # print("*        =================================================        *")
        # print("*             Berryminator (BM) Theme (eYRC 2021-22)              *")
        # print("*        =================================================        *")
        # print("*                                                                 *")
        # print("*    This test suite is intended to check the output of Task 6    *")
        # print("*            of Berryminator (BM) Theme (eYRC 2021-22)            *")
        # print("*                ( Original Configuration )                       *")
        # print("*******************************************************************")

        # try:
        # 	team_id = int(input('\nEnter your Team ID (for e.g.: "1234" or "321"): '))

        # except ValueError:
        # 	print("\n[ERROR] Enter your Team ID which is an integer!\n")
        # 	sys.exit()

        platform_uname = platform.uname().system

        # conda_env_name = os.environ['CONDA_DEFAULT_ENV']

        # expected_conda_env_name = 'BM_' + str(team_id)

        # if conda_env_name == expected_conda_env_name:
            
        # 	conda_env_name_flag = 1

        # else:
            
        # 	conda_env_name_flag = 0
        # 	print("\n[WARNING] Conda environment name is not found as expected: BM_%s. Run this file with correct conda environment.\n" %(str(team_id)))
        # 	sys.exit()

        conda_env_name_flag = 1


        # TODO: team_id to be taken from GUI as we are not accepting any input from command line
        # TODO: TRY and EXCEPT BLOCK 
        # print("\nTODO: team_id to be taken from GUI as we are not accepting any input from command line")
        # team_id = int(input('\nEnter your Team ID (for e.g.: "1234" or "321"): '))

        if conda_env_name_flag == 1:
            
            curr_date_time      = str(datetime.now())
            encrypted_team_id   = cryptocode.encrypt(str(team_id), "s}ZYmS{:QgMx'9Qd")
            encrypted_date_time = cryptocode.encrypt(curr_date_time, "s}ZYmS{:QgMx'9Qd")
            encrypted_platform  = cryptocode.encrypt(platform_uname, "s}ZYmS{:QgMx'9Qd")
            encrypted_mac       = cryptocode.encrypt(str(hex(uuid.getnode())), "s}ZYmS{:QgMx'9Qd")


            print("Keep CoppeliaSim opened with your Theme_Arena.ttt", flush=True)

            # input("\nPress ENTER to start Task 6 evaluation  ")
            print(flush=True)

            task_6_cardinal_main()

            index = 0
            if output_list_child[index] != "CI":
                print("The data from the simulation got corrupted.", flush=True)
                print("Kindly rerun the exe", flush=True)
                print("theme_implementation_result.txt NOT generated", flush=True)
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


            # Calculate Score as per the Rulebook
            total_no_of_collisions = len(collisions_list_to_show)
            effective_time         = float(end_simulation_time)  - float(time_to_substract)
            data_from_calculate_score = calculate_score( effective_time, ci_list_to_show, pluck_list_to_show, dropped_in_cb1, dropped_in_cb2, total_no_of_collisions)
            # Order od return data = [ valid_run_flag, T, CI, CP, CD, P ,B ,score, final_ci_list, final_cp_list, final_cd_list, final_cb1_drops, final_cb2_drops ]


            # RTF calculation
            rtf_python = 0
            rtf_python = float("{0:.5f}".format(float(end_simulation_time)/(end_real_time - init_real_time)))
            print('\nCalculated Real-Time Factor (rtf) = ', rtf_python, flush=True)
            if rtf_python >= 0.8:
                eval_rtf_python = 1
            else:
                eval_rtf_python = 0

            # Encrypting data
            # # Password = "s}ZYmS{:QgMx'9Qd"
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
            encrypted_init_real_time       = cryptocode.encrypt( str(init_real_time), "s}ZYmS{:QgMx'9Qd")
            encrypted_end_real_time        = cryptocode.encrypt( str(end_real_time), "s}ZYmS{:QgMx'9Qd")
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
           
            
            #Used to send data to e-Yantra Server (A google sheet)
            return_value = send_data_e_yantra_server()
            
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

                print("\n-----------------------------------------------------------", flush=True)
                print("\n Way to go "+str(team_id)+"! 'theme_implementation_result.txt' generated successfully.", flush=True)
                print("\n-----------------------------------------------------------", flush=True)
            else:
                print('\nFAILED generating theme_implementation_result.txt. Please follow Task 6 Instructions.', flush=True)

except KeyboardInterrupt:
    print('\n[ERROR] Test script for Task 6 interrupted by user!', flush=True)
    end_program()


except Exception:
    print('\n[ERROR] An Exception occurred', flush=True)
    print('Stop the CoppeliaSim simulation manually if started.\n', flush=True)
    # traceback.print_exc(file=sys.stdout)
    print(flush=True)
    end_program()
    sys.exit()