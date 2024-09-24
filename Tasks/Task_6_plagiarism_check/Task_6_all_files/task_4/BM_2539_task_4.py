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

# Team ID:			BM_2539
# Author List:		Swastik Pal
# Filename:			task_4.py
# Functions:		get_arm_joint_handles, arm_stop, move_arm, rotate_arm
# Global variables:	
# 					ping_time


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode

from task_2a import get_vision_sensor_image, get_vision_sensor_depth_image, transform_vision_sensor_depth_image, \
    detect_berries, detect_berry_positions, get_labeled_image
from task_3 import start_simulation, stop_simulation, init_remote_api_server, exit_remote_api_server, \
    transform_vision_sensor_image, task_3_primary, view_image, rotate, translate, strafe, init_setup, encoders

##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
    import sim

except Exception:
    print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
    print('\n[WARNING] Make sure to have following files in the directory:')
    print(
        'sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
    sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

def robo_see(client_id):
    _, vision_sensor_path_finder_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1',
                                                                  sim.simx_opmode_blocking)
    _, vision_sensor_berry_finder_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2',
                                                                   sim.simx_opmode_blocking)

    # Get image array and depth buffer from vision sensor in CoppeliaSim scene
    try:
        vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id,
                                                                                     vision_sensor_berry_finder_handle)
        vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id,
                                                                                                         vision_sensor_berry_finder_handle)

        if ((return_code == sim.simx_return_ok) and (return_code_2 == sim.simx_return_ok) and (
                len(image_resolution) == 2) and (len(depth_image_resolution) == 2) and (
                len(vision_sensor_image) > 0) and (len(vision_sensor_depth_image) > 0)):
            print('\nImage captured from Vision Sensor in CoppeliaSim successfully!')

            # Get the transformed vision sensor image captured in correct format
            try:
                transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
                transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image,
                                                                              depth_image_resolution)

                if (type(transformed_image) is np.ndarray) and (type(transformed_depth_image) is np.ndarray):

                    berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
                    print("Berries Dictionary = ", berries_dictionary)
                    berry_positions_dictionary = detect_berry_positions(berries_dictionary)
                    print("Berry Positions Dictionary = ", berry_positions_dictionary)

                    labelled_image = get_labeled_image(transformed_image, berries_dictionary,
                                                       berry_positions_dictionary)

                    view_image(transformed_image, 3000, 'transformed image')
                    view_image(transformed_depth_image, 3000, 'transformed depth image')
                    view_image(labelled_image, 3000, 'labelled image')

                else:
                    print(
                        '\n[ERROR] transform_vision_sensor_image function is not configured correctly, check the code.')
                    print('Stop the CoppeliaSim simulation manually.')
                    print()
                    return
            except Exception:
                print(
                    '\n[ERROR] Your transform_vision_sensor_image function threw an Exception, kindly debug your code!')
                print('Stop the CoppeliaSim simulation manually.\n')
                traceback.print_exc(file=sys.stdout)
                print()
                return
        else:
            print('\n[ERROR] get_vision_sensor function is not configured correctly, check the code.')
            print('Stop the CoppeliaSim simulation manually.')
            print()
            return
    except Exception:
        print('\n[ERROR] Your get_vision_sensor_image function threw an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        return


def sort_and_filter_dictionary(berry_positions_dictionary):
    """
    Sorts and filters the dictionary to prevent arm collisions and attempts at plucking berries in the basket

    :param berry_positions_dictionary: the dictionary of berry positions
    """
    sorted_dict = {}
    # Sorting
    for key, value in berry_positions_dictionary.items():
        sorted_dict[key] = sorted(value, key=lambda x: x[2])
    # print(sorted_dict) #Debugging
    # Filtering
    for key, value in sorted_dict.items():
        sorted_dict[key] = list(filter(lambda x: x[2] > 0.5, value))
    # print(sorted_dict) #debugging
    return sorted_dict


def find_berries(client_id):
    """
    Locates berries in the image.
    :param client_id: the client id of the communication thread
    """
    _, vision_sensor_berry_finder_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_2',
                                                                   sim.simx_opmode_blocking)

    vision_sensor_image, image_resolution, _ = get_vision_sensor_image(client_id,
                                                                       vision_sensor_berry_finder_handle)
    vision_sensor_depth_image, depth_image_resolution, _ = get_vision_sensor_depth_image(client_id,
                                                                                         vision_sensor_berry_finder_handle)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image,
                                                                  depth_image_resolution)
    berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
    # print("Berries Dictionary = ", berries_dictionary)  # debugging only
    berry_positions_dictionary = detect_berry_positions(berries_dictionary)
    # print("Berry Positions Dictionary = ", berry_positions_dictionary)  # debugging only

    # view_image(get_labeled_image(transformed_image, berries_dictionary, berry_positions_dictionary), wait=5000)  # debugging only

    return berries_dictionary, berry_positions_dictionary


def arm_stop(client_id: int, arm_joint_handles: tuple or list):
    """
    Stops all joints in the robotic arm.
    :param arm_joint_handles: the list of arm joints
    :param client_id: the client id of the communication thread
    """
    if sim.simxPauseCommunication(clientID=client_id, enable=True) != 0:
        print("Could not pause.")

    for handle in arm_joint_handles:
        sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=handle, targetVelocity=0,
                                       operationMode=sim.simx_opmode_oneshot)

    if sim.simxPauseCommunication(clientID=client_id, enable=False) != 0:
        print("Could not resume.")


def move_arm(client_id: int, joint_handle: int, target_pos: float):
    """
    This is a PID controller that moves joints to the desired position.

    :param client_id: the client id of the communication thread
    :param joint_handle: the handle of the joint to operate
    :param target_pos: the angle (in degrees)
    """

    target_pos = math.radians(target_pos)
    _, current_pos = sim.simxGetJointPosition(clientID=client_id, jointHandle=joint_handle,
                                              operationMode=sim.simx_opmode_blocking)
    kp = 1
    ki = 0.5
    kd = 0.1
    error = target_pos - current_pos
    error_old = error
    error_diff = 0
    accumulation = 0
    del_time = 1
    derivative = 0
    speed = kp * error
    clock_start = time.monotonic()
    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=speed,
                                   operationMode=sim.simx_opmode_blocking)
    for i in range(20):
        _, current_pos = sim.simxGetJointPosition(clientID=client_id, jointHandle=joint_handle,
                                                  operationMode=sim.simx_opmode_blocking)
        error = target_pos - current_pos
        speed = kp * error + ki * accumulation + kd * derivative
        clock_end = clock_start
        clock_start = time.monotonic()
        sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=speed,
                                       operationMode=sim.simx_opmode_blocking)
        error_diff = error - error_old
        error_old = error
        # speed_old = speed
        del_time = clock_end - clock_start
        accumulation += error_diff * del_time
        derivative = error_diff / del_time
        if math.isclose(error, 0, abs_tol=0.01):
            sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=0,
                                           operationMode=sim.simx_opmode_blocking)
            break

    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=0,
                                   operationMode=sim.simx_opmode_blocking)
    # print(math.degrees(current_pos), speed, error)  # debugging only


def rotate_arm(client_id: int, joint_handle: int, target_pos: float, speed: float = 0.4):
    """
    This is a method that moves joints to the desired position. No error correction.

    :param client_id: the client id of the communication thread
    :param joint_handle: the handle of the joint to operate
    :param target_pos: the angle (in degrees)
    :param speed: the speed of rotation
    """

    target_pos = math.radians(target_pos)
    _, current_pos = sim.simxGetJointPosition(clientID=client_id, jointHandle=joint_handle,
                                              operationMode=sim.simx_opmode_blocking)

    speed = math.copysign(speed, target_pos - current_pos)

    time_to_wait = (target_pos - current_pos) / speed - ping_time / 2
    if time_to_wait < 0:
        time_to_wait = 0

    # if sim.simxPauseCommunication(clientID=client_id, enable=True) != 0:
    #     print("Could not pause.")

    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=speed,
                                   operationMode=sim.simx_opmode_oneshot)

    time.sleep(time_to_wait)

    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint_handle, targetVelocity=0,
                                   operationMode=sim.simx_opmode_blocking)

    # if sim.simxPauseCommunication(clientID=client_id, enable=False) != 0:
    #     print("Could not resume.")

    _, current_pos = sim.simxGetJointPosition(clientID=client_id, jointHandle=joint_handle,
                                              operationMode=sim.simx_opmode_blocking)

    # print(math.degrees(current_pos), target_pos - current_pos, ping_time)  # debugging only


def get_arm_joint_handles(client_id):
    """
    Uses the API to fetch the arm joint handles in the following order:
        - robotic_arm_rj_r1
        - robotic_arm_rj_12
        - robotic_arm_rj_23
        - robotic_arm_rj_34

    :rtype: tuple
    :param client_id: the client id of the communication thread
    :return: a tuple of arm joint handles
    """
    _, joint_handle_r1 = sim.simxGetObjectHandle(clientID=client_id, objectName="robotic_arm_rj_r1",
                                                 operationMode=sim.simx_opmode_blocking)
    _, joint_handle_12 = sim.simxGetObjectHandle(clientID=client_id, objectName="robotic_arm_rj_12",
                                                 operationMode=sim.simx_opmode_blocking)
    _, joint_handle_23 = sim.simxGetObjectHandle(clientID=client_id, objectName="robotic_arm_rj_23",
                                                 operationMode=sim.simx_opmode_blocking)
    _, joint_handle_34 = sim.simxGetObjectHandle(clientID=client_id, objectName="robotic_arm_rj_34",
                                                 operationMode=sim.simx_opmode_blocking)

    joint_handles = (joint_handle_r1, joint_handle_12, joint_handle_23, joint_handle_34)

    return joint_handles


def get_basket_joint_handle(client_id):
    """
    Uses the API to fetch the basket joint handle:

    :rtype: int
    :param client_id: the client id of the communication thread
    :return: a tuple of arm joint handles
    """
    _, basket_joint_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="basket_rj",
                                                     operationMode=sim.simx_opmode_blocking)

    return basket_joint_handle


def open_close_gripper(client_id, command):
    """
    Used to open or close the gripper jaw. Doesn't wait for the operation to finish.

    :param client_id: the client id of the communication thread
    :param command: Must be "open" or "close"
    """
    input_ints = []
    input_floats = []
    input_strings = [command]
    input_buffer = bytearray()
    res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(clientID=client_id,
                                                                                scriptDescription='gripper',
                                                                                options=sim.sim_scripttype_childscript,
                                                                                functionName='open_close',
                                                                                inputInts=input_ints,
                                                                                inputFloats=input_floats,
                                                                                inputStrings=input_strings,
                                                                                inputBuffer=input_buffer,
                                                                                operationMode=sim.simx_opmode_blocking)
    if res != sim.simx_return_ok:  # some error occurred
        print(retInts)
        print(retFloats)
        print(retStrings)
        print(retBuffer)


def target_berry(client_id: int, berry_name: str, berry_positions_dictionary: dict = None):
    """
    Targets a berry to find, pick and drop
    :param client_id: the client id of the communication thread
    :param berry_name: the type of berry
    :param berry_positions_dictionary: a dictionary containing tbe berry positions
    """

    if berry_positions_dictionary is None:
        # find berry
        # clear_vision_sensor(client_id)
        berries_dictionary, berry_positions_dictionary = find_berries(client_id)
        berry_positions_dictionary = sort_dictionary(berry_positions_dictionary)

    if len(berry_positions_dictionary[berry_name]) > 0:
        x_coord = berry_positions_dictionary[berry_name][0][0]
        y_coord = berry_positions_dictionary[berry_name][0][1]
        depth = berry_positions_dictionary[berry_name][0][2]
        del berry_positions_dictionary[berry_name][0]  # remove berry from list
        print(berry_name, x_coord, y_coord, depth)  # debugging only
        return_code = send_identified_berry_data(client_id, berry_name, x_coord, y_coord, depth)  # evaluation only
        # print(return_code)  # debugging during evaluation only

        reset_bot(client_id)

        # pick berry
        print("Picking", berry_name, "...", (x_coord, y_coord, depth))  # debugging only
        # pick_berry_using_sim_ik(client_id, x_coord, y_coord, depth)
        # Logic 1
        # align_arm_with_berry(client_id, x_coord, depth)
        # pluck_berry(client_id, *get_angles_from_coord(x_coord, y_coord, depth))
        # reset_bot(client_id)
        # Logic 2
        align_bot_with_berry(client_id, x_coord, "set")
        pluck_berry(client_id, *get_angles_from_coord(0, y_coord, depth))
        reset_bot(client_id)
        align_bot_with_berry(client_id, x_coord, "unset")
        # either Logic 1 or Logic 2 must be used

        # drop berry
        print("Dropping berry...")  # debugging only
        drop_berry_in_basket(client_id)
        reset_bot(client_id)

    else:
        print("No berry of type", berry_name, "found.")

    return


def align_arm_with_berry(client_id, x_coord, depth):
    """
    Redundant function currently.
    """
    from shared_resources import arm_joint_handles
    angle = -math.degrees(math.atan2(x_coord, depth))
    # print("Aligning arm...")  # debugging only
    # print(x_coord, depth, angle)  # debugging only
    move_arm(client_id, arm_joint_handles[0], angle)


def align_bot_with_berry(client_id, x_coord, command):
    """
    Used to align the robot with the berry to be plucked
    """
    import move_commands
    from shared_resources import wheel_joints
    # x_coord=x_coord*1.05 #Offset to prevent collision
    if command == "set":
        if x_coord > 0:
            translate(client_id, move_commands.move_right, wheel_joints, distance=x_coord)
        else:
            translate(client_id, move_commands.move_left, wheel_joints, distance=-x_coord)
    elif command == "unset":
        if x_coord > 0:
            translate(client_id, move_commands.move_left, wheel_joints, distance=x_coord)
        else:
            translate(client_id, move_commands.move_right, wheel_joints, distance=-x_coord)


def pluck_berry(client_id, t1, t2, t3):
    """
    Plucks the berry by using the values obtained from numerical solution of IK
    """
    open_close_gripper(client_id, "open")
    from shared_resources import arm_joint_handles
    move_arm(client_id, arm_joint_handles[2], t2)
    move_arm(client_id, arm_joint_handles[3], t3)
    move_arm(client_id, arm_joint_handles[1], t1)
    arm_stop(client_id, arm_joint_handles)
    open_close_gripper(client_id, "close")
    time.sleep(1)  # wait for gripper to close


def drop_berry_in_basket(client_id):
    """
    Manipulates the arm to release the berry in the storage basket
    """
    from shared_resources import arm_joint_handles
    # move_arm(client_id, arm_joint_handles[0], 0)
    move_arm(client_id, arm_joint_handles[1], -60)
    move_arm(client_id, arm_joint_handles[2], 0)
    move_arm(client_id, arm_joint_handles[3], 180)

    open_close_gripper(client_id, "open")
    time.sleep(1)  # wait for gripper to open


def tip_basket(client_id):
    """
    Tips the basket forward by 60 degrees.
    100% effective when vision_sensor_1 is directly over the QR code near the collection box.
    """
    print("Tipping Basket ")  # Debugging
    basket_joint_handle = get_basket_joint_handle(client_id)
    move_arm(client_id, basket_joint_handle, -60)
    time.sleep(1)
    move_arm(client_id, basket_joint_handle, 0)
    time.sleep(1)


def reset_bot(client_id):
    """
    Brings arm back to neutral position
    """
    from shared_resources import arm_joint_handles
    move_arm(client_id, arm_joint_handles[3], 0)
    move_arm(client_id, arm_joint_handles[1], 0)
    move_arm(client_id, arm_joint_handles[2], 0)
    move_arm(client_id, arm_joint_handles[0], 0)

    arm_stop(client_id, arm_joint_handles)


def clear_vision_sensor(client_id):
    """
    Removes all objects from the fov of the robot
    """
    from shared_resources import arm_joint_handles
    move_arm(client_id, arm_joint_handles[0], 0)
    move_arm(client_id, arm_joint_handles[1], -60)
    move_arm(client_id, arm_joint_handles[2], 0)
    move_arm(client_id, arm_joint_handles[3], 0)

    arm_stop(client_id, arm_joint_handles)


def get_angles_from_coord(x, y, z):
    """
    Function for inverse kinematics.
    It is supposed to find the arm angles from the berry position.
    
    :param x: x-coordinate of the target
    :param y: y-coordinate of the target
    :param z: depth of the target
    :return: t12, t23, t34 - the final angles made by joints 12,23,34 (destination)
    """
    # Length of arms in meters
    l1 = 0.35  # Length of link 1
    l2 = 0.2984  # length of link 2
    l3 = 0.375  # length of link 3
    offset = 0.3299
    # Length of arms in meters
    # l1 = 0.3489  # Length of link 1
    # l2 = 0.3003  # length of link 2
    # l3 = 0.4229  # length of link 3
    # offset = 0.41

    l3ang = 0  # the constant angle l3 must maintain with the ground plane. Must be in degrees
    x = (x ** 2 + z ** 2) ** 0.5  # Effective x_coord for 2d ik
    y = -y + offset  # Effective y for 2d ik
    res = []  # Stores results
    resang = []  # Stores the angle of joints

    # Initializing t12, t23, t34
    t12 = 0
    t23 = 0
    t34 = 0

    try:
        # Part 1 of function iterates through all possible values of t1 and t2 in a given range (can be extended) to obtain
        # values of t1 and t2 which give least difference from x and y. The value is appended to res list
        for t1 in range(0, 90):
            for t2 in range(0, 90):
                if (abs(l1 * math.sin(math.radians(t1)) + l2 * math.cos(
                        math.radians(60 - t2 + t1)) + l3 - x) <= 0.2 and abs(
                    l1 * math.cos(math.radians(t1)) - l2 * math.sin(math.radians(60 - t2 + t1)) - y) <= 0.2):
                    res.append(
                        [abs(l1 * math.sin(math.radians(t1)) + l2 * math.cos(math.radians(60 - t2 + t1)) + l3 - x),
                         abs(l1 * math.cos(math.radians(t1)) - l2 * math.sin(math.radians(60 - t2 + t1)) - y)])

        # Part 2 of function is to find and store the value of t1 and t2 obtained in the earlier function.
        for t1 in range(0, 90):
            for t2 in range(0, 90):
                if (abs(l1 * math.sin(math.radians(t1)) + l2 * math.cos(math.radians(60 - t2 + t1)) + l3 - x) ==
                        min(res, key=max)[0] and abs(
                            l1 * math.cos(math.radians(t1)) - l2 * math.sin(math.radians(60 - t2 + t1)) - y) ==
                        min(res, key=max)[1]):
                    resang.append([t1, t2])
                    break

        print(min(res, key=max))  # Debugging
        print(resang)  # Debugging
        t34 = 80 - resang[0][0] + resang[0][1]
        t23 = -1 * resang[0][1]
        t12 = resang[0][0]
        print([t12, t23, t34])  # Debugging
    except:
        return t12, t23, t34
    return t12, t23, t34


def pick_berry_using_sim_ik(client_id, x_coord, y_coord, depth):
    """
    Attempt at using sim ik (Failed)
    """
    open_close_gripper(client_id, "open")
    return_code = non_functional_target(client_id, False)
    print(return_code)  # debugging only
    return_code = set_target_dummy_position(client_id, x_coord, y_coord, depth)
    print(return_code)  # debugging only
    # return_code = move_to_target(client_id)
    # print(return_code)  # debugging only
    # for i in range(100000):
    #     pass  # time pass
    # time.sleep(30)  # wait for gripper to reach target
    arm_stop(client_id, arm_joint_handles)
    return_code = non_functional_target(client_id, True)
    print(return_code)  # debugging only
    open_close_gripper(client_id, "close")
    return_code = non_functional_target(client_id, True)
    time.sleep(1)  # wait for gripper to close


def non_functional_target(client_id, state):
    """
    Sim IK function
    """
    emptybuff = bytearray()

    if (type(state) != bool):
        state = bool(state)

    data_to_send = [str(state).lower()]
    # print("Sending data", data_to_send)  # debugging only
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',
                                                                                      sim.sim_scripttype_childscript,
                                                                                      'non_functional_target', [], [],
                                                                                      data_to_send, emptybuff,
                                                                                      sim.simx_opmode_blocking)
    return return_code


def set_target_dummy_position(client_id, x_coord, y_coord, depth):
    """
    Sim IK Function
    """
    emptybuff = bytearray()

    if (type(x_coord) != float):
        x_coord = float(x_coord)
    if (type(y_coord) != float):
        y_coord = float(y_coord)
    if (type(depth) != float):
        depth = float(depth)

    data_to_send = [x_coord, y_coord, depth]
    print("Sending data", data_to_send)  # debugging only
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',
                                                                                      sim.sim_scripttype_childscript,
                                                                                      'set_target_dummy_position', [],
                                                                                      data_to_send, [], emptybuff,
                                                                                      sim.simx_opmode_blocking)
    print("Data received", oufloats)  # debugging only
    return return_code


def move_to_target(client_id):
    """
    Sim IK Function
    """
    emptybuff = bytearray()

    data_to_send = []
    return_code = -1

    for i in range(1):
        return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'robotic_arm',
                                                                                          sim.sim_scripttype_childscript,
                                                                                          'create_IK_groups', [],
                                                                                          data_to_send, [], emptybuff,
                                                                                          sim.simx_opmode_blocking)
    return return_code


##############################################################
def send_identified_berry_data(client_id, berry_name, x_coor, y_coor, depth):
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
    ##################################################
    ## You are NOT allowed to make any changes in the code below. ##
    emptybuff = bytearray()

    if (type(berry_name) != str):
        berry_name = str(berry_name)

    if (type(x_coor) != float):
        x_coor = float(x_coor)

    if (type(y_coor) != float):
        y_coor = float(y_coor)

    if (type(depth) != float):
        depth = float(depth)

    data_to_send = [berry_name, str(x_coor), str(y_coor), str(depth)]
    return_code, outints, oufloats, outstring, outbuffer = sim.simxCallScriptFunction(client_id, 'eval_bm',
                                                                                      sim.sim_scripttype_childscript,
                                                                                      'detected_berry_by_team', [], [],
                                                                                      data_to_send, emptybuff,
                                                                                      sim.simx_opmode_blocking)
    return return_code

    ##################################################


def task_4_primary(client_id):
    """
    Purpose:
    ---
    This is the only function that is called from the main function. Make sure to fill it
    properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

    Input Arguments:
    ---
    `client_id`         :   [ integer ]
        the client id of the communication thread returned by init_remote_api_server()

    
    Returns:
    ---
    
    Example call:
    ---
    task_4_primary(client_id)
    
    """

    global ping_time
    _, ping_time = sim.simxGetPingTime(clientID=client_id)
    ping_time /= 1000  # convert to secs

    global arm_joint_handles
    arm_joint_handles = get_arm_joint_handles(client_id)

    global wheel_joints
    wheel_joints = init_setup(client_id)
    _, bot_base = sim.simxGetObjectHandle(clientID=client_id, objectName="BM_Bot",
                                          operationMode=sim.simx_opmode_blocking)

    # reset_bot(client_id)
    # arm_stop(client_id, arm_joint_handles)

    # print(encoders(client_id))
    # task_3_primary(client_id, target_points=[(4, 3.7)])  # the bot is a little too big

    # print("Starting Movement...")
    # move_arm(client_id, arm_joint_handles[0], 90)
    # rotate_arm(client_id, arm_joint_handles[0], 90, speed=0.2)
    # move_arm(client_id, arm_joint_handles[1], 30)
    # rotate_arm(client_id, arm_joint_handles[1], -30)
    # move_arm(client_id, arm_joint_handles[2], -45)
    # rotate_arm(client_id, arm_joint_handles[2], 45)
    # move_arm(client_id, arm_joint_handles[3], 60)
    # rotate_arm(client_id, arm_joint_handles[3], -60)
    # arm_stop(client_id, arm_joint_handles)

    # robo_see(client_id)  # debugging only

    # open_close_gripper(client_id, "open")
    # time.sleep(1.5)  # time to observe
    # open_close_gripper(client_id, "close")
    # tip_basket(client_id)
    # last_ditch_effort(client_id)
    # target_berry(client_id, 'Lemon')
    # target_berry(client_id, 'Strawberry')
    # target_berry(client_id, 'Blueberry')

    # time.sleep(3)  # time to observe

    # print(encoders(client_id))
    return


if __name__ == "__main__":

    ##################################################
    ## You are NOT allowed to make any changes in the code below ##

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
                print('\n[ERROR] Your start_simulation function threw an Exception, kindly debug your code!')
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
        print('\n[ERROR] Your init_remote_api_server function threw an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()

    try:

        task_4_primary(client_id)
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
                    print(
                        '\n[ERROR] Your exit_remote_api_server function threw an Exception, kindly debug your code!')
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
            print('\n[ERROR] Your stop_simulation function threw an Exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

    except Exception:
        print('\n[ERROR] Your control_logic function threw an Exception, kindly debug your code!')
        print('Stop the CoppeliaSim simulation manually if started.\n')
        traceback.print_exc(file=sys.stdout)
        print()
        sys.exit()
