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

# Team ID:			BM_2539
# Author List:		Swastik Pal, Amanullah Asad, Hritik Singla
# Filename:			task_3.py
# Utility Functions:
#                   is_bot_at_rest, time_to_wait, stop_bot, translate, rotate, view_image
#                   determine_velocity, determine_angular_velocity [Debugging only]
# Global variables:
#


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
import sys
from pyzbar.pyzbar import decode

import shared_resources

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

def is_bot_at_rest(client_id, time_to_observe=0.01):
    """
    This function watches the bot to determine if it is moving.

    :rtype: bool
    :return: True if the wheel joints are stationary, False otherwise
    :param client_id: the client id of the communication thread
    :param time_to_observe: the duration for which the wheels will be observed
    """

    at_rest = False

    old_position = encoders(client_id)
    time.sleep(time_to_observe)
    new_position = encoders(client_id)
    diff = new_position - old_position

    if np.isclose(diff, 0):
        at_rest = True

    return at_rest


def time_to_wait(target: float, acceleration: float, max_velocity_allowed: float):
    """
    This function uses the equations of motion to determine the time
    a body should start decelerating to cover the given distance(target)
    considering it starts from rest and decelerates to a stop.
    at^2 + atT = target
    We need to find t + T
    where t <= max_velocity_allowed/acceleration
    :param target: the distance to cover. The distance between two consecutive points is 10 units.
    :param acceleration:
    :param max_velocity_allowed:
    :return:
    :rtype: float
    """
    max_accl_time = max_velocity_allowed / acceleration
    t = math.sqrt(target / acceleration) / 2
    if t > max_accl_time:
        t = target / max_velocity_allowed
        # t += max_accl_time
    return t


def stop_bot(client_id, wheel_joints, wait_till_stop=True):
    """
    Stops the bot.

    :param client_id: the client id of the communication thread
    :param wheel_joints: list containing joint object handles of individual joints
    :param wait_till_stop:
    """
    if sim.simxPauseCommunication(clientID=client_id, enable=True) != 0:
        print("Could not pause.")
    joint_target_velocity = 0
    for joint in wheel_joints:
        sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=joint,
                                       targetVelocity=joint_target_velocity, operationMode=sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID=client_id, enable=False)

    if wait_till_stop:
        joint_positions = encoders(client_id)
        error = joint_positions
        old_joint_positions = joint_positions
        while np.isclose(all(error), 0, atol=0.001):
            joint_positions = encoders(client_id)
            error = old_joint_positions - joint_positions

    return


def get_bot_position(client_id):
    """
    Returns the position(s) read by vision_sensor_1
    :param client_id: the client id of the communication thread
    :return: a list of qr_code detected. Can be empty.
    """
    vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    # view_image(transformed_image)  # debugging only
    qr_codes = detect_qr_codes(transformed_image)
    # print("qr_codes:", qr_codes)  # debugging only
    return qr_codes


def translate(client_id, command, wheel_joints, distance):
    """
    Moves the bot.

    :param client_id: the client id of the communication thread
    :param command: tells the bot how to move
    :param distance: the distance to move (in meters)
    :param wheel_joints: list containing joint object handles of individual joints
    :return:
    """

    from move_commands import move_left, move_right, move_forward, move_backward

    if command == move_forward:
        scaling_factor = shared_resources.forward_scaling_factor
        direction = [1, 1, 1, 1]

    elif command == move_backward:
        scaling_factor = shared_resources.backward_scaling_factor
        direction = [-1, -1, -1, -1]

    elif command == move_left:
        scaling_factor = shared_resources.left_scaling_factor
        direction = [-1, 1, 1, -1]

    elif command == move_right:
        scaling_factor = shared_resources.right_scaling_factor
        direction = [1, -1, -1, 1]

    else:
        print("Command not recognized.")
        return

    current_pos = encoders(client_id)
    distance = scaling_factor * distance
    target_pos = np.add(current_pos, np.multiply(direction, np.repeat(distance, 4)))

    kp = 2
    ki = 0.01
    kd = 0.01
    error = direction[0] * (target_pos[0] - current_pos[0])
    error_old = error
    error_diff = 0
    accumulation = 0
    del_time = 1
    derivative = 0
    speed = kp * error
    # if math.fabs(speed) > speed_limit:
    #     speed = math.copysign(speed_limit, speed)
    clock_start = time.monotonic()

    pause_and_set_joint_target_velocity(client_id, wheel_joints,
                                        target_velocity_fl=direction[0] * speed,
                                        target_velocity_fr=direction[1] * speed,
                                        target_velocity_rl=direction[2] * speed,
                                        target_velocity_rr=direction[3] * speed)

    while not math.isclose(error, 0, abs_tol=0.01):
        current_pos = encoders(client_id)
        error = direction[0] * (target_pos[0] - current_pos[0])
        speed = kp * error + ki * accumulation + kd * derivative
        # if math.fabs(speed) > speed_limit:
        #     speed = math.copysign(speed_limit, speed)
        clock_end = clock_start
        clock_start = time.monotonic()
        pause_and_set_joint_target_velocity(client_id, wheel_joints,
                                            target_velocity_fl=direction[0] * speed,
                                            target_velocity_fr=direction[1] * speed,
                                            target_velocity_rl=direction[2] * speed,
                                            target_velocity_rr=direction[3] * speed)
        error_diff = error - error_old
        error_old = error
        # speed_old = speed
        del_time = clock_end - clock_start
        accumulation += error_diff * del_time
        derivative = error_diff / del_time

    stop_bot(client_id, wheel_joints)

    # current_pos = encoders(client_id)  # debugging only
    # error = direction[0] * (target_pos[0] - current_pos[0])  # debugging only
    # print("Error...", error)  # debugging only


def rotate(client_id, wheel_joints, rotate_through):
    """
    Rotates the bot.

    :param client_id: the client id of the communication thread
    :param wheel_joints: list containing joint object handles of individual joints
    :param rotate_through: the angle to rotate (in degrees)
    :return:
    """

    direction = -math.copysign(1, rotate_through)  # 1 means rotate left, -1 means rotate right, 0 means stop rotation
    scaling_factor = shared_resources.rotation_scaling_factor  # a constant
    speed_limit = 10  # in degrees/s

    current_pos = encoders(client_id)
    rotate_through = (scaling_factor * math.pi * rotate_through) / 90  # converts bot rotation to joint rotation
    target_pos = np.add(current_pos, [-rotate_through, rotate_through, -rotate_through, rotate_through])

    kp = 2
    ki = 0.08
    kd = 0.1
    error = direction * ((target_pos[0] - target_pos[1] + target_pos[2] - target_pos[3]) - (
            current_pos[0] - current_pos[1] + current_pos[2] - current_pos[3])) / 4
    error_old = error
    error_diff = 0
    accumulation = 0
    del_time = 1
    derivative = 0
    speed = kp * error
    # if math.fabs(speed) > speed_limit:
    #     speed = math.copysign(speed_limit, speed)
    clock_start = time.monotonic()

    pause_and_set_joint_target_velocity(client_id, wheel_joints,
                                        target_velocity_fl=direction * speed,
                                        target_velocity_fr=-direction * speed,
                                        target_velocity_rl=direction * speed,
                                        target_velocity_rr=-direction * speed)

    while not math.isclose(error, 0, abs_tol=0.01):
        current_pos = encoders(client_id)
        error = direction * ((target_pos[0] - target_pos[1] + target_pos[2] - target_pos[3]) - (
            current_pos[0] - current_pos[1] + current_pos[2] - current_pos[3])) / 2
        speed = kp * error + ki * accumulation + kd * derivative
        # if math.fabs(speed) > speed_limit:
        #     speed = math.copysign(speed_limit, speed)
        clock_end = clock_start
        clock_start = time.monotonic()
        pause_and_set_joint_target_velocity(client_id, wheel_joints,
                                            target_velocity_fl=direction * speed,
                                            target_velocity_fr=-direction * speed,
                                            target_velocity_rl=direction * speed,
                                            target_velocity_rr=-direction * speed)
        error_diff = error - error_old
        error_old = error
        # speed_old = speed
        del_time = clock_end - clock_start
        accumulation += error_diff * del_time
        derivative = error_diff / del_time

    stop_bot(client_id, wheel_joints)

    # current_pos = encoders(client_id)  # debugging only
    # error = direction * (target_pos[0] - current_pos[0])  # debugging only
    # print("Error...", error)  # debugging only


def strafe(client_id, wheel_joints, x, y):
    """
    Adds strafing support to the bot. The bot must be stopped manually after invoking this function.

    :param client_id: the client id of the communication thread
    :param wheel_joints: list containing joint object handles of individual joints
    :param x: the distance to move along the X-axis
    :param y: the distance to move along the Y-axis
    :return:
    """

    scaling_factor = 1.00159
    x = x * scaling_factor  # counteract imperfect strafing
    angle = math.atan2(y, x)
    magnitude = math.sqrt(x ** 2.0 + y ** 2.0)

    quarter_pi = math.pi / 4
    target_velocity_fl = math.sin(angle + quarter_pi) * magnitude
    target_velocity_fr = math.sin(angle - quarter_pi) * magnitude
    target_velocity_rl = math.sin(angle - quarter_pi) * magnitude
    target_velocity_rr = math.sin(angle + quarter_pi) * magnitude

    pause_and_set_joint_target_velocity(client_id, wheel_joints,
                                        target_velocity_fl,
                                        target_velocity_fr,
                                        target_velocity_rl,
                                        target_velocity_rr)


def pause_and_set_joint_target_velocity(client_id: int, wheel_joints: list,
                                        target_velocity_fl: float, target_velocity_fr: float,
                                        target_velocity_rl: float, target_velocity_rr: float) -> None:
    """
    Sets the target velocities of all the wheel joints.

    :param client_id: the client id of the communication thread
    :param wheel_joints: a list containing joint object handles of individual joints
    :param target_velocity_fl: target velocity of the front-left wheel
    :param target_velocity_fr: target velocity of the front-right wheel
    :param target_velocity_rl: target velocity of the rear-left wheel
    :param target_velocity_rr: target velocity of the rear-right wheel
    """
    if sim.simxPauseCommunication(clientID=client_id, enable=True) != sim.simx_return_ok:
        print("Could not pause.")

    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=wheel_joints[0],
                                   targetVelocity=target_velocity_fl,
                                   operationMode=sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=wheel_joints[1],
                                   targetVelocity=target_velocity_fr,
                                   operationMode=sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=wheel_joints[2],
                                   targetVelocity=target_velocity_rl,
                                   operationMode=sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID=client_id, jointHandle=wheel_joints[3],
                                   targetVelocity=target_velocity_rr,
                                   operationMode=sim.simx_opmode_oneshot)

    if sim.simxPauseCommunication(clientID=client_id, enable=False) != sim.simx_return_ok:
        print("Could not resume.")


def view_image(image: object, wait: int = 1000, window_label="Image"):
    """
    Displays the provided image in a window named "Image". Uses cv2.

    :param window_label: The name of the window
    :param image: The image to display
    :param wait: The minimum time (in milliseconds) to wait before destroying the window
    :return: Nothing
    """
    cv2.imshow(window_label, image)
    cv2.waitKey(wait)
    cv2.destroyWindow(window_label)


def determine_velocity(client_id, time_to_observe):  # debugging only
    """
    The velocity is calculated as displacement / Del(time).
    This function MUST NOT BE USED in production.

    :param client_id: the client id
    :param time_to_observe: the time period
    :return: the velocity (v_x, v_y, v_z) as a list
    :rtype: tuple
    """
    velocity = []
    # finding velocity of bot
    _, bot_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="BM_Bot",
                                            operationMode=sim.simx_opmode_blocking)
    _, bot_old_position = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_handle,
                                                    relativeToObjectHandle=-1, operationMode=sim.simx_opmode_blocking)
    time.sleep(time_to_observe)
    _, bot_new_position = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_handle,
                                                    relativeToObjectHandle=-1, operationMode=sim.simx_opmode_blocking)
    for i in range(3):
        velocity.append((bot_new_position[i] - bot_old_position[i]) / time_to_observe)
    return velocity


def determine_angular_velocity(client_id, time_to_observe):  # debugging only
    """
    The velocity is calculated as displacement / Del(time).
    This function MUST NOT BE USED in production.

    :param client_id: the client id
    :param time_to_observe: the time period
    :return: the velocity (v_x, v_y, v_z) as a list
    :rtype: tuple
    """
    angular_velocity = []
    # finding velocity of bot
    _, bot_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="BM_Bot",
                                            operationMode=sim.simx_opmode_blocking)
    _, bot_old_orientation = sim.simxGetObjectOrientation(clientID=client_id, objectHandle=bot_handle,
                                                          relativeToObjectHandle=-1,
                                                          operationMode=sim.simx_opmode_blocking)
    time.sleep(time_to_observe)
    _, bot_new_orientation = sim.simxGetObjectOrientation(clientID=client_id, objectHandle=bot_handle,
                                                          relativeToObjectHandle=-1,
                                                          operationMode=sim.simx_opmode_blocking)
    for i in range(3):
        angular_velocity.append((bot_new_orientation[i] - bot_old_orientation[i]) / time_to_observe)
    return angular_velocity


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

    # Closing all open connections

    sim.simxFinish(clientID=client_id)

    # Starting new connection with localhost:19997
    client_id = sim.simxStart(connectionAddress="127.0.0.1", connectionPort=19997, waitUntilConnected=True,
                              doNotReconnectOnceDisconnected=False, timeOutInMs=5000, commThreadCycleInMs=5)

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

    return_code = sim.simxStartSimulation(clientID=client_id, operationMode=sim.simx_opmode_blocking)

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

    ##############	ADD YOUR CODE HERE	##############

    sim_img_colour_rgb = 0
    sim_img_colour_grey = 1

    return_code, sensor_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="vision_sensor_1",
                                                         operationMode=sim.simx_opmode_blocking)
    return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(clientID=client_id,
                                                                                      sensorHandle=sensor_handle,
                                                                                      options=sim_img_colour_rgb,
                                                                                      operationMode=sim.simx_opmode_blocking)

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

    transformed_image = None

    ##############	ADD YOUR CODE HERE	##############

    # Converting list to Numpy array
    transformed_image = np.array(vision_sensor_image, dtype=np.uint8)

    # Reshaping 1-D unit8 array to 3-D (length, breadth, 3) array (image)
    transformed_image.shape = (image_resolution[0], image_resolution[1], 3)
    # view_image(transformed_image)

    # Converting colourspace from BGR to RBG
    transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
    # view_image(transformed_image)

    # Flipping the image about the X-axis
    transformed_image = np.fliplr(transformed_image)
    # view_image(transformed_image)

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

    return_code = sim.simxStopSimulation(clientID=client_id, operationMode=sim.simx_opmode_blocking)

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

    sim.simxFinish(clientID=client_id)

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

    qr_codes = []

    ##############	ADD YOUR CODE HERE	##############

    gray_transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
    detected_codes = decode(gray_transformed_image)

    for code in detected_codes:
        res = []
        (x, y, w, h) = code.rect
        code_data = code.data.decode("utf-8")
        res.append(code_data)
        # res.append((x, y))
        res.append(((x + w) / 2, (y + h) / 2))
        qr_codes.append(res)
        # qr_codes.append(code_data)

    ##################################################

    return qr_codes


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

    wheel_joints = []

    ##############	ADD YOUR CODE HERE	##############

    _, joint_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="rollingJoint_fl",
                                              operationMode=sim.simx_opmode_blocking)
    wheel_joints.append(joint_handle)
    _, joint_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="rollingJoint_fr",
                                              operationMode=sim.simx_opmode_blocking)
    wheel_joints.append(joint_handle)
    _, joint_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="rollingJoint_rl",
                                              operationMode=sim.simx_opmode_blocking)
    wheel_joints.append(joint_handle)
    _, joint_handle = sim.simxGetObjectHandle(clientID=client_id, objectName="rollingJoint_rr",
                                              operationMode=sim.simx_opmode_blocking)
    wheel_joints.append(joint_handle)

    ##################################################

    return wheel_joints  # the joints are stored in order - fl, fr, rl, rr


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

    return_code, signal_value = sim.simxGetStringSignal(client_id, 'combined_joint_position', sim.simx_opmode_blocking)
    signal_value = signal_value.decode()
    joints_position = signal_value.split("%")

    for index, joint_val in enumerate(joints_position):
        joints_position[index] = float(joint_val)

    # change the order of joints
    fl = joints_position[0]
    rl = joints_position[1]
    rr = joints_position[2]
    fr = joints_position[3]
    joints_position = [fl, fr, rl, rr]

    return joints_position


def nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False, vert_first=False):
    """
    Purpose:
    ---
    This function implements the bot's navigation logic.
    :param client_id: the client id of the communication thread
    :param target_points: the target navigational co-ordinates
    :param wheel_joints: a list containing joint object handles of individual joints
    :param straight_line_movement: whether the bot should move in straight lines.
    (Should be set to True in the absence of obstacles)
    :param vert_first: if straight_line_movement is disabled, should the bot move vertically first
    """

    from move_commands import move_forward, move_backward, move_left, move_right

    path = shortest_path(target_points, shared_resources.bot_coord, r_theta_resolution=straight_line_movement)

    if straight_line_movement:
        for i in range(0, len(path)):
            dist, angle = path[i]

            angle_to_rotate = angle - shared_resources.bot_front_angle
            rotate(client_id, wheel_joints, angle_to_rotate)
            shared_resources.bot_front_angle = angle

            distance_to_cover = dist * shared_resources.y_distance_between_two_consecutive_points
            translate(client_id, move_forward, wheel_joints, distance_to_cover)
            shared_resources.bot_coord = target_points[i]

    else:
        if shared_resources.bot_front_angle != 0 and shared_resources.bot_front_angle != 90:
            angle = 90  # make the bot face upward
            angle_to_rotate = angle - shared_resources.bot_front_angle
            rotate(client_id, wheel_joints, angle_to_rotate)
            shared_resources.bot_front_angle = angle

        for i in range(0, len(path)):
            hor, vert = path[i]
            if shared_resources.bot_front_angle == 0:  # adjust
                temp = hor
                hor = -vert
                vert = temp
                vert_first = not vert_first

            vertical_distance_to_cover = math.fabs(vert * shared_resources.y_distance_between_two_consecutive_points)
            horizontal_distance_to_cover = math.fabs(hor * shared_resources.x_distance_between_two_consecutive_points)

            if vert_first:

                if vert < 0:
                    translate(client_id, move_backward, wheel_joints, vertical_distance_to_cover)
                elif vert > 0:
                    translate(client_id, move_forward, wheel_joints, vertical_distance_to_cover)
                if hor < 0:
                    translate(client_id, move_left, wheel_joints, horizontal_distance_to_cover)
                elif hor > 0:
                    translate(client_id, move_right, wheel_joints, horizontal_distance_to_cover)

            else:

                if hor < 0:
                    translate(client_id, move_left, wheel_joints, horizontal_distance_to_cover)
                elif hor > 0:
                    translate(client_id, move_right, wheel_joints, horizontal_distance_to_cover)
                if vert < 0:
                    translate(client_id, move_backward, wheel_joints, vertical_distance_to_cover)
                elif vert > 0:
                    translate(client_id, move_forward, wheel_joints, vertical_distance_to_cover)

            shared_resources.bot_coord = target_points[i]

    stop_bot(client_id, wheel_joints)
    # time.sleep(1)
    return


def shortest_path(target_points, origin=None, r_theta_resolution=False):
    """
    Purpose:
    ---
    This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
    It can work in two ways.
    Straight line movement creates larger errors compared to xy movement but is much faster.

    Input Arguments:
    ---
    `target_points`         :   [ list ]
        the points where the bot is supposed to move to
    `r_theta_resolution`    :   [ boolean ]
        whether to resolve into r and theta components

    Returns:
    ---
    `path`      :   [ (x,y)]
        Python list containing the x-axis and y-axis translation respectively
    `path`      :   [ (r,theta)]
        Python list containing the r (translation) and theta (rotation) respectively

    Example call:
    ---
    shortest_path(target_points)
    """
    if origin is None:
        origin = [0, 0]
    path = []  # Each element of this list tells how much the car has to move

    if r_theta_resolution:
        """
        This variation on the above function finds the angle of rotation in degrees and distance to be moved to move from one point to another
        sqrt(x^2 + y^2)
        arctan2(y/x) which is a special variation of arctan. Definition as copied from wikipedia: https://en.wikipedia.org/wiki/Atan2
        Returns value in range (-pi to pi) with all angles measured from +ve x axis. 
        """
        for i in range(0, len(target_points)):
            angle = math.atan2((target_points[i][1] - origin[1]), (target_points[i][0] - origin[0])) * 180 / math.pi
            dist = math.sqrt((target_points[i][0] - origin[0]) ** 2 + (target_points[i][1] - origin[1]) ** 2)
            path.append((dist, angle))
            origin = [target_points[i][0], target_points[i][1]]
    else:
        for i in range(0, len(target_points)):
            hor = target_points[i][0] - origin[0]
            vert = target_points[i][1] - origin[1]
            path.append((hor, vert))
            origin = [target_points[i][0], target_points[i][1]]

    return path


def calibrate():
    """
    Used to determine the scaling factor after major changes to the bot.
    The bot should have enough free space to move around.

    This function MUST NOT BE USED in production.
    """

    client_id = shared_resources.client_id
    # ping_time = shared_resources.ping_time
    # arm_joint_handles = shared_resources.arm_joint_handles
    wheel_joints = shared_resources.wheel_joints
    _, bot_base = sim.simxGetObjectHandle(clientID=client_id, objectName="BM_Bot",
                                          operationMode=sim.simx_opmode_blocking)

    # testing and determining scaling factor

    _, initial_ort = sim.simxGetObjectOrientation(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
                                                  operationMode=sim.simx_opmode_blocking)
    initial_ort = encoders(client_id)
    rotate(client_id, wheel_joints, 90)
    # rotate(client_id, wheel_joints, -90)
    # rotate(client_id, wheel_joints, -180)
    # rotate(client_id, wheel_joints, 270)
    # rotate(client_id, wheel_joints, 360)
    _, final_ort = sim.simxGetObjectOrientation(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
                                                operationMode=sim.simx_opmode_blocking)
    final_ort = encoders(client_id)
    print("Angle rotated =", np.subtract(final_ort, initial_ort) * 90 / (4 * math.pi))

    # testing and determining scaling factor

    # from move_commands import move_forward, move_backward, move_left, move_right
    #
    # _, initial_pos = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
    #                                            operationMode=sim.simx_opmode_blocking)
    # translate(client_id, move_backward, wheel_joints, distance=1)
    # _, final_pos = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
    #                                          operationMode=sim.simx_opmode_blocking)
    # print("Distance moved =", np.subtract(final_pos, initial_pos))

    # testing and determining scaling factor
    # _, initial_pos = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
    #                                            operationMode=sim.simx_opmode_blocking)
    # strafe(client_id, wheel_joints, -5, 5)
    # time.sleep(5)  # see it in action
    # _, final_pos = sim.simxGetObjectPosition(clientID=client_id, objectHandle=bot_base, relativeToObjectHandle=-1,
    #                                          operationMode=sim.simx_opmode_blocking)
    # print("Distance moved =", np.subtract(final_pos, initial_pos))


def task_3_primary(client_id, target_points):
    """
    Purpose:
    ---

    # NOTE:This is the only function that is called from the main function and from the executable.

    Make sure to call all the necessary functions (apart from the ones called in the main) according to your logic.
    The bot should traverse all the target navigational co-ordinates.

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

    wheel_joints = init_setup(client_id)
    # print("wheel_joints:", wheel_joints)  # debugging only

    # testing
    # vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    # view_image(transformed_image)  # debugging only
    # qr_codes = detect_qr_codes(transformed_image)
    # print("qr_codes:", qr_codes)  # debugging only
    # joints_position = encoders(client_id)
    # print("joints_position:", joints_position)  # debugging only

    # for i in range(len(wheel_joints)):
    #     _, position = sim.simxGetJointPosition(client_id, wheel_joints[i], sim.simx_opmode_blocking)
    #     if not math.isclose(position, joints_position[i], abs_tol=0.01):  # verify that we are using the correct joints
    #         print("Something is wrong...", position, joints_position[i])

    # nav_logic(client_id, target_points, wheel_joints, straight_line_movement=False)

    # vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id)
    # transformed_image = transform_vision_sensor_image(vision_sensor_image, image_resolution)
    # view_image(transformed_image)  # debugging only
    # qr_codes = detect_qr_codes(transformed_image)
    # print("qr_codes:", qr_codes)  # debugging only

    # Function ends here
