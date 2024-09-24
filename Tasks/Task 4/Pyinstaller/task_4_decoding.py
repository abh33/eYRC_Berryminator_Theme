'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*
*  This script is intended to decode the output of Task 4
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			task_4_decoding.py
*  Created:				
*  Last Modified:		
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
'''

import cryptocode

if __name__ == "__main__":
    f = open("task_4_result.txt", "r")
    content = f.readlines()

    # Extracting team no and score
    i = 1
    for x in content:
        if   i == 1:
            team_id = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 2:
            dt = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 3:
            platform = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 4:
            mac = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 5:
            ci = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 6:
            cp = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 7:
            cd = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 8:
            collisions = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i==9:
            mass_of_arm=cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 10:
            path = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 11:
            output_list_child = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 12:
            eval_rtf_python = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 13:
            end_simulation_time = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 14:
            init_real_time = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 15:
            end_real_time = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 16:
            rtf_python = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 17:
            eval_all_dynamics = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 18:
            eval_no_of_joints = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 19:
            no_of_joints = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 20:
            torque = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 21:
            force = cryptocode.decrypt(x, "?8G]tzGW{T")
        elif i == 22:
            individual_values = cryptocode.decrypt(x, "?8G]tzGW{T")
        i += 1

    print("Team no                         : ", team_id)
    print("Date and Time                   : ", dt)
    print("Platform                        : ", platform)
    print("Mac                             : ", mac)
    print("Correctly Identified            : ", ci)
    print("Correctly Plucked               : ", cp)
    print("Correctly Dropped               : ", cd)
    print("Collisions                      : ", collisions)
    print("Mass of Arm                     : ", mass_of_arm)
    print("Path                            : ", path)
    print("output_list_child               : ", output_list_child)
    print("eval rtf                        : ", eval_rtf_python)
    print("Total simulation time           : ", end_simulation_time)
    print("Init real time                  : ", init_real_time)
    print("End real time                   : ", end_real_time)
    print("RTF from Python                 : ", rtf_python)
    print("Eval all dynamics               : ", eval_all_dynamics)
    print("Eval no of joints               : ", eval_no_of_joints)
    print("No. of joints                   : ", no_of_joints)
    print("Torque                          : ", torque)
    print("Force                           : ", force)
    print("Indivdual Values                : ", individual_values)