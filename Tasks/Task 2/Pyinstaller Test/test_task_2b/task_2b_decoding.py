'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*
*  This script is intended to check the output of Task 2B
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			task1c_decoding.py
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
    f = open("task_2b_result.txt", "r")
    content = f.readlines()

    # Extracting team no and score
    i = 1
    for x in content:
        if   i == 1:
            team_id = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 2:
            dt = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 3:
            platform = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 4:
            mac = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 5:
            eval_no_of_joints = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 6:
            eval_vol = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 7:
            torque = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 8:
            force = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 9:
            individual_values = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 10:
            eval_rtf = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 11:
            eval_all_dynamics = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 12:
            eval_mass = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 13:
            no_of_joints = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 14:
            volume = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 15:
            rtf = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 16:
            mass = cryptocode.decrypt(x, "13_madison_kingdom")
        elif i == 17:
            dynamically_not_enabled_list = cryptocode.decrypt(x, "13_madison_kingdom")
        i += 1

    print("Team no                         : ", team_id)
    print("Date and Time                   : ", dt)
    print("Platform                        : ", platform)
    print("Mac                             : ", mac)
    print("eval_no_of_joints               : ", eval_no_of_joints)
    print("eval_vol                        : ", eval_vol)
    print("Torque                          : ", torque)
    print("Force                           : ", force)
    print("Individual values               : ", individual_values)
    print("eval_rtf                        : ", eval_rtf)
    print("eval_all_dynamics               : ", eval_all_dynamics)
    print("eval_mass                       : ", eval_mass)
    print("No. of joints                   : ", no_of_joints)
    print("Volume                          : ", volume)
    print("RTF                             : ", rtf)
    print("Mass                            : ", mass)
    print("Dynamically not enabled objects : ", dynamically_not_enabled_list)
    