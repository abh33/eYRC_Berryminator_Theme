'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*
*  This script is intended to decode the output of Task 6
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			theme_implementation_result_decoding.py
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
    f = open("theme_implementation_result.txt", "r")
    content = f.readlines()

    # Extracting team no and score
    i = 1
    for x in content:
        if   i == 1:
            team_id = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 2:
            dt = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 3:
            platform = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 4:
            mac = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 5:
            T = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 6:
            CI = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 7:
            CP = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 8:
            CD = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 9:
            P = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 10:
            B = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 11:
            score = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 12:
            final_ci_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 13:
            final_cp_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 14:
            final_cd_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 15:
            final_cb1_drops = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 16:
            final_cb2_drops = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")

        elif i == 17:
            raw_ci_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 18:
            raw_cp_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 19:
            raw_dropped_in_cb1_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 20:
            raw_dropped_in_cb2_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 21:
            raw_collisions_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 22:
            mass_of_arm = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 23:
            path = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")



        elif i == 24:
            eval_rtf_python = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 25:
            end_simulation_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 26:
            init_real_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 27:
            end_real_time = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 28:
            rtf_python = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 29:
            eval_all_dynamics = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 30:
            eval_no_of_joints = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 31:
            no_of_joints = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 32:
            torque = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 33:
            force = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 34:
            individual_values = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 35:
            dynamically_not_enabled_list = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 36:
            valid_run_flag = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 37:
            time_to_substract = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")    
        elif i == 38:
            output_list_child = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")
        elif i == 39:
            output_list_custom = cryptocode.decrypt(x, "s}ZYmS{:QgMx'9Qd")

        i += 1

    print("Team no                         : ", team_id)
    print("Date and Time                   : ", dt)
    print("Platform                        : ", platform)
    print("Mac                             : ", mac)
    print("VALID Run?                      : ", valid_run_flag)
    print("T                               : ", T)
    print("CI                              : ", CI)
    print("CP                              : ", CP)
    print("CD                              : ", CD)
    print("P                               : ", P)
    print("B                               : ", B)
    print("score                           : ", score)
    print("Final CI List                   : ", final_ci_list)
    print("Final CP List                   : ", final_cp_list)
    print("Final CD List                   : ", final_cd_list)
    print("Final CB1 Drops                 : ", final_cb1_drops)
    print("Final CB2 Drops                 : ", final_cb2_drops)
    print()
    print("Raw CI List                     : ", raw_ci_list)
    print("Raw CP List                     : ", raw_cp_list)
    print("Raw Dropped in CB1 List         : ", raw_dropped_in_cb1_list)
    print("Raw Dropped in CB2 List         : ", raw_dropped_in_cb2_list)
    print("Collisions                      : ", raw_collisions_list)
    print("Mass of Arm                     : ", mass_of_arm)
    print("Path                            : ", path)
    print("eval rtf                        : ", eval_rtf_python)
    print("Total simulation time           : ", end_simulation_time)
    print("Time to substract               : ", time_to_substract)
    print("Init real time                  : ", init_real_time)
    print("End real time                   : ", end_real_time)
    print("RTF from Python                 : ", rtf_python)
    print("Eval all dynamics               : ", eval_all_dynamics)
    print("Eval no of joints               : ", eval_no_of_joints)
    print("No. of joints                   : ", no_of_joints)
    print("Torque                          : ", torque)
    print("Force                           : ", force)
    print("Indivdual Values                : ", individual_values)
    print("Dyn not enabled list            : ", dynamically_not_enabled_list)

    print("output_list_child               : ", output_list_child)
    print("output_list_custom              : ", output_list_custom)