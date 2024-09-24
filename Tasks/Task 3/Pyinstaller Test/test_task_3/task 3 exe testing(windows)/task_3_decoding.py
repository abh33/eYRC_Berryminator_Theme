'''
*****************************************************************************************
*
*        =================================================
*             Berryminator (BM) Theme (eYRC 2021-22)
*        =================================================
*
*  This script is intended to check the output of Task 3
*  of Berryminator (BM) Theme (eYRC 2021-22).
*
*  Filename:			task_3_decoding.py
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
    f = open("task_3_result.txt", "r")
    content = f.readlines()

    # Extracting team no and score
    i = 1
    for x in content:
        if   i == 1:
            team_id = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 2:
            dt = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 3:
            platform = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 4:
            mac = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 5:
            random_points = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 6:
            score = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 7:
            path = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 8:
            eval_rtf_python = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 9:
            end_simulation_time = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 10:
            init_real_time = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 11:
            end_real_time = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        elif i == 12:
            rtf_python = cryptocode.decrypt(x, "SKDFc?rt=5_X9jb2")
        i += 1

    print("Team no                         : ", team_id)
    print("Date and Time                   : ", dt)
    print("Platform                        : ", platform)
    print("Mac                             : ", mac)
    print("Random points                   : ", random_points)
    print("Score                           : ", score)
    print("Path                            : ", path)
    print("eval rtf                        : ", eval_rtf_python)
    print("Total simulation time           : ", end_simulation_time)
    print("Init real time                  : ", init_real_time)
    print("End real time                   : ", end_real_time)
    print("RTF from Python                 : ", rtf_python)