'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 1A of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_1a.py
# Functions:		scan_image
# 					[ Comma separated list of functions in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the three available ##
## modules for this task (numpy, opencv, os)                ##
##############################################################
import cv2
import numpy as np
import os
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################
#length of sides of 4 sided polygon
def quad(a,b):
	dist = ( (((a[0]-b[0])**2)+((a[1]-b[1])**2))**0.5 )
	return (round(dist))

#slope of sides of 4 sided polygon
def slope(a,b):
	if((b[0]-a[0])==0):
		slope='not defined'
		angle = 90
	else:
		slope=((b[1]-a[1])/(b[0]-a[0]))
		angle = (np.arctan(slope)*180)/np.pi
	return(round(angle))





##############################################################

def detect_shapes(img):

	"""
	Purpose:
	---
	This function takes the image as an argument and returns a nested list
	containing details of colored (non-white) shapes in that image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	Returns:
	---
	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image
	
	Example call:
	---
	shapes = detect_shapes(img)
	"""    
	detected_shapes = []

	##############	ADD YOUR CODE HERE	##############
	
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	
	_,thresh = cv2.threshold(gray,220,255,cv2.THRESH_BINARY)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours = contours[1:]
		
	properties = []
	
	for cnt in contours:        
#shape       
		epsilon = 0.03*cv2.arcLength(cnt,True)
		approx = cv2.approxPolyDP(cnt,epsilon,True)
		
		if len(approx)==3:
			shape = 'Triangle'
		
		elif len(approx)==4:
			a=quad(approx[0][0],approx[1][0])
			b=quad(approx[1][0],approx[2][0])
			c=quad(approx[2][0],approx[3][0])
			d=quad(approx[3][0],approx[0][0])
		
			e=slope(approx[0][0],approx[1][0])
			f=slope(approx[1][0],approx[2][0])
			g=slope(approx[3][0],approx[2][0])
			h=slope(approx[0][0],approx[3][0])
			
			if (int(e) in range(int(g)-2,int(g)+3)) and (int(f) in range(int(h)-2,int(h)+3)):
				if (int(a) in range(int(b)-2,int(b)+3)) and (int(b) in range(int(c)-2,int(c)+3)) and (int(c) in range(int(d)-2,int(d)+3)):
					shape = 'Square'
				elif (int(a) in range(int(c)-2,int(c)+3)) and (int(b) in range(int(d)-2,int(d)+3)) and (int(a) != int(b)):
					shape = 'Rectangle'
			
			
		elif len(approx)==5:
			shape = 'Pentagon'
			
		else:
			shape = 'Circle' 
			
#colour
		M = cv2.moments(cnt)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		
		px=img[cY,cX]
		colr = np.uint8([[px]])
		
		hsv_colr = cv2.cvtColor(colr,cv2.COLOR_BGR2HSV)
		h_value=hsv_colr[0][0][0]
		
		if (h_value>=110 and h_value<=130):
			colour = 'Blue'
		elif (h_value>=50 and h_value<=70):
			colour = 'Green'
		elif ((h_value>=0 and h_value<=10) or (h_value>=170 and h_value<=179) ):
			colour = 'Red'
		elif (h_value>=10 and h_value<=25):
			colour = 'Orange'
		else:
			colour = 'None'
			
		shape_properties = [colour, shape, (cX, cY)]
			
#label and save
#         cv2.drawContours(img, contours, -1, (0, 0, 0), 3)
#         cv2.putText(img, colour+'-'+shape,(cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
#         cv2.imwrite(r'C:\Users\shyam\Downloads\test_images\lab'+img_name, img)
#output
#         properties.append(colour+'-'+shape)
		detected_shapes.append(shape_properties)	
	
	 


	##################################################
	
	return detected_shapes

def get_labeled_image(img, detected_shapes):
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########
	"""
	Purpose:
	---
	This function takes the image and the detected shapes list as an argument
	and returns a labelled image

	Input Arguments:
	---
	`img` :	[ numpy array ]
			numpy array of image returned by cv2 library

	`detected_shapes` : [ list ]
			nested list containing details of colored (non-white) 
			shapes present in image

	Returns:
	---
	`img` :	[ numpy array ]
			labelled image
	
	Example call:
	---
	img = get_labeled_image(img, detected_shapes)
	"""
	######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS FUNCTION #########    

	for detected in detected_shapes:
		colour = detected[0]
		shape = detected[1]
		coordinates = detected[2]
		cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
	return img

if __name__ == '__main__':
	
	# path directory of images in 'test_images' folder
	img_dir_path = 'all_test_images/'

	# path to 'test_image_1.png' image file
	file_num = 1
	img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
	
	# read image using opencv
	img = cv2.imread(img_file_path)
	
	print('\n============================================')
	print('\nFor test_image_' + str(file_num) + '.png')
	
	# detect shape properties from image
	detected_shapes = detect_shapes(img)
	print(detected_shapes)
	
	# display image with labeled shapes
	img = get_labeled_image(img, detected_shapes)
	cv2.imshow("labeled_image", img)
	cv2.waitKey(2000)
	cv2.destroyAllWindows()
	
	choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
	
	if choice == 'y':

		for file_num in range(1, 15):
			
			# path to test image file
			img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
			
			# read image using opencv
			img = cv2.imread(img_file_path)
	
			print('\n============================================')
			print('\nFor test_image_' + str(file_num) + '.png')
			
			# detect shape properties from image
			detected_shapes = detect_shapes(img)
			print(detected_shapes)
			
			# display image with labeled shapes
			img = get_labeled_image(img, detected_shapes)
			cv2.imshow("labeled_image", img)
			cv2.waitKey(2000)
			cv2.destroyAllWindows()


