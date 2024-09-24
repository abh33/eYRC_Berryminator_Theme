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

# Team ID:			[ BM_1115 ]
# Author List:		[ Rabbi Sudheer Zacharias,D S Sai Rohith,Nisarga B, Ramya ]
# Filename:			task_1a.py
# Functions:		detect_shapes
# 					[find_centroid, find_shapes, find_colour ]


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
detected_shapes=list()
def find_centroid(image,detected_shapes):
	thresh=cv2.bitwise_not(image)
	contours,hierarchies=cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	a=0
	for i in contours:
		M=cv2.moments(i)
		if M['m00']!=0:
			cx=int(M['m10']/M['m00'])
			cy=int(M['m01']/M['m00'])
			b=0
			for e in detected_shapes:
				if a==b:
					if len(e)<3:
						e.append((cx,cy))
						b+=1

def find_shapes(gray,u,detected_shapes):  	
 
 _, thrash = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
 contours, _ = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
 i=0
 for contour in contours:
     approx = cv2.approxPolyDP(contour, 0.0255* cv2.arcLength(contour, True), True)
     cv2.drawContours(gray, [approx], 0, (0, 0, 0), 2)
     rect = cv2.minAreaRect(contour)
	 # Creates box around that rectangle
     box = cv2.boxPoints(rect)
		# Not exactly sure
     box = np.int0(box)
		# Gets center of rotated rectangle
     center = rect[0]
		# Gets rotation of rectangle; same as rotation of contour
     rotation = rect[2]
		# Gets width and height of rotated rectangle
     width = rect[1][0]
     height = rect[1][1]
		# Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
     #rotation = translateRotation(rotation, width, height)
     #x = approx.ravel()[0]
     #y = approx.ravel()[1] - 5

     #print(" In Find Shapes")
     #print(approx,rect,box,center,rotation,width,height)

	 
	 
     #if len(approx) == 3:
      #   s1='Triangle'
        # if i>0:
         #    detected_shapes.append([u,s1])

     if len(approx) == 4 and 330>width>270 and 330>height>270:
         x1 ,y1, w, h = cv2.boundingRect(approx)

         aspectRatio = float(w/h)
         #print(aspectRatio)
		 #if aspectRatio >= 0.95 and aspectRatio <= 1.05 and w>60:
         if  aspectRatio >= 0.95 and aspectRatio <= 1.05 :
             s2='Square'
             #if i>0:
            # print(" In Square")
             #print("   ")
			#print(approx,rect,box,center,rotation,width,height)
             #print(rotation)
             detected_shapes.append([rotation,rotation])
             return rotation
        
         else:
             s3='Rectangle'
            # if i>0:
             print(" In Rectangle")
             print("   ")
             print(rotation)
             detected_shapes.append([rotation,rotation])
             return rotation
            # i+=1    
             
     elif len(approx) == 5:
         s4='Pentagon'
         #if i>0:
            # detected_shapes.append([u,s4])
             
     elif len(approx)>=6:
         s5='Circle'
         #if i>0:
          #   detected_shapes.append([u,s5])

def find_colour(img1,detected_shapes):
 
 font = cv2.FONT_HERSHEY_COMPLEX
 detected_shapes.clear()
 
 hsv=cv2.cvtColor(img1,cv2.COLOR_BGR2HSV)

 lower_range_green = np.array([30,50,30])
 upper_range_green = np.array([89,255,255])
 lower_range_blue = np.array([90,50,70])
 upper_range_blue = np.array([128,255,255])
 lower_range_red = np.array([0,50,50])
 upper_range_red = np.array([10,255,255])
 lower_range_orange = np.array([10,100,20])
 upper_range_orange = np.array([25,255,255])

 #lower_range_black = np.array([255,255,255])
 #upper_range_black = np.array([225,220,220])

 lower_range_black = np.array([0,0,0])
 upper_range_black = np.array([72,72,72])

 grn=cv2.inRange(hsv, lower_range_green, upper_range_green)
 blue=cv2.inRange(hsv,lower_range_blue, upper_range_blue)
 red=cv2.inRange(hsv, lower_range_red,upper_range_red)
 orange=cv2.inRange(hsv,lower_range_orange,upper_range_orange)
 black=cv2.inRange(hsv,lower_range_black,upper_range_black)
 #resg = cv2.bitwise_and(img,img, mask= grn)
 #resb=cv2.bitwise_and(img,img, mask= blue)

 hasGreen = np.sum(grn)
 hasBlue = np.sum(blue)
 hasRed = np.sum(red)
 hasOrange = np.sum(orange)
 hasBlack = np.sum(black)
 

 if hasGreen>0:
     u1='Green'
     gray_g=cv2.bitwise_not(grn)
     
     find_shapes(gray_g,u1,detected_shapes)
     find_centroid(grn,detected_shapes)


 if hasBlue>0:
     u2='Blue'
     gray_b=cv2.bitwise_not(blue)
    
     find_shapes(gray_b,u2,detected_shapes)
     find_centroid(blue,detected_shapes)

 if hasRed>0:
     u3='Red'
     gray_r=cv2.bitwise_not(red)
    
     find_shapes(gray_r,u3,detected_shapes)
     find_centroid(red,detected_shapes)

 if hasOrange>0:
     u4='Orange'
     #gray_o=cv2.bitwise_not(orange)
	 
     #find_shapes(gray_o,u4,detected_shapes)  
     #find_centroid(orange,detected_shapes)      


 if hasBlack>0:
     u4='Black'
	 
     gray_o=cv2.bitwise_not(black)
    
	 #cv2.imshow(black)
     rotation=find_shapes(gray_o,u4,detected_shapes) 
	  
     find_centroid(black,detected_shapes) 
	 
     return rotation





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

	rotation=find_colour(img,detected_shapes)
	 


	##################################################
	
	return rotation

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
		
		
		if len (detected)==4:
			coordinates = detected[3]
			cv2.putText(img, str((colour, shape,detected[2])),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
		else:
			coordinates = detected[2]
			cv2.putText(img, str((colour, shape)),coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)	
	return img

if __name__ == '__main__':
	
	# path directory of images in 'test_images' folder
	img_dir_path = 'test_images/'

	# path to 'test_image_1.png' image file
	file_num = 1
	img_file_path = img_dir_path + 'test_image_' + str(file_num) + '.png'
	
	# read image using opencv
	img = cv2.imread(img_file_path)
	
	print('\n============================================')
	print('\nFor test_image_' + str(file_num) + '.png')
	
	# detect shape properties from image
	angel = detect_shapes(img)
	#print(detected_shapes)
	print(angel)
	
	# display image with labeled shapes
	img = get_labeled_image(img, detected_shapes)
	cv2.imshow("labeled_image", img)
	cv2.waitKey(20000)
	cv2.destroyAllWindows()
	
	choice = input('\nDo you want to run your script on all test images ? => "y" or "n": ')
	
	if choice == 'y':

		for file_num in range(1, 16):
			
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


