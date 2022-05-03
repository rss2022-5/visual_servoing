
import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	lower_orange = (0, 140, 190)
	upper_orange = (25,255,255)
	#ones that didn't work
	#41 240 247 (hit the red)
	#these were ok...
	# lower_orange = (5,160,50)
	#upper_orange = (15,255,255)
	mask = cv2.inRange(hsv, lower_orange, upper_orange)
	mask_and = cv2.bitwise_and(hsv, hsv, mask = mask)
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))	

	# Erode then dilate
	result_erode = cv2.erode(mask_and, kernel)
	result_dilate = cv2.dilate(result_erode, kernel)
	inter_result = cv2.cvtColor(result_dilate, cv2.COLOR_HSV2BGR)
	result = cv2.cvtColor(inter_result, cv2.COLOR_BGR2GRAY)
	img2, contours, hierarchy = cv2.findContours(result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) != 0:
    		# find the biggest countour (c) by the area
    		c = max(contours, key = cv2.contourArea)
    		x,y,w,h = cv2.boundingRect(c)
	else:
		x =y=w=h  = 0
	bounding_box = ((x,y),(x+w,y+h))
	cv2.rectangle(result, (x,y), (x+w,y+h), (255,255,255), 4)
	########### YOUR CODE ENDS HERE ##########
	# Return bounding box
	return bounding_box
