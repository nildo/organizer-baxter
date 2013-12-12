#!/usr/bin/env python

import roslib; roslib.load_manifest('organizer')
import rospy
import iodevices
import time
import cv, cv2
import numpy as np

from homography import *
from baxter import *

# Global variables:

H = []	# The current homography matrix.
Z = 0	# The Z coordinate of the table.
baxter = Baxter() # The object that controls Baxter. Defined in baxter.py.
floor_reference_points = [] # The floor reference points.
floor_reference_orientations = [] # The floor reference orientations.
n_clicks = 0
tot_clicks = 4
points = []

original_position = None
current_position = None



def initial_setup_baxter():
	"""
	Enable and set up baxter.

	"""
	
	print 'Initializing node...'
	rospy.init_node('organizer')
	baxter.enable()
	baxter.calibrateLeftGripper()

def initial_setup_camera():
	"""
	Setup camera

	"""
	pass
	
def on_mouse_click(event, x, y, flag, param):
	global n_clicks, points
	if event == cv.CV_EVENT_FLAG_LBUTTON:
		print 'Point %s captured: (%s,%s)' % (n_clicks+1,x,y)
		points.append([x, y])
		n_clicks += 1

def get_img_reference_points():
	"""
	This function get 4 points of reference from the image from the right hand
	of baxter. Returns an array of size 4, with 4 coordinates:
	[[x1,y1], [x2,y2], [x3,y3], [x4,y4]].

	TODO: implement this. We have to define a color we will mark the table
	and get 4 points of that color from the image.

	"""
	# The following line is just for test.
	raw_input('Enter to capture image.')
	image = baxter.getImageFromRightHandCamera()
	cvimage = baxter.getLastCvImage()
	while n_clicks <= tot_clicks-1:
		# displays the image
		cv.ShowImage("Click", cvimage)
		#calls the callback function "on_mouse_click'when mouse is clicked inside window
		cv.SetMouseCallback("Click", on_mouse_click, param=1)
		cv.WaitKey(1000)
		
	
	#print points
	return points

def get_floor_reference_points():
	"""
	This function get 4 points of reference from the real world, asking the
	user to move the baxter arm to the position of each corresponding point
	in the image, and then getting the X,Y and Z coordinates of baxter's hand.
	Returns an array of size 4 containing 4 coordinates:
	[[x1,y1], [x2,y2], [x3,y3], [x4,y4]].
	All the coordinates Z should be approximatelly the same. We assume the table
	is niveled. Save the Z coordinate in the global variable.

	TODO: Implement this. Figure out a way to get the end position of baxter
	hand. I know that in baxter_msgs

	"""
	global Z # This declaration is needed to modify the global variable Z
	global floor_reference_points # Maybe erase.
	global floor_reference_orientations # Maybe erase.
	
	raw_input('Move the LEFT arm to point 1 and press enter.')
	p1 = baxter.getLeftArmPosition()
	o1 = baxter.getLeftArmOrientation()
	print 'Point 1 =', p1
	raw_input('Move the LEFT arm to point 2 and press enter.')
	p2 = baxter.getLeftArmPosition()
	o2 = baxter.getLeftArmOrientation()
	print 'Point 2 =', p2
	raw_input('Move the LEFT arm to point 3 and press enter.')
	p3 = baxter.getLeftArmPosition()
	o3 = baxter.getLeftArmOrientation()
	print 'Point 3 =', p3
	raw_input('Move the LEFT arm to point 4 and press enter.')
	p4 = baxter.getLeftArmPosition()
	o4 = baxter.getLeftArmOrientation()
	print 'Point 4 =', p4
	
	floor_reference_points = [p1,p2,p3,p4]
	floor_reference_orientations = [o1,o2,o3,o4]
	
	# Calculate the z coordinate average:
	Z = (p1[2] + p2[2] + p3[2] + p4[2]) / 4
	#Z = -0.19733055364191465
	
	return [[p1[0],p1[1]], [p2[0],p2[1]], [p3[0],p3[1]], [p4[0],p4[1]]]
	#return [[0.5317777930389638, 0.35113395608632486],
	#[0.7861491312411216, 0.3585160199636554],
	#[0.7251350261395254, -0.07691071384877325],
	#[0.5336363208864047, -0.11315404861126654]]

def calibrate_homography():
	global H, Hinv
	floor_points = get_floor_reference_points()
	img_points = get_img_reference_points()
	H = homography_floor_to_img(img_points, floor_points)
	
def move_left_arm_to(point):
	origin = baxter.getLeftArmPosition()
	print 'origin =', origin
	if origin[2] < Z + 0.2:
		dest1 = origin[:]
		dest1[2] += 0.2
		print 'dest1 =', dest1
		baxter.moveLeftArm(dest1, floor_reference_orientations[0])
		print 'done dest1'
	if point[2] < Z + 0.1:
		dest2 = point[:]
		dest2[2] += 0.2
		print 'dest2 =', dest2
		baxter.moveLeftArm(dest2, floor_reference_orientations[0])
		print 'done dest2'
	print 'point =', point
	baxter.moveLeftArm(point, floor_reference_orientations[0])
	print 'done point'
	
def getthresholdedimg(im):
	'''
	this function take RGB image.Then convert it into HSV for easy colour
	detection and threshold it with red part as white and all other regions as
	black.Then return that image
	
		h1 = 165
		s1 = 50
		v1 = 200
		h2 = 175
		s2 = 200
		v2 = 255
	
	'''
	imghsv =cv.CreateImage(cv.GetSize(im),8,3)
	cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)
	imgred=cv.CreateImage(cv.GetSize(im),8,1)
	# EDIT THIS LINE
	cv.InRangeS(imghsv,cv.Scalar(165,50,200),cv.Scalar(175,255,255),imgred)
	#cv.ShowImage('Click', imgred)
	#cv.WaitKey()
	return imgred

def getthresholdedimg2(im,v):
	'''
	this function take RGB image.Then convert it into HSV for easy colour
	detection and threshold it with red part as white and all other regions as
	black.Then return that image
	
	'''
	imghsv =cv.CreateImage(cv.GetSize(im),8,3)
	cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)
	imgred=cv.CreateImage(cv.GetSize(im),8,1)
	# EDIT THIS LINE
	cv.InRangeS(imghsv,cv.Scalar(v[0],v[1],v[2]),cv.Scalar(v[3],v[4],v[5]),imgred)
	cv.ShowImage('Click', imgred)
	cv.WaitKey()
	return imgred

def get_object_position():
	baxter.getImageFromRightHandCamera()
	cvimage = baxter.getLastCvImage()
	frame_size = cv.GetSize(cvimage)

	cv.Smooth(cvimage,cvimage,cv.CV_GAUSSIAN, 3,0)
	imgredthresh = getthresholdedimg(cvimage)
	#cv.Erode(imgredthresh,imgredthresh,None,3)
	#cv.Dilate(imgredthresh,imgredthresh,None,10)
	storage = cv.CreateMemStorage(0)
	contour = cv.FindContours(imgredthresh,storage,cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
	
	if not contour:
		return None
	
	bpoints1 = []
	bpoints2 = []
	
	while contour:
		# Draw bounding rectangles
		bound_rect = cv.BoundingRect(list(contour))
		contour = contour.h_next()
		#print bound_rect
		# for more details about cv.BoundingRect,see documentation
		pt1 = [bound_rect[0], bound_rect[1]]
		pt2 = [bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3]]
		bpoints1.append(pt1)
		bpoints2.append(pt2)			

	z = np.array([])		
	
	if np.size(bpoints1) > 0:
		#bpoints1 is top left, bpoints 2 is bottom right
		pt1max = np.array([0,0])
		pt2max = np.array([0,0])

		for i in range(len(bpoints1)):
			maxdiag = np.linalg.norm(pt2max - pt1max)
	
			p1 = np.array([bpoints1[i][0],bpoints1[i][1]])
			p2 = np.array([bpoints2[i][0],bpoints2[i][1]])
			diag = np.linalg.norm(p2 - p1)
			if diag > maxdiag:
				pt1max = p1
				pt2max = p2
		
		# UPPER LEFT CORNER OF THE LARGEST RECTANGLE
		pt1 = pt1max
		# LOWER RIGHT CORNER OF THE LARGEST RECTANGLE
		pt2 = pt2max
		cv.Rectangle(cvimage,tuple(pt1),tuple(pt2), cv.CV_RGB(255,0,0), 1)
		# HORIZON COORDINATE OF THE LARGEST RECTANGLE
		centroidu = (pt1[0]+pt2[0])/2
		# VERTICAL COORDINATE OF THE LARGEST RECTANGLE
		centroidv = (pt1[1]+pt2[1])/2
		cv.Circle(cvimage,(centroidu,centroidv),5,0,-1)
		floor_point = pixel_to_floor(H,[centroidu,centroidv])
		cv.ShowImage("Click",cvimage)
		cv.WaitKey(1000)
		#cv2.destroyAllWindows()
		return floor_point + [Z]
	
def test():
	global n_clicks, points
	cvimage = baxter.getLastCvImage()
	while True:
		raw_input("Put the object and press enter.")
		position = get_object_position()
		print 'position =', position
		move_left_arm_to(position)
	
	
	"""while True:
		n_clicks = 0
		points = []
		while n_clicks == 0:
			# displays the image
			cv.ShowImage("Click", cvimage)
			#calls the callback function "on_mouse_click'when mouse is clicked inside window
			cv.SetMouseCallback("Click", on_mouse_click, param=1)
			cv.WaitKey(1000)
	
		img_point = points[0]
		floor_point = pixel_to_floor(H, img_point)
		floor_point += [Z]
		move_left_arm_to(floor_point)"""
	
	
def test2():
	for i in range(60,240,10):
		h1 = 165
		s1 = 50
		v1 = 200
		h2 = 175
		s2 = 200
		v2 = 255
		print h1, s1, v1
		print h2, s2, v2
		print
		baxter.getImageFromRightHandCamera()
		cvimage = baxter.getLastCvImage()
		frame_size = cv.GetSize(cvimage)
		cv.Smooth(cvimage,cvimage,cv.CV_GAUSSIAN, 3,0)
		imgredthresh = getthresholdedimg2(cvimage,[h1,s1,v1,h2,s2,v2])
	
def wait_modification():
	"""
	Implement a function that will wait a modification in the camera image.

	"""
	pass

def grab_in_current_position():
	move_left_arm_to(current_position)
	baxter.closeLeftGripper()
	
def release_in_original_position():
	move_left_arm_to(original_position)
	baxter.openLeftGripper()
	rest_position = original_position[:]
	rest_position[2] += 0.3
	baxter.moveLeftArm(rest_position, floor_reference_orientations[0])

def organize():
	"""
	Implement a function that will organize the disorganized environment.

	"""
	if not current_position or not original_position:
		return
	
	print 'current =', current_position
	grab_in_current_position()
	print 'original =', original_position
	release_in_original_position()

def disable_baxter():
	"""
	Disable Baxter.

	"""
	baxter.disable()


def get_original_positions():
	global original_position
	while not original_position:
		raw_input('Put the object in the original position and press Enter.')
		original_position = get_object_position()
		if not original_position:
			print 'Object not found.'
		
def equal_positions():
	delta = 0.1
	if original_position and current_position:
		op = np.array(original_position)
		cp = np.array(current_position)
		vector = op - cp
		dist = np.linalg.norm(vector)
		if dist > delta:
			return False
		else:
			return True
	else:
		return True
	
def check_positions():
	global current_position
	current_position = get_object_position()
	if current_position:
		return equal_positions()
	else:
		# Did not find object. Just return true.
		return True
	

def main():
	print 'Baxter Organizer initiated.'
	
	initial_setup_baxter()
	initial_setup_camera()
	calibrate_homography()
	get_original_positions()
	print 'Now you can move the object at any time and wait.'
	while True:
		print 'Checking objects.'
		if not check_positions():
			print 'What a mess! >:('
			organize()
		else:
			print 'Everything ok! :)'
		time.sleep(10)
	
	done = False
	print 'Esc to exit.'
	while not done and not rospy.is_shutdown():
		c = iodevices.getch()
		if c:
			# Get 'esc' or 'ctrl+c'
			if c in ['\x1b', '\x03']:
				done = True
		
	#disable_baxter()

	print 'Baxter Organizer finished.'

if __name__ == '__main__':
	main()
