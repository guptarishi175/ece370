from picamera.array import PiRGBArray
from picamera import PiCamera
from geometry_msgs.msg import Vector3

import time
import cv2
import numpy as np
import rospy

def getCenter(mask):
	M = cv2.moments(mask)
	if M["m00"] == 0:
		M["m00"] = 1

	x = int(M['m10']/M['m00'])
	y = int(M['m01']/M['m00'])
	theOut = (x,y)
	return theOut

def getColor(img, lowerBound, upperBound):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(img, lowerBound, upperBound)
	return mask



def getFOV(xPrime, yPrime, FOVw, FOVh):
	xDeg = xPrime*FOVw/2.0
	yDeg = yPrime*FOVh/2.0
		
	return (xDeg, yDeg)

def normalize(mask, x, y):
	H, W = mask.shape[:2]
	dW = 2.0/W
	dH = 2.0/H
	
	xPrime = (x-W/2.0)*dW
	yPrime = -(y-H/2.0)*dH

	return (xPrime, yPrime)


def talker():
	pub = rospy.Publisher('centerCoord', Vector3, queue_size=10)
	rospy.init_node('camera', anonymous=True)
	while not rospy.is_shutdown():
		center_coord_msg = Vector3()
		center_coord_msg.x = 

cap = cv2.VideoCapture(0)

lowerbound = np.array([105,100,100], np.uint8)
upperbound = np.array([127,255,255], np.uint8)

while(True):
	tick = time.time()
	
	ret, frame = cap.read()	

	imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(imgHSV,lowerbound,upperbound)

	(cX, cY) = getCenter(mask)

	(xPrime, yPrime) = normalize(mask, cX, cY)
	
	FOV = getFOV(xPrime,yPrime, 62.2, 48.8)
	
	cv2.circle(frame,(cX,cY),50,(0,0,255),-1)

	print (cX, cY)

	print FOV

	cv2.imshow("Frame", frame)
	
	tock = time.time()

	print "Time to complete = " + str(tock-tick) + " sec"

	cv2.waitKey(1)

