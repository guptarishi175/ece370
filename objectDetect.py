from picamera.array import PiRGBArray
from picamera import PiCamera
from geometry_msgs.msg import Vector3
import sys
import time
import cv2
import numpy as np
import rospy
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

rightmotor = [7,11,13,15]
leftmotor = [29, 31, 33, 35]

kp = 1

for pin in rightmotor:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)
for pin in leftmotor:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)

rightMotor_forward = [[1,0,0,0],
	[1,1,0,0],
	[0,1,0,0],
	[0,1,1,0],
	[0,0,1,0],
	[0,0,1,1],
	[0,0,0,1],
	[1,0,0,1]]

rightMotor_backward = [[0,0,0,1],
	[0,0,1,1],
	[0,0,1,0],
	[0,1,1,0],
	[0,1,0,0],
	[1,1,0,0],
	[1,0,0,0],
	[1,0,0,1]]

leftMotor_forward = [[0,0,0,1],
	[0,0,1,1],
	[0,0,1,0],
	[0,1,1,0],
	[0,1,0,0],
	[1,1,0,0],
	[1,0,0,0],
	[1,0,0,1]]

leftMotor_backward = [[1,0,0,0],
	[1,1,0,0],
	[0,1,0,0],
	[0,1,1,0],
	[0,0,1,0],
	[0,0,1,1],
	[0,0,0,1],
	[1,0,0,1]]

def moveForward():
	rightMotor_control(rightMotor_forward)
	leftMotor_control(leftMotor_forward)

def moveBackward():
	rightMotor_control(rightMotor_backward)
	leftMotor_control(leftMotor_backward)

def turnRight():
	rightMotor_control(rightMotor_forward)

def turnLeft():
	leftMotor_control(leftMotor_forward)

def reverseLeft():
	rightMotor_control(rightMotor_backward)

def reverseRight():
	leftMotor_control(leftMotor_backward)

def scan_circle():
	rightMotor_control(rightMotor_forward)
	leftMotor_control(leftMotor_backward)

def rightMotor_control(sequence):
	while True:
		for i in range(512):
			for halfstep in range(8):
				for pin in range(4):
					GPIO.output(rightmotor[pin], sequence[halfstep][pin])
			time.sleep(0.001)

def leftMotor_control(sequence):
	while True:
		for i in range(512):
			for halfstep in range(8):
				for pin in range(4):
					GPIO.output(leftmotor[pin], sequence[halfstep][pin])
			time.sleep(0.001)


def doControl(thetaDis, theta):
	e = thetaDis - theta
	return e*kp

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
		(xDeg, yDeg) = getImgCenter()
		center_coord_msg = Vector3()
		center_coord_msg.x = xDeg
		center_coord_msg.y = yDeg
		rospy.loginfo(center_coord_msg)
		pub.publish(center_coord_msg)

def getImgCenter():

	ret, frame = cap.read()	

	imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(imgHSV,lowerbound,upperbound)

	(cX, cY) = getCenter(mask)

	(xPrime, yPrime) = normalize(mask, cX, cY)
	
	(xDeg, yDeg) = getFOV(xPrime,yPrime, 62.2, 48.8)
	
	return (xDeg, yDeg)

cap = cv2.VideoCapture(0)
lowerbound = np.array([105, 100, 100], np.uint8)
upperbound = np.array([127, 255, 255], np.uint8)
	




if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
