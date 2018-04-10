import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Vector3
import sys

pub_data = Vector3()

GPIO.setmode(GPIO.BOARD)

rightmotor = [7, 11, 13, 15]
leftmotor = [29, 31, 33, 35]

for pin in rightmotor:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)

for pin in leftmotor:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)

kp = 1

rightmotor_forward = [ [1,0,0,0],
	[1,1,0,0],
	[0,1,0,0],
	[0,1,1,0],
	[0,0,1,0],
	[0,0,1,1],
	[0,0,0,1],
	[1,0,0,1] ]

leftmotor_forward = [ [0,0,0,1],
	[0,0,1,1],
	[0,0,1,0],
	[0,1,1,0],
	[0,1,0,0],
	[1,1,0,0],
	[1,0,0,0],
	[1,0,0,1] ]

rightmotor_backward = leftmotor_forward
leftmotor_backward = rightmotor_forward

def motorControl(motor, wheel_direction):
	for i in range(64):	
		for halfstep in range(8):
			for pin in range(4):
				GPIO.output(motor[pin], wheel_direction[halfstep][pin])
			time.sleep(0.001)

def moveForward():
	for i in range(64):
		for halfstep in range(8):
			for pin in range(4):
				GPIO.output(rightmotor[pin], rightmotor_forward[halfstep][pin])
				GPIO.output(leftmotor[pin], leftmotor_forward[halfstep][pin])
			time.sleep(0.001)

def doControl(des, act):
	global kp
	e = des - act
	return e*kp

def callback(data):
	global pub_data
	pub_data = data
	rospy.loginfo("x = " + str(data.x) + " y = " + str(data.y))

def listener():
	global pub_data
	rospy.init_node('camera_listener', anonymous = True)
	rospy.Subscriber('centerCoord', Vector3, callback)
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		rospy.loginfo("--- OLD --- x = " + str(pub_data.x) + " y = " + str(pub_data.y))
		err = doControl(0.0, pub_data.x)
		if err <= 1.0 and err >= -1.0:
			moveForward()
		if err > 1.0 :
			motorControl(rightmotor, rightmotor_forward)
		if err < -1.0 :
			motorControl(leftmotor, leftmotor_forward)
			
		rate.sleep()


if __name__ == '__main__':
	listener()


GPIO.cleanup()
