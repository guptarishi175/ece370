import RPi.GPIO as GPIO
import time
import sys

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

def motorControl(motor, wheel_directon):
	for halfstep in range(8):
		for pin in range(4):
			GPIO.output(motor[pin], wheel_direction[halfstep][pin])
		time.sleep(0.001)
GPIO.cleanup()
