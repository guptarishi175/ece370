import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BOARD)

ControlPins = [7, 11, 13, 15]
ControlPins_2 = [29, 31, 33, 35]

for pin in ControlPins:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)

for pin in ControlPins_2:
	GPIO.setup(pin, GPIO.OUT)
	GPIO.output(pin, 0)



backwards = [ [1,0,0,0],
	[1,1,0,0],
	[0,1,0,0],
	[0,1,1,0],
	[0,0,1,0],
	[0,0,1,1],
	[0,0,0,1],
	[1,0,0,1] ]

forwards = [ [1,0,0,1],
	[1,0,0,0],
	[1,1,0,0],
	[0,1,0,0],
	[0,1,1,0],
	[0,0,1,0],
	[0,0,1,1],
	[0,0,0,1] ]

while True:
	for i in range(512):

		for halfstep in range(8):
			for pin in range(4):
				GPIO.output(ControlPins[pin], forwards[halfstep][pin])
			time.sleep(0.001)
GPIO.cleanup()
