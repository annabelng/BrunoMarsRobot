import readchar
import sys
import time
import RPi.GPIO as GPIO

import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685

"""
mode=GPIO.getmode()
GPIO.cleanup()
Forward=13
Backward=12
Motor=11
"""

Forward=18
Backward=27
Motor=17
sleeptime=1

left_spd = 100
right_spd = 100

line_pin_right = 19
line_pin_middle = 16
line_pin_left = 20

right45deg = 0.5
left45deg = -0.5
straightahead = 0.0

global pwm_B


#def setup():
GPIO.cleanup()
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
hat.frequency = 60
GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)
GPIO.setup(Motor, GPIO.OUT)
GPIO.setup(line_pin_right,GPIO.IN)
GPIO.setup(line_pin_middle,GPIO.IN)
GPIO.setup(line_pin_left,GPIO.IN)
pwm_B = GPIO.PWM(Motor, 1000)
kit = ServoKit(channels=16)

def forward(x, speed):
	GPIO.output(Forward, GPIO.HIGH)
	pwm_B.start(100)
	pwm_B.ChangeDutyCycle(speed)
	print("Moving Forward")
	time.sleep(x)
	GPIO.output(Forward, GPIO.LOW)

def reverse(x, speed):
	pwm_B.start(0)
	pwm_B.ChangeDutyCycle(speed)
	GPIO.output(Backward, GPIO.HIGH)
	print("Moving Backward")
	time.sleep(x)
	GPIO.output(Backward, GPIO.LOW)

def stop():
	# GPIO.PWM(Motor, GPIO.LOW)
	GPIO.output(Forward, GPIO.LOW)
	GPIO.output(Backward, GPIO.LOW)

def findline():
	status_right = GPIO.input(line_pin_right)
	status_middle = GPIO.input(line_pin_middle)
	status_left = GPIO.input(line_pin_left)
	if status_left == 1:
		kit.continuous_servo[2].throttle = left45deg
		print("turning left 45 degrees")
	elif status_middle == 1:
		kit.continuous_servo[2].throttle = straightahead
		print("continuing straight")
	elif status_right == 1:
		kit.continuous_servo[2].throttle = right45deg
		print("turning right 45 degrees")
	else:
		kit.continuous_servo[2].throttle = straightahead
		print("nothing ahead")

	
while (True):
	c = readchar.readchar()
	if c == 'w':
		forward(3,50)
	elif c == 's':
		reverse(3,50)
	elif c == 'e':
		kit.continuous_servo[2].throttle = 0.5
		print("Straightening")
	elif c == 'd':
		kit.continuous_servo[2].throttle = 0.7
		print("Turning right")
	elif c == 'a':
		kit.continuous_servo[2].throttle = -0.7
		print("Turning left")
	elif c == 'q':
		stop()
		GPIO.cleanup()
		break
	findline()
