import readchar
import sys
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685

Forward = 18
Backward = 27
Motor = 17
sleeptime = 1
Increment = 5
line_pin_right = 19
line_pin_left = 20
Maxspeed = 90
Minspeed = 0
Maxthrottle = 1.0
Minthrottle = -1.0
calibration = -0.3

global pwm_B
global speed
global turn
global kit

def motor_setup():
	global pwm_B
	global speed
	global turn
	global kit

	turn = 0.45
	speed = 20
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	i2c = busio.I2C(board.SCL, board.SDA)
	hat = adafruit_pca9685.PCA9685(i2c)
	hat.frequency = 60
	GPIO.setup(Forward, GPIO.OUT)
	GPIO.setup(Backward, GPIO.OUT)
	GPIO.setup(Motor, GPIO.OUT)
	pwm_B = GPIO.PWM(Motor, 1000)
	kit = ServoKit(channels=16)

def line_setup():
	GPIO.setup(line_pin_right,GPIO.IN)
	GPIO.setup(line_pin_left,GPIO.IN)

def line_right():
	return GPIO.input(line_pin_right)

def line_left():
	return GPIO.input(line_pin_left)

def motor_forward():
	global pwm_B
	GPIO.output(Backward, GPIO.LOW)
	GPIO.output(Forward, GPIO.HIGH)

def motor_backward():
	global pwm_B
	GPIO.output(Forward, GPIO.LOW)
	GPIO.output(Backward, GPIO.HIGH)

def motor_accelerate():
	global pwm_B
	global speed

	if(speed < Maxspeed):
		speed += Increment

	pwm_B.start(100)
	pwm_B.ChangeDutyCycle(speed)

def motor_decelerate():
	global pwm_B
	global speed

	if(speed > Minspeed):
		speed -= Increment

	pwm_B.start(100)
	pwm_B.ChangeDutyCycle(speed)

def motor_stop():
	GPIO.output(Forward, GPIO.LOW)
	GPIO.output(Backward, GPIO.LOW)
	GPIO.cleanup()

def motor_left():
	global turn
	global kit

	if(turn < Maxthrottle):
		turn += 0.1
#	kit = ServoKit(channels=16)
	kit.continuous_servo[2].throttle = turn

def motor_right():
	global turn
	global kit

	if(turn > Minthrottle):
		turn -= 0.1

	kit.continuous_servo[2].throttle = turn

motor_setup()

while(True):
	c = readchar.readchar()
	if(line_right() == 1 and line_left() == 1):
		c = 'q'
	if c == 'w':
		motor_forward()
		print("forward")
	elif c== 's':
		motor_backward()
		print("backward")
	elif c == 'e':
		motor_accelerate()
		print("accelerating")
	elif c == 'd':
		motor_decelerate()
		print("decelerating")
	elif c == 'f':
		motor_right()
		print("right")
	elif c == 'a':
		motor_left()
		print("left")
	elif c == 'x':
		kit.continuous_servo[2].throttle = calibration
		print("straighten")
	elif c == 'q':
		motor_stop()
		break
