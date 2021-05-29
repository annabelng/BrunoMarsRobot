import sys
import time
import RPi.GPIO as GPIO

import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685

mode=GPIO.getmode()

GPIO.cleanup()

"""
Forward=13
Backward=12
Motor=11
"""
Forward=18
Backward=27
Motor=17
sleeptime=1

global pwm_B

GPIO.cleanup()
GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

hat.frequency = 60

GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)
GPIO.setup(Motor, GPIO.OUT)
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
	
 
#kit.continuous_servo[1].throttle = 0.2
kit.continuous_servo[2].throttle = 0.2
forward(2, 70)
stop()
reverse(2, 70)
stop()
#kit.continuous_servo[1].throttle = 0.4
kit.continuous_servo[2].throttle = 0.4
forward(2,80)
stop()
reverse(2, 70)
stop()
#kit.continuous_servo[1].throttle = 0.6
kit.continuous_servo[2].throttle = 0.6
forward(2, 70)
stop()
reverse(2, 70)
#kit.continuous_servo[1].throttle = 0.2
kit.continuous_servo[2].throttle = 0
stop()
GPIO.cleanup()
