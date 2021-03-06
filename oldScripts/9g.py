"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

hat.frequency = 60
 
# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)
 
# kit.servo[4].angle = 180
x = 0.55
while (x > -0.55):
	# kit.continuous_servo[1].throttle = x
	kit.continuous_servo[2].throttle = x
	time.sleep(0.5)
	x -= 0.1

x = -0.55
while (x < 0.55):
	# kit.continuous_servo[1].throttle = x
	kit.continuous_servo[2].throttle = x
	time.sleep(0.5)
	x += 0.1
	

# kit.continuous_servo[1].throttle = 0.2
kit.continuous_servo[2].throttle = 0.2
	

# kit.continuous_servo[4].throttle = -1
# time.sleep(1)
# kit.servo[4].angle = 0
# kit.continuous_servo[4].throttle = 0
GPIO.cleanup()
