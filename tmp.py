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
line_pin_right = 20
line_pin_left = 19
Maxspeed = 90
Minspeed = 0
Maxthrottle = 1.0
Minthrottle = -1.0
calibration = 0.2
line_mark_count = 1

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

def motor_adjust(increment):
    global pwm_B
    global speed

    speed += increment
    if(speed > Maxspeed):
        speed = Maxspeed
    elif(speed < Minspeed):
        speed = Minspeed

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

def motor_straight():
	global turn
	global kit

	turn = calibration
	kit.continuous_servo[2].throttle = calibration

def run():

    motor_setup()
    line_setup()
    right_count = 0
    left_count = 0

    # Staighten the wheel and start the motor. Accelerate three times and decelerate once to get to the right speed
    motor_straight()
    motor_forward()
    motor_adjust(5)
    time.sleep(0.1)
    motor_adjust(5)
    time.sleep(0.1)
    motor_adjust(5)
    time.sleep(0.1)
    '''
    motor_accelerate()
    time.sleep(0.1)
    motor_accelerate()
    time.sleep(0.1)
    motor_accelerate()
    time.sleep(0.1)
    motor_decelerate()
    motor_decelerate()
    motor_decelerate()
    '''

    while(True):
        # check line detection
        right_detected = line_right()
        left_detected = line_left()

        # To avoid glitches, we want to see the right marker two times before acting
        if (right_detected):
            right_count += 1
            print("RIGHT DETECTED")
        else:
            right_count = 0
        if (right_count >= line_mark_count):
            actual_right_detected = True
        else:
            actual_right_detected = False

        # To avoid glitches, we want to see the left marker two times before acting
        if (left_detected):
            left_count += 1
            print("LEFT DETECTED")
        else:
            left_count = 0
        if (left_count >= line_mark_count):
            actual_left_detected = True
        else:
            actual_left_detected = False

        #print("l = ", left_detected, " lc = ", left_count, " al = ", actual_left_detected)
        #print("r = ", right_detected, " rc = ", right_count, " ar = ", actual_right_detected)

        if (actual_right_detected):
            motor_right()
            motor_right()
            motor_right()
            motor_right()
            # Turn right for 1/2 second. Set the turn in motion and sleep.
            time.sleep(0.2)
            # Straighten the wheel and continue moving
            motor_straight()
        elif (actual_left_detected):
            motor_left()
            motor_left()
            motor_left()
            motor_left()
            # Turn left for 1/2 second. Set the turn in motion and sleep.
            time.sleep(0.2)
            # Straighten the wheel and continue moving
            motor_straight()
        elif (actual_right_detected and actual_left_detected):
            #motor_stop()
            #break
            pass
        else:
            # Keep going
            time.sleep(0.001)

if __name__ == "__main__":
    run()
