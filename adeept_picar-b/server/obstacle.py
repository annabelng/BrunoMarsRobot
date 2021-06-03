import readchar
import sys
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
from ultra import *

class Robot:

        def __init__(self):
        # setup controllers
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = adafruit_pca9685.PCA9685(self.i2c)
        self.hat.frequency = 60
        # line pin right
        GPIO.setup(20,GPIO.IN)

        # line pin left
        GPIO.setup(19,GPIO.IN)

        # forward
        GPIO.setup(18, GPIO.OUT)

        # backward
        GPIO.setup(27, GPIO.OUT)

        # motor
        GPIO.setup(17, GPIO.OUT)

        self.pwm_B = GPIO.PWM(17, 1000)
        self.kit = ServoKit(channels=16)

        # set forwards direction
        self.set_direction(GPIO.LOW, GPIO.HIGH)

        # set speed
        self.set_speed(0)
        self.set_turn(0.2)

        self.left_count = 0
        self.right_count = 0

    # setting direction of motor
    def set_direction(self, backwards, forwards):
        GPIO.output(27, backwards)
        GPIO.output(18, forwards)

    # motor shutdown
    def stop(self):
        self.set_direction(GPIO.LOW, GPI.LOW)
        GPIO.cleanup()

    # set motor speed
    def set_speed(self):
        self.speed = speed
        self.pwm_B.start(100)
        self.pwm_B.ChangeDutyCycle(self.speed)

    # adjust amount of turn
    def adjust_turn(self, increment):
        self.turn += increment
        if(self.turn > 0.9):
            self.turn = 0.9
        elif(self.turn < -0.5):
            self.turn = -0.5

        self.kit.continuous_servo[2].throttle = self.turn

    # straighten out wheels
    def straight(self):
        self.set_turn(0.2)

    # move forward until robot encounters obstacle 10 cm away
    def run_course(self)
        # set the speed
        self.set_speed(40)
        self.straight
        while(True):
            distance = checkDist()
            if distance < 10:
                self.set_speed(0)
                self.stop()
                print ("obstacle is less than 10 cm away")

if __name__ == "main":
    robot = Robot()
    robot.run_course()
