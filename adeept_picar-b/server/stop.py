import readchar
import sys
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
from ultra import *
from move import *
import Adafruit_PCA9685
from led import *

class Robot:

    def __init__(self):
        # setup controllers
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.hat = adafruit_pca9685.PCA9685(self.i2c)
        self.hat.frequency = 60

        # initiate pwm
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50)

        #green light
        GPIO.setup(23, GPIO.OUT)
        GPIO.setup(9, GPIO.OUT)

        #red light
        GPIO.setup(22, GPIO.OUT)
        GPIO.setup(10, GPIO.OUT)

        # line pin right
        #GPIO.setup(20,GPIO.IN)

        # line pin left
        #GPIO.setup(19,GPIO.IN)

        # forward
        GPIO.setup(18, GPIO.OUT)

        # backward
        GPIO.setup(27, GPIO.OUT)

        # motor
        GPIO.setup(17, GPIO.OUT)

        # servo for controlling horizontal rotation of ultrasonic sensor
        self.servoPort = 1
        # middle position of servo
        self.servoMiddle = 150
        # left position of servo
        self.servoLeft = 0
        # right position of servo
        self.servoRight = 300

        # range of cm before robot turns or stops
        self.range = 30

        # scan direction
        # 1 is left to right
        # -1 is right to left
        self.scanDir = 1
        # current scan position
        self.scanPos = 1
        self.scanNum = 3
        self.scanList = [0,0,0]

        # distance in cm before robot turns
        self.rangeKeep = 15

        self.pwm_B = GPIO.PWM(17, 1000)
        self.kit = ServoKit(channels=16)

        # set forwards direction
        self.set_direction(GPIO.LOW, GPIO.HIGH)

        # set speed
        self.set_speed(0)
        self.set_turn(0.2)

        self.left_count = 0
        self.right_count = 0

    def get_line_right(self):
        return GPIO.input(20)

    def get_line_left(self):
        return GPIO.input(19)

    def set_direction(self, backwards, forwards):
        GPIO.output(27, backwards)
        GPIO.output(18, forwards)

    def stop(self):
        self.set_direction(GPIO.LOW, GPIO.LOW)
        GPIO.cleanup()

    def pause(self):
        self.set_direction(GPIO.LOW, GPIO.LOW)

    def adjust_speed(self, increment):
        self.speed += increment
        if(self.speed > 100):
            self.speed = 100
        elif(self.speed < 0):
            self.speed = 0

        self.pwm_B.start(100)
        self.pwm_B.ChangeDutyCycle(self.speed)

    def set_turn(self, turn):
        self.turn = turn
        if(self.turn > 0.9):
            self.turn = 0.9
        elif(self.turn < -0.5):
            self.turn = -0.5
        self.kit.continuous_servo[2].throttle = self.turn

    def set_speed(self, speed):
        self.speed = speed
        self.pwm_B.start(100)
        self.pwm_B.ChangeDutyCycle(self.speed)

    def adjust_turn(self, increment):
        self.turn += increment
        if(self.turn > 0.9):
            self.turn = 0.9
        elif(self.turn < -0.5):
            self.turn = -0.5

        self.kit.continuous_servo[2].throttle = self.turn

    def straight(self):
        self.set_turn(0.2)
 
if __name__ == "__main__":
    robot = Robot()
    #robot.test(40)
    robot.stop()
    print('clean up')
    #robot.test_turn()
    #robot.check_obstacle(40)
    #robot.run() 
