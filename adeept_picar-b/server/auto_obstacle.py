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
        GPIO.setup(16, GPIO.OUT)
        GPIO.setup(21, GPIO.OUT)

        #red light
        GPIO.setup(15, GPIO.OUT)
        GPIO.setup(19, GPIO.OUT)

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
        self.servoMiddle = 180
        # left position of servo
        self.servoLeft = 60
        # right position of servo
        self.servoRight = 300

        # range of cm before robot turns or stops
        self.range = 25

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

    def test(self, speed):
        #setup()
        self.straight()
        self.set_speed(40)
        time.sleep(0.2)
        self.set_speed(speed)
       # self.set_turn(turn)
        while(True):
            if (checkdist() < 30):
                print("obstacle is less than 30 cm away")
                #both_off()
                GPIO.output(16, off)
                GPIO.output(21, off)
                red()
                self.stop()
                break
               # continue
            else:
                print ("obstacle is more than 30 cm away")
                green()

    def run_obstacle(self,speed):
        self.straight()
        #self.set_speed(speed)
        self.avoid_obstacle()

    def avoid_obstacle(self):
        count = 0
        while (True):
            print('automatic obstacle avoidance')
            #self.straight()
            # rotate head to left and check distance
            if self.scanPos == 1:
                # arguments are channel number, on, number to count up to
                # before turning off
                self.pwm.set_pwm(self.servoPort, 0, self.servoLeft)
                time.sleep(0.3)
                self.scanList[0] = checkdist()

            # rotate head to middle and check distance
            elif self.scanPos == 2:
                self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)
                time.sleep(0.3)
                self.scanList[1] = checkdist()

            # rotate head to right and check distance
            elif self.scanPos == 3:
                self.pwm.set_pwm(self.servoPort, 0,self.servoRight)
                time.sleep(0.3)
                self.scanList[2] = checkdist()

            self.scanPos = self.scanPos + self.scanDir

            if self.scanPos > self.scanNum or self.scanPos < 1:
                if self.scanDir == 1:
                    self.scanDir = -1
                elif self.scanDir == -1:
                    self.scanDir = 1
                self.scanPos = self.scanPos + self.scanDir*2
            print(self.scanList)
            count += 1
            if count > 3:
                self.check_turn()

    def check_turn(self):
        min_dist = min(self.scanList)
        max_dist = max(self.scanList)
        left = self.scanList[0]
        middle = self.scanList[1]
        right = self.scanList[2]

        if min_dist < self.rangeKeep:
            min_index = self.scanList.index(min_dist)

            # check if shortest distance is on left
            if min_index == 0:
                # turn right
                self.adjust_turn(0.2)
                self.adjust_speed(0.1)
                time.sleep(2)
                self.straight()
                print("turn right")

            # shortest distance on right
            elif min_index == 2:
                # turn left
                self.adjust_turn(-0.2)
                self.adjust_speed(0.4)
                print("turn left")
                time.sleep(2)
                self.straight()

            # shortest distance in middle
            # compare left and right distance
            else:
                if left > right:
                    # turn right
                    self.adjust_turn(0.2)
                    self.adjust_speed(0.1)
                    print("turn right")
                    time.sleep(2)
                    self.straight()
                else:
                    self.adjust_turn(-0.2)
                    self.adjust_speed(0.1)
                    print("turn left")
                    time.sleep(2)
                    self.straight()
            if max_dist < self.rangeKeep:
                # reverse robot
                self.stop()
                self.set_direction(GPIO.HIGH, GPIO.LOW)
                self.set_speed(30)
                time.sleep(3)
                print("reverse")
        else:
            # no obstacle so go forward
            self.straight()
            self.set_speed(30)
            time.sleep(0.5)
            print("forward")

    def test_turn(self):
        while (True):
            self.straight()
            self.set_speed(50)
            time.sleep(0.5)
            self.adjust_turn(0.2)
            self.adjust_speed(10)
            time.sleep(1.0)

    def run(self):
        # Staighten the wheel and start the motor. Accelerate three times and decelerate once to get to the right speed
        self.set_speed(50)
        time.sleep(0.2)
        self.set_speed(40)
        self.straight()
        while(True):
            print(self.speed)
            right = self.get_line_right()
            left = self.get_line_left()
            if left:
                self.left_count += 1
                self.adjust_turn(0.2)
                self.adjust_speed(0.1)
            else:
                self.left_count = 0

            if right:
                self.right_count += 1
                self.adjust_turn(-0.2)
                self.adjust_speed(0.1)
            else:
                self.right_count = 0

            if not right and not left:
                self.straight()
                self.set_speed(35)


if __name__ == "__main__":
    robot = Robot()
    robot.test(40)
    #robot.stop()
    print('am i running')
    #robot.test_turn()
    #robot.run_obstacle(50)
    #robot.run()
    
