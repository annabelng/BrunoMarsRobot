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
        self.servoMiddle = 235
        # left position of servo
        self.servoLeft = 350
        # right position of servo
        self.servoRight = 100

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

    def check_obstacle(self, speed):
        #setup()
        time.sleep(2)
        self.straight()
        self.set_speed(40)
        time.sleep(0.2)
        self.set_speed(speed)
       # self.set_turn(turn)
        while(True):
            # check is obstacle is less than 30 cm away
            self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)
            if (checkdist() < self.range):
                print("obstacle is less than " + str(self.range) + " cm away")
                # turn on red light
                self.red()

                # stop motors
                self.pause()

                # check distances in each direction
                # figure out which direction to turn
                # turn for 2 seconds
                self.turn_obstacle()

            else:
                #print ("obstacle is more than "+ str(self.range) + " cm away")
                self.green()

    def turn_obstacle(self):
        dir = self.check_turn()
        # reverse for 2 seconds
        self.set_speed(35)
        self.set_direction(GPIO.HIGH, GPIO.LOW)
        time.sleep(2)
        # stop after reversing
        self.pause()
        time.sleep(0.5)
        # set forward direction
        self.set_direction(GPIO.LOW, GPIO.HIGH)
        if dir == 'right':
            # turn right for 2 sec
            print('turning right')
            self.set_turn(0.6)
            self.set_speed(60)
            time.sleep(1.9)

            # straighten motor after turning
            # lower speed
            self.straight()
            self.set_speed(40)
            time.sleep(0.2)

             # straighten to forward direction
            # by turning the opposite direction
            self.set_turn(-0.2)
            self.set_speed(50)
            time.sleep(1.5)
            # straighten again
            self.straight()
            self.set_speed(40)


        elif dir == 'left':
            # turn left for 2 sec
            print('turning left')
            self.set_turn(-0.2)
            self.set_speed(60)
            time.sleep(1.9)

            # straighten motor after turning
            # lower speed
            self.straight()
            self.set_speed(40)
            time.sleep(0.2)

            # straighten to forward direction
            # by turning the opposite direction
            self.set_turn(0.6)
            self.set_speed(50)
            time.sleep(1.5)
            # straighten again
            self.straight()
            self.set_speed(40)

    def obstacle_distance(self):
        self.scanPos = 1
        for x in range(3):
            print('checking distances')
            #self.straight()
            # rotate head to left and check distance
            if self.scanPos == 1:
                # print('left distance')
                # arguments are channel number, on, number to count up to
                # before turning off
                self.pwm.set_pwm(self.servoPort, 0, self.servoLeft)
                time.sleep(0.2)
                self.scanList[0] = checkdist()
                print('left distance is ' + str(self.scanList[0]))

            # rotate head to middle and check distance
            elif self.scanPos == 2:
                #print('middle distance')
                self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)
                time.sleep(0.2)
                self.scanList[1] = checkdist()
                print('middle distance is ' + str(self.scanList[0]))

            # rotate head to right and check distance
            elif self.scanPos == 3:
                #print('right distance')
                self.pwm.set_pwm(self.servoPort, 0,self.servoRight)
                time.sleep(0.2)
                self.scanList[2] = checkdist()
                print('right distance is ' + str(self.scanList[0]))

            # next direction 
            self.scanPos += 1

        # reset head to middle
        self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)

    def check_turn(self):
        # get updated distancs in each dir
        self.obstacle_distance()

        # min and max distance value
        min_dist = min(self.scanList)
        max_dist = max(self.scanList)

        # setting distances for each dir
        left = self.scanList[0]
        middle = self.scanList[1]
        right = self.scanList[2]

        # check if minimum distance is less than range
        if min_dist < self.range:
            min_index = self.scanList.index(min_dist)

            # check if shortest distance is on left
            if min_index == 0:
                # turn right since obstacle on left
                print('closest obstacle is ' + str(left) + ' away on the left')
                return('right')

            # shortest distance on right
            elif min_index == 2:
                # turn left since obstacle on right
                print('closest obstacle is ' + str(right) + ' away on the right')
                return('left')

            # shortest distance in middle
            # compare left and right distance
            else:
                if left > right:
                    # turn right
                    print('closest obstacle is ' + str(left) + ' away on the left')
                    return('right')

                else:
                    # turn left
                    print('closest obstacle is ' + str(right) + ' away on the right')
                    return('left')

    def test_turn(self):
        while (True):
            self.straight()
            self.set_speed(50)
            time.sleep(0.5)
            self.adjust_turn(0.2)
            self.set_speed(60)
            time.sleep(1.0)

    def test_head(self):
        print('left head')
        self.pwm.set_pwm(self.servoPort, 0, self.servoLeft)
        time.sleep(1)
        print('middle head')
        self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)
        time.sleep(1)
        print('right head')
        self.pwm.set_pwm(self.servoPort, 0, self.servoRight)
        time.sleep(1)
        self.pwm.set_pwm(self.servoPort, 0, self.servoMiddle)



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
                self.set_speed(60)
            else:
                self.left_count = 0

            if right:
                self.right_count += 1
                self.adjust_turn(-0.2)
                self.set_speed(60)
            else:
                self.right_count = 0

            if not right and not left:
                self.straight()
                self.set_speed(35)

    def red(self):
        # turning green off
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(9, GPIO.HIGH)

        # turning red on
        GPIO.output(22, GPIO.LOW)
        GPIO.output(10, GPIO.LOW)
        print('red on')

    def green(self):
        # turning red off
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(10, GPIO.HIGH)

        # turning green on
        GPIO.output(23, GPIO.LOW)
        GPIO.output(9, GPIO.LOW)
        #print('green on')

    def both_off(self):
        GPIO.output(22, GPIO.HIGH)
        GPIO.output(10, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(9, GPIO.HIGH)
        print('both off')

    def lights(self):
        for x in range(3):
            self.red()
            time.sleep(0.3)
            self.green()
            time.sleep(0.3)
        self.both_off()

if __name__ == "__main__":
    robot = Robot()
    #robot.test_head()
    #robot.test(40)
    #robot.stop()
    print('am i running')
    #robot.test_turn()
    robot.check_obstacle(40)
    #robot.run() 
    #robot.lights()
