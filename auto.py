import readchar
import sys
import time
import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685

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

    def get_line_right(self):
        return GPIO.input(20)

    def get_line_left(self):
        return GPIO.input(19)

    def set_direction(self, backwards, forwards):
        GPIO.output(27, backwards)
        GPIO.output(18, forwards)

    def stop(self):
        self.set_direction(GPIO.LOW, GPI.LOW)
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

    def test(self, speed, turn):
        self.set_speed(50)
        time.sleep(0.2)
        self.set_speed(speed)
        self.set_turn(turn)
        while(True):
            continue

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

    '''
    while(True):
        right = line_right()
        left = line_left()


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
    '''

if __name__ == "__main__":
    robot = Robot()
    #robot.test(50, 0.2)
    robot.run()
