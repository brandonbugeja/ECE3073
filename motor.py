# File name   : motor.py
# https://github.com/adeept/Adeept_PiCar-B/blob/master/server/motor.py

#!/usr/bin/python3
# Date modified: 10/08/2021

# originally created by:
# File name   : motor.py
# Description : Control Motors 
# Website     : www.adeept.com
# E-mail      : support@adeept.com
# Author      : William
# Date        : 2018/10/12

import RPi.GPIO as GPIO
import time

# motor_EN_A: Pin7         |  motor_EN_B: Pin11
# motor_A:  Pin8, Pin10    |  motor_B: Pin13, Pin12

# GPIO input
Enc_A = 17  
Enc_B = 27

# GPIO output
Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

frequency = 50000

# initial pwm state
pwm_A = 0
pwm_B = 0

def setup():# Motor initialization
    global pwm_A, pwm_B
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_A_EN, GPIO.OUT)
    GPIO.setup(Motor_B_EN, GPIO.OUT)
    GPIO.setup(Motor_A_Pin1, GPIO.OUT)
    GPIO.setup(Motor_A_Pin2, GPIO.OUT)
    GPIO.setup(Motor_B_Pin1, GPIO.OUT)
    GPIO.setup(Motor_B_Pin2, GPIO.OUT)
    try:
        pwm_A = GPIO.PWM(Motor_A_EN, frequency)
        pwm_B = GPIO.PWM(Motor_B_EN, frequency)
    except:
        pass

def motorStop():# Motor stops
    GPIO.output(Motor_A_Pin1, GPIO.LOW)
    GPIO.output(Motor_A_Pin2, GPIO.LOW)
    GPIO.output(Motor_B_Pin1, GPIO.LOW)
    GPIO.output(Motor_B_Pin2, GPIO.LOW)
    GPIO.output(Motor_A_EN, GPIO.LOW)
    GPIO.output(Motor_B_EN, GPIO.LOW)

def motor_right(status, direction, speed):# Motor 2 positive and negative rotation
    global  pwm_B
    if status == 0: # stop
        motorStop()
    else:
        if direction == Dir_forward:
            GPIO.output(Motor_B_Pin1, GPIO.HIGH)
            GPIO.output(Motor_B_Pin2, GPIO.LOW)
            pwm_B.start(100)
            pwm_B.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_B_Pin1, GPIO.LOW)
            GPIO.output(Motor_B_Pin2, GPIO.HIGH)
            pwm_B.start(0)
            pwm_B.ChangeDutyCycle(speed)

def motor_left(status, direction, speed):# Motor 1 positive and negative rotation
    global pwm_A
    if status == 0: # stop
        motorStop()
    else:
        if direction == Dir_forward:#
            GPIO.output(Motor_A_Pin1, GPIO.HIGH)
            GPIO.output(Motor_A_Pin2, GPIO.LOW)
            pwm_A.start(100)
            pwm_A.ChangeDutyCycle(speed)
        elif direction == Dir_backward:
            GPIO.output(Motor_A_Pin1, GPIO.LOW)
            GPIO.output(Motor_A_Pin2, GPIO.HIGH)
            pwm_A.start(0)
            pwm_A.ChangeDutyCycle(speed)
    return direction


def destroy():
    motorStop()
    GPIO.cleanup()             # Release resource


try:
    pass
except KeyboardInterrupt:
    destroy()
   
  
  # Encoder counter code


import RPi.GPIO as GPIO
from time import sleep
 
counter = 10
Enc_A = 1
Enc_B = 2
  
 
 
def init():
    print ("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Enc_A , GPIO.IN)
    GPIO.setup(Enc_B , GPIO.IN)
    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode, bouncetime=10)
    return
 
 
def rotation_decode(Enc_A):
    global counter
    sleep(0.002)
    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)
 
    if (Switch_A == 1) and (Switch_B == 0):
        counter += 1
        print ("direction -> ", counter)
        while Switch_B == 0:
            Switch_B = GPIO.input(Enc_B)
        while Switch_B == 1:
            Switch_B = GPIO.input(Enc_B)
        return
 
    elif (Switch_A == 1) and (Switch_B == 1):
        counter -= 1
        print ("direction <- ", counter)
        while Switch_A == 1:
            Switch_A = GPIO.input(Enc_A)
        return
    else:
        return
 
def main():
    try:
        init()
        while True :
            sleep(1)
 
    except KeyboardInterrupt:
        GPIO.cleanup()
 
if __name__ == '__main__':
    main()
