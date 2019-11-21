#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
#import numpy as np 


#define sensors
gyro = GyroSensor(Port.S2)
sensor = UltrasonicSensor(Port.S1)

#define motors
left = Motor(Port.B)
right = Motor(Port.C)
robot = DriveBase(left, right, 56, 114)
beak = Motor(Port.A, Direction.CLOCKWISE,  [12, 36])



def left_turn(angle):
    #robot will rotate to specified angle in a counter clockwise direction 
    gyro.reset_angle(0)

    while (gyro.angle() < angle):
        left.run(-100)
        right.run(100)

def right_turn(angle):
    #robot will rotate to specified angle in a clockwise direction
    gyro.reset_angle()

    while (gyro.angle() >-angle):
        left.run(100)
        right.run(-100)

def beak_open():
    beak.run_time(-100, 1500)
    
def beak_close():
    beak.run_time(100, 1500)
    
    
def squawk():
    #Opens the beak, makes chicken noises then closes beak
    beak_open()
    brick.sound.file('chicken.wav')
    beak_close()


def strut ():
    #a fast then slow motion to imitate a chicken strut
    robot.drive(100,0)
    wait(500)
    left.run(100)
    right.run(100)
    wait(1000)
    robot.drive(100,0)
    wait(500)
    left.run(100)
    right.run(100)
    wait(1000)
    robot.drive(100,0)
    wait(500)
    left.run(100)
    right.run(100)
    wait(1000)
    

squawk()
strut()
strut()
right_turn(45)
strut()
squawk()
