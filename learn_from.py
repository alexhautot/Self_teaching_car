#!/usr/bin/env python3

# Code adapted from https://github.com/tjohnson250/ev3rl/blob/master/qlearn%20touch%20and%20distance%20sensor.py
# many thanks to TJohnson

import datetime
import numpy as np
import ev3dev2.sensor.lego as sensors
import ev3dev2.motor as motors
import random
import os

#set up sensors
ultra = sensors.UltrasonicSensor()
ultra.mode = 'US-DIST-CM'

# Reducing the distance sensor states to 5 discrete states 
numDistSensorStates = 5

bins = [0]
for i in range(numDistSensorStates - 1):
    bins.append((i+1)*8)

# returns the discrete distance from the ultrasonic sensor
def getCoarseDistance(sensor, bins):
    rawDist = ultra.distance_centimeters
    dist = np.digitize([rawDist], bins)[0]
    return dist - 1

# Sets up the touch sensors
touch = sensors.TouchSensor()

numTouchSensorStates = 2

#set up tank motors
tank = motors.MoveTank('outB', 'outC')

gyro = sensors.GyroSensor()
speed = 30

# Defining basic movement - turning isn't quiet perfectly 90 degrees
# but that can be fixed after the hard tasks have been completed.
def forward():
    tank.on(speed, speed)
    tank.wait_while('running', 500)

def turn_right():
    a = gyro.value()
    while (gyro.value() >(a-90)):
        tank.on(speed, -speed)


def turn_left():
    a = gyro.value()
    while (gyro.value() <(a+90-30)):
        tank.on(-speed, speed)

def backward():
    tank.on(-speed, -speed)
    tank.wait_while('running', 500)


actions = [forward, turn_left, turn_right, backward]
numactions = len(actions)
action_names = (a.__name__ for a in actions)

def ev3action(a):
    actions[a]()

# From git hub repo, seems to be too high a dimension, could combine the world state in to [numtouch + num dist]
q_table = np.zeros([numTouchSensorStates, numDistSensorStates, numactions])

f = open("test.txt", 'r')
for i in range(numTouchSensorStates):
    for j in range(numDistSensorStates):
        for k in range(numactions):
            (q_table[i,j,k]) =str(f.readline())

f.close()

#HYPERPARAMETERS for learning
alpha = 0.1     # learning rate
gamma = 0.9     # discount rate
T = 0.8         # temperature for random 

steps = 50

def softmax(l, T):
    return(np.exp(l/T)/np.sum(np.exp(l/T)))

def softmaxAction(l, T):
    return(np.random.choice(len(l), 1, p = softmax(l, T))[0])

st = touch.value()
dist = getCoarseDistance(ultra, bins)
N0 = 10

total_reward = 0
rewards = np.zeros(200)
cycle_time = 990

for step in range(0, steps-1): 
    # Use epsilon greedy policy based on Q table
    epsilon = 0
    if random.random() > epsilon:
        a = np.argmax(q_table[st, dist]) # find action (index) with max q value for state
    else:
        a = random.randint(0, numactions-1)

    # Send selected command to EV3 robot
    ev3action(a)
   
    # read touch and ultrasonic sensors to find current state after taking action
    stp = touch.value()
    distp = getCoarseDistance(ultra, bins)

    # calculate the reward
    if stp == 1 or distp ==0 : # robot hit wall
        r = -10
    elif a == 0: # robot moved forward without hitting wall
        r = 2
    elif a == 1 or a == 2: # robot turned left or right
        r = 0
    else:       # all other possibilities
        r = -1
    
    #Updates the action/reward expectation table
    total_reward += r
    q_table[st, dist, a] = q_table[st, dist, a] + alpha * (r + gamma * np.amax(q_table[stp, distp]) - q_table[st, dist, a])
    st = stp
    dist = distp
open('file.txt', 'w').close()
f = open("test.txt", 'w')
for i in range(numTouchSensorStates):
    for j in range(numDistSensorStates):
        for k in range(numactions):
            line = str(q_table[i,j,k]) + '\n'
            f.write(line)

f.close()