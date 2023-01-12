#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

# Declare motors 
x = int()
x == 0
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
arm_motor = Motor(Port.A)
ultrasonicSensor = UltrasonicSensor(Port.S4)
forward = 0
left = 0
right = 0
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)


# ultrasonic sensor start
arm_motor.run_until_stalled(250)
arm_motor.reset_angle(45)
arm_motor.run_target(90,0)

def finished():
    if ultrasonicSensor.distance() < 400:
        return True
    else:
        return False

def maze_solver():
    if  arm_motor.angle() <= -90 and ultrasonicSensor.distance() >= 400:
        return True
    if  arm_motor.angle() > 0 and ultrasonicSensor.distance() >= 400:
        return True
    else:
        return False


# maze_solver start
maze_solver == False
in_file = open  

#this is deciding whether or not to move on to the next while statement and will move on if it decides it should
while maze_solver False:
#find opening
    if maze_solver = True:
        break
        
    if x == 0:
        # right side
        print(arm_motor.angle())
        print(ultrasonicSensor.distance())
        arm_motor.run_until_stalled(250)
        robot.straight(50)
        sleep(2)
        x == 1
       
    
    
    if x == 1:
        # left side
        arm_motor.run_until_stalled(-250)
        print(arm_motor.angle())
        print(ultrasonicSensor.distance())
        robot.straight(50)
        sleep(2)
        x == 0

while maze_solver = True:
    #setting up the do until
    finished = False
        if arm_motor.angle() <= 90

            while not finished:
                robot.drive(200, 0)
            if finished = True
                robot.turn(-90)
                robot.straight(70)
                break

        if arm_motor.angle() > 0

            while not finished:
                robot.drive(200, 0)
            if finished = True
                robot.turn(-90)
                robot.straight(70)
                break
    


    if maze_solver = False
        break

    


in_file.close()
