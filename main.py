#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.media.ev3dev import SoundFile, ImageFile
from robotics import Robot, Navigator
from globals import *
from behaviors import (RobotBehavior, TouchBehavior, FireDetection, WallFollowing, Wander)
import math
import heapq


def extinguish():
	ev3.speaker.say("Fire has been found")
	
print("-------------------- Start --------------------------")
print("Program running...")
print("-----------------------")

# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.speaker.set_speech_options(voice='f3')

fireNotFound = True

robot = Robot(Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE), Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE), 
              Navigator(), TouchSensor(Port.S3), TouchSensor(Port.S1),
			  ColorSensor(Port.S4), UltrasonicSensor(Port.S2))


while robot.fireNotFound:
	robot.update_sensors()
	robot.update_queue()
	robot.process_behavior()

extinguish()



