#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.media.ev3dev import SoundFile, ImageFile
from robotics import Robot, Navigator
from globals import *

def extinguish():
	"""Extinshuishes Flame"""
	ev3.speaker.say("Fire has been found")
	
print("-------------------- Start --------------------------")

# Initialize the EV3 Brick.
ev3 = EV3Brick()
ev3.speaker.set_speech_options(voice='f3')

fireNotFound = True

# Initialize sensors and motors
rMotor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
lMotor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
nav = Navigator()
fSensor = TouchSensor(Port.S3)
lSensor = TouchSensor(Port.S1)
cSensor = ColorSensor(Port.S4)
uSensor = UltrasonicSensor(Port.S2)

robot = Robot(lMotor,rMotor, nav, fSensor, lSensor, cSensor ,uSensor)

while robot.fireNotFound:
	robot.update_sensors()
	robot.update_queue()
	robot.process_behavior()

extinguish()

print("-------------------- End --------------------------")




