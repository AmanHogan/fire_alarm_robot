#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction 
from pybricks.media.ev3dev import SoundFile, ImageFile
from robotics import Robot, Navigator
from globals import *
from behaviors import (RobotBehavior, TouchBehavior, FireDetection, WallFollowing, Wander)
import math
import heapq

print("-------------------- Start --------------------------")
print("Program running...")
print("-----------------------")

# Initialize the EV3 Brick.
ev3 = EV3Brick()



robot = Robot(Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE), 
              Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE), 
              Navigator(),
			  TouchSensor(Port.S3),
			  ColorSensor(Port.S4),
			  UltrasonicSensor(Port.S2))


while True:

	robot.update_sensors()
	robot.update_queue()
	robot.process_behavior()



# while True:
#     robot.run()
#     #isTouched = touch_sensor.pressed()
#     #a = sonic_sensor.distance()

#     if isTouched:
#         robot.turn(90)
#         robot.turn(90)
#         robot.turn(45)
        


    # if a < 200:
    #     angle_to_wall = math.degrees(math.atan(a / d))
    #     turning_angle = navigator.orientation - angle_to_wall
    #     # Adjust the turning angle within the range of 0 to 180 degrees for simplicity
    #     turning_angle = turning_angle % 180
    #     # Turn the robot by the calculated angle
    #     robot.turn(180-turning_angle)
    

    
    #print("Distance: " + str(sonic_sensor.distance()))
    #print("Touched: " + str(touch_sensor.pressed()))
    #print("Color: " + str(color_sensor.ambient()))







# # Get angles between points on path found
# print("List of Angles")
# angles = kn.find_angles_between_positions(path_found)
# angles = [round(i,5) for i in angles]
# angles.pop(0)
# print(angles)
# print("-----------------------")

# # Get distances between points on path found
# print("List of distances")
# distances = kn.calculate_distances(path_found)
# distances = [round(i,5) for i in distances]
# print(distances)
# print("-----------------------")

