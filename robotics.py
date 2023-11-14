"""Acts as a replacement for pybricks robotics module. Module is
responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Color
from globals import *
from logger import log
import heapq
from behaviors import (RobotBehavior, TouchBehavior, FireDetection, WallFollowing, Wander)
import math

######################################### Navigation class ############################################

class Navigator:
    """Class responsible for keeping track of the robot's logical orientation
    """
    def __init__(self):
        self.orientation = 0 # orientation [deg]
        self.orientations = [self.orientation] # List of orientations [deg]
        log("Current Robot Orientation: " + str(self.orientation) + " degrees...")

    def update_nav(self, angle) -> None:
        """
        Updates the logical orientation of the robot and keeps track of previous positions\n
        Args: angle (int): angle that the robot needs to be be turned by
        """
        # Update the orientation based on the turn and keep within the range of -180 to 180 degrees
        self.orientation = (self.orientation + angle) % 360
        self.orientations.append(self.orientation)
        

######################################### Robot class ############################################

class Robot:
    """Custom defined Robot class for the lego ev3 robot. Responsible for moving and turning the robot.
    """
    def __init__(self, left_motor: Motor, right_motor: Motor, navigator: Navigator, 
                 frontTouch: TouchSensor, leftTouch: TouchSensor,
                 color: ColorSensor, sonic: UltrasonicSensor ):
        
        self.left_motor = left_motor # controls left tire
        self.right_motor = right_motor # controls right tire
        self.navigator = navigator # object to keep track of robot's orientation
        
        self.frontTouch = frontTouch # right touch sensor
        self.leftTouch = leftTouch # left touch sensor
        self.color = color # color sensor
        self.sonic = sonic # ultrasonic sensor

        self.queue = [] # priority queue
        heapq.heapify(self.queue)

        self.hasHitFrontWall = frontTouch.pressed() # True if sensor touched front wall
        self.hasHitLeftWall = leftTouch.pressed() # True if sensor touched left wall
        self.distanceToWall = sonic.distance() # distance to wall [mm]
        self.current_color = color.color() # Color detected by Color Sensor

        self.wallFollowingDistance = 0 # Starting Distance from wall when wall following [mm]
        self.isFollowingWall = False # True if robot is in WallFollowing
        self.isWandering = False # True if Robot is in Wander
        self.isFollowingFire = False # True if robot is in FireDetection
        self.fireNotFound = True # True if fire is found

    def move(self, distance) -> None:
        """
        Moves robot a given distance in [mm]\n
        Args: distance (float): Distance to be traveled [mm]
        """

        # tire rotation angle
        angle =  (((distance)/TIRE_CIRC)*FULL_ROTATION)*DISTANCE_ERROR 
        self.left_motor. run_angle(TIRE_RPM, angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, angle)
        log("Robot Moved: " + str(distance) + " milimeters...")

    def turn(self, angle) -> None:
        """
        Turns robot a given angle [deg]\n
        Args: angle (float): angle to turn robot [deg]
        """

        # Calculate steering angle
        steering_angle =  (((2*M_PI*ROBOT_RADIUS*(angle/360))/TIRE_CIRC)*FULL_ROTATION)*TURN_ERROR
        log("Robot supposed to turn: " + str(angle) + " degrees...")

        for i in range(11):

            self.update_sensors()
            if self.hasHitFrontWall:
                log("Robot Hit a wall in the front while turning!")
                self.move(-50)
                break

            if self.hasHitLeftWall:
                log("Robot Hit a wall on the left while turning!")
                self.left_motor. run_angle(TIRE_RPM, (steering_angle/3), wait=False)
                self.right_motor.run_angle(TIRE_RPM, -(steering_angle/3))
                break

            self.left_motor. run_angle(TIRE_RPM, -(steering_angle/10), wait=False)
            self.right_motor.run_angle(TIRE_RPM, (steering_angle/10))
            self.navigator.update_nav(steering_angle/10) # update orientation
            
        log("Current Robot Orientation: " + str(self.navigator.orientation) + " degrees...")
        self.update_sensors()
        
    def run(self) -> None:
        """
        The motor accelerates to TIRE_RPM 
        and keeps running at this speed 
        until you give a new command.
        """
        self.left_motor.run(TIRE_RPM)
        self.right_motor.run(TIRE_RPM)

    def stop(self) -> None:
        """
        Stops the motor and lets it spin freely.
        The motor gradually stops due to friction.
        """
        self.left_motor.brake()
        self.right_motor.brake()

    def process_behavior(self) -> None:
        """
        Proccess a behavior from the priority queue. Pops the highest priority 
        behavior from queue then processes the behavior in RobotBehavior classes.
        """

        log("Priority queue BEFORE Pop..." + str(self.queue))
        behavior = heapq.heappop(self.queue)
        log("Priority after AFTER Pop..." + str(self.queue))
        
        if behavior.priority == TOUCH:
            log("Touched a wall, recalibrating position...")
            behavior.run(self)
            log("Finished Recalibrating position...")

        if behavior.priority == FIRE:
            log("Detected a light...")
            behavior.run(self)

        if behavior.priority == WALL_FOLLOW:
            log("Detected a wall...")
            behavior.run(self)

        if behavior.priority == WANDER:
            log("Starting to Wander...")
            behavior.run(self)
            
    def update_sensors(self) -> None:
        """Function updates all of the robot's sensor values and stores these values.
        """
        self.hasHitFrontWall = self.frontTouch.pressed()
        self.hasHitLeftWall = self.leftTouch.pressed()
        self.distanceToWall = self.sonic.distance()
        self.current_color = self.color.color()

    def update_queue(self) -> None:
        """Updates the priority queue using the robot's sensor values. Defaults to Wander if
        priority queue is empty.
        """
        if not self.queue:
            if not any(isinstance(behavior, Wander) for behavior in self.queue):
                self.queue.append(Wander())

        if self.hasHitFrontWall or self.hasHitLeftWall:
            if not any(isinstance(behavior, TouchBehavior) for behavior in self.queue):
                self.queue.append(TouchBehavior())

        if self.distanceToWall < MIN_WALL_DISTANCE:
            if not any(isinstance(behavior, WallFollowing) for behavior in self.queue):
                if self.isFollowingWall == False:
                    self.queue.append(WallFollowing())

        if self.current_color ==  Color.RED:
            if not any(isinstance(behavior, FireDetection) for behavior in self.queue):
                self.queue.append(FireDetection())
        

############################################################################################
