"""Acts as a replacement for pybricks robotics module. Module is
responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction
from globals import *
from logger import log
import heapq
from behaviors import (RobotBehavior, TouchBehavior, FireDetection, WallFollowing, Wander)



######################################### Navigation class ############################################

class Navigator:
    """Class responsible for keeping track of the robot's logical orientation
    """
    def __init__(self):
        self.orientation = 0 # orientation [deg]
        self.orientations = [self.orientation] # List of orientations [deg]
        log("Orientation: " + str(self.orientation))

    def update_nav(self, angle) -> None:
        """Updates the logical orientation of the robot and keeps track of previous positions\n
        Args: angle (int): angle that the robot needs to be be turned by
        """
        # Update the orientation based on the turn and keep within the range of -180 to 180 degrees
        self.orientation = (self.orientation + angle) % 360
        self.orientations.append(self.orientation)
        log("Orientation: " + str(self.orientation))

######################################### Robot class ############################################

class Robot:
    """Custom defined Robot class for the lego ev3 robot. Responsible for moving and turning the robot.
    """
    def __init__(self, left_motor: Motor, right_motor: Motor, 
                 navigator: Navigator, 
                 touch: TouchSensor, color: ColorSensor, sonic: UltrasonicSensor ):
        
        self.left_motor = left_motor # controls left tire
        self.right_motor = right_motor # controls right tire
        self.navigator = navigator # object to keep track of robot position
        self.touch = touch # touch sensor
        self.color = color # color sensor
        self.sonic = sonic # ultrasonic sensor

        self.queue = [] # priority queue
        heapq.heapify(self.queue)

        self.hasHitWall = touch.pressed() # True if sensor touched wall
        self.distanceToWall = sonic.distance() # distance to wall [mm] 
        self.fireDetected = color.ambient() # percentage value of ambient light [#]
        self.isFollowingWall = False # True if robot is currently fallowing a wall
        self.isWandering = False # True if Robot is in Wandering behavior

    def move(self, distance) -> None:
        """Moves robot a given distance in [mm]\n
        Args: distance (float): Distance to be traveled [mm]
        """

        # tire rotation angle
        angle =  (((distance) / TIRE_CIRCUMFERENCE) * FULL_ROTATION)*ERROR_FACTOR_DISTANCE 
        self.left_motor. run_angle(TIRE_RPM, angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, angle)
        log("Move: " + str(distance))


    def turn(self, angle) -> None:
        """Turns robot a given angle [deg]

        Args:
            angle (float): angle to turn robot [deg]
        """

        self.navigator.update_nav(angle) # notify naviagtor
        log("Turn: " + str(angle))
        
        # Calculate the angle needed to rotate the robot to be in the specified angle
        arc_length = 2 * M_PI * ROBOT_RADIUS_MM * (angle/360)
        new_angle =  ((arc_length) / TIRE_CIRCUMFERENCE) * FULL_ROTATION
        new_angle =  new_angle * ERROR_FACTOR_TURN
        
        # Turn tires
        self.left_motor. run_angle(TIRE_RPM, -new_angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, new_angle)
    
    def run(self):
        log("Running Until Stopped...")
        self.left_motor.run(TIRE_RPM)
        self.right_motor.run(TIRE_RPM)

    def stop(self):
        self.left_motor.brake()
        self.right_motor.brake()

    def backup(self):
        self.move(-200)
        self.turn(45)
        self.hasHitWall = False

    def follow_wall(self):
        self.isFollowingWall = True
        while self.isFollowingWall:
            self.run()
            self.update_sensors()
            if self.hasHitWall:
                self.queue.append(TouchBehavior())
                self.stop()
                self.isFollowingWall = False

            if self.fireDetected > FIRE_LIGHT_INTENSITY:
                self.queue.append(FireDetection())
                self.stop()
                self.isFollowingWall = False

    def wander(self):
        self.isWandering = True

        while self.isWandering:
            self.run()
            self.update_sensors()

            if self.hasHitWall:
                self.queue.append(TouchBehavior())
                self.stop()
                self.isWandering = False

            if self.distanceToWall < MIN_WALL_DISTANCE:
                if not any(isinstance(behavior, WallFollowing) for behavior in self.queue):
                    if self.isFollowingWall == False:
                        self.queue.append(WallFollowing())
                        self.stop()
                        self.isWandering = False

            if self.fireDetected > FIRE_LIGHT_INTENSITY:
                self.queue.append(FireDetection())
                self.stop()
                self.isWandering = False
        
    
    def execute_commands(self, robot_commands) -> None:
        """ Takes a list of commands in format ('move', <float>) or ('turn', <float>)
        Then executes these commands one-by-one

        Args:
            robot_commands (list[tuple[str, float]]): list of commands 
            in format ('move', <float>) or ('turn', <float>)
        """
        for command in robot_commands:
            action, value = command
            if action == 'move':
                self.move(value)
            elif action == 'turn':
                self.turn(value)

    def process_behavior(self):
        behavior = heapq.heappop(self.queue)
        
        if behavior.priority == 0:
            behavior.run(self)
            print("Finished Recalibrating position...")

        if behavior.priority == 1:
            behavior.run()
            print("Finished Fire Detection...")

        if behavior.priority == 2:
            behavior.run(self)
            print("Finished Following wall...")

        if behavior.priority == 3:
            behavior.run(self)
            print("Finished Wandering...")
            
    
    def update_sensors(self):

        self.hasHitWall = self.touch.pressed()
        self.distanceToWall = self.sonic.distance()
        self.fireDetected = self.color.ambient()

    def update_queue(self):

        if not self.queue:
            if not any(isinstance(behavior, Wander) for behavior in self.queue):
                self.queue.append(Wander())

        if self.hasHitWall:
            if not any(isinstance(behavior, TouchBehavior) for behavior in self.queue):
                self.queue.append(TouchBehavior())

        if self.distanceToWall < MIN_WALL_DISTANCE:
            if not any(isinstance(behavior, WallFollowing) for behavior in self.queue):
                if self.isFollowingWall == False:
                    self.queue.append(WallFollowing())

        if self.fireDetected > FIRE_LIGHT_INTENSITY:
            self.queue.append(FireDetection())
    

############################################################################################
