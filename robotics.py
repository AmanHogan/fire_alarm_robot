"""Acts as a replacement for pybricks robotics module. Module is
responsible for moving the robot, angle tracking, and robot behavior proccesing"""

from random import randint
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
        """
        Updates the logical orientation of the robot and keeps track of previous positions\n
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
        self.wallFollowingDistance = 0
        self.fireDetected = color.ambient() # percentage value of ambient light [#]
        self.isFollowingWall = False # True if robot is currently fallowing a wall
        self.isWandering = False # True if Robot is in Wandering behavior

    def move(self, distance) -> None:
        """
        Moves robot a given distance in [mm]\n
        Args: distance (float): Distance to be traveled [mm]
        """
        # tire rotation angle
        angle =  (((distance) / TIRE_CIRCUMFERENCE) * FULL_ROTATION)*ERROR_FACTOR_DISTANCE 
        self.left_motor. run_angle(TIRE_RPM, angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, angle)
        log("Move: " + str(distance))

    def turn(self, angle) -> None:
        """
        Turns robot a given angle [deg]\n
        Args: angle (float): angle to turn robot [deg]
        """

        # Calculate steering angle
        steering_angle =  (((2 * M_PI * ROBOT_RADIUS_MM * (angle/360)) / 
                            TIRE_CIRCUMFERENCE) * FULL_ROTATION) * ERROR_FACTOR_TURN
        
        # Turn tires
        self.left_motor. run_angle(TIRE_RPM, -steering_angle, wait=False)
        self.right_motor.run_angle(TIRE_RPM, steering_angle)

        self.navigator.update_nav(angle) # notify naviagtor
        log("Turn: " + str(angle))
        
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

    def backup(self) -> None:
        """
        Given that the robot ran into a wall,
        backup the robot 200 mm and turn it 90 degrees.
        """
        random_angle = randint(45, 180)
        self.move(-200)
        self.turn(random_angle)
        self.hasHitWall = False

    def follow_wall(self) -> None:
        """
        Ensures the robot follows the wall until it detects that it should not 
        be following the wall anymore
        """
        self.isFollowingWall = True
        self.wallFollowingDistance = self.distanceToWall

        while self.isFollowingWall:
            self.run()
            self.update_sensors()

            if self.hasHitWall:
                self.queue.append(TouchBehavior())
                self.stop()
                print("Stopped Follwoing wall because of hittinh wall")
                self.isFollowingWall = False

            if self.fireDetected > FIRE_LIGHT_INTENSITY:
                self.queue.append(FireDetection())
                self.stop()
                print("Stopped Follwoing wall because of fire detetcted ")
                self.isFollowingWall = False

            log("Distance to wall: " + str(self.distanceToWall) + "Following Distance: " + str(self.wallFollowingDistance))
            if (self.distanceToWall < self.wallFollowingDistance - 50) or (self.distanceToWall > self.wallFollowingDistance + 50):
                self.queue.append(Wander())
                self.stop()
                print("Stopped Follwoing wall beause sensor out of range ")
                self.isFollowingWall = False


    def wander(self):
        """
        Robot just moves forward until sensors 
        detect that a behavior should happen
        """
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

    def process_behavior(self) -> None:
        """
        Proccess a behavior from the priority queue. Pops the behavior from
        queue before it is proccessed.
        """
        behavior = heapq.heappop(self.queue)
        print(self.queue)
        
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
            
    
    def update_sensors(self) -> None:
        """Function updates the robot's sensor values and stores these values.
        """
        self.hasHitWall = self.touch.pressed()
        self.distanceToWall = self.sonic.distance()
        self.fireDetected = self.color.ambient()

    def update_queue(self) -> None:
        """Updates the priority queue using the robot's sensor values
        """
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
            if not any(isinstance(behavior, FireDetection) for behavior in self.queue):
                self.queue.append(FireDetection())
    
############################################################################################
