"""responsible for coordinating the behaviors that the robot will take given the sensor data"""
from random import randint
import math
from logger import log
from globals import *
from pybricks.parameters import Color
import time

class RobotBehavior:
    def __init__(self, priority):
        self.priority = priority

    def run(self):
        pass

    def stop_behavior(self, robot):
        pass

    def __lt__(self, other):
        return self.priority < other.priority

class TouchBehavior(RobotBehavior):
    """Coordinates behaviors if robot touched a wall.
    Priority of 0
    """
    def __init__(self):
        super().__init__(0)

    def run(self, robot):
        if robot.hasHitFrontWall:
            self.recalibrate_front(robot)
        if robot.hasHitLeftWall:
            self.recalibrate_left(robot)

    def recalibrate_front(self, robot) -> None:
        """
        Given that the robot ran into a wall,
        backup the robot 200 mm and turn it 90 degrees.
        """
        random_angle = randint(45, 180)
        robot.move(-200)
        robot.turn(random_angle)
        robot.hasHitFrontWall = False

    def recalibrate_left(self, robot) -> None:
        """
        Given that the robot ran into a wall,
        backup the robot 200 mm and turn it 90 degrees.
        """
        random_angle = randint(45, 180)
        robot.turn(-random_angle)
        robot.hasHitLeftWall = False
    


class FireDetection(RobotBehavior):
    """Coordinates behaviors if robot sees a bright enough light.
    Priority of 1
    """
    def __init__(self):
        super().__init__(1)

    def run(self, robot):
        self.move_to_fire(robot)

    def move_to_fire(self, robot):

        robot.isFollowingFire = True
        previous_color = robot.current_color

        log("Current Color: " + str(robot.current_color))
        
        while robot.isFollowingFire:

            robot.move(50)
            robot.update_sensors()
            
            log("Curr Light " + str(robot.current_color) + " | Prev Light: " + str(previous_color))

            if robot.hasHitFrontWall:
                self.stop_behavior(robot, "Stopped follwoing fire because touched a wall in front")

            if robot.hasHitLeftWall:
                self.stop_behavior(robot, "Stopped follwoing fire because touched a wall to the left")

            if robot.current_color == Color.RED:
                self.stop_behavior(robot, "FIRE HAS BEEN FOUND!!!!")
                robot.fireNotFound = False

            if robot.current_color != Color.RED:
                self.stop_behavior(robot, "False alarm (Flase positive color detection)")
                
    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFollowingFire = False
        log(msg)
        
class WallFollowing(RobotBehavior):
    """Coordinates behaviors if robot gets close enough to the wall.
    Priority of 2.
    """
    def __init__(self):
        super().__init__(2)

    def run(self, robot):
        self.follow_wall(robot)

    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isFollowingWall = False
        log(msg)
    
    def align_robot_with_wall(self,robot):
        # Turn robot to orient itself with the wall
        angle_to_turn = math.degrees(math.atan(ROBOT_LENGTH / robot.distanceToWall))
        robot.turn(angle_to_turn)

    def follow_wall(self, robot) -> None:
        """
        Ensures the robot follows the wall until it detects that it should not 
        be following the wall anymore
        """
        self.align_robot_with_wall(robot)
        robot.isFollowingWall = True
        robot.wallFollowingDistance = robot.distanceToWall

        log("Current distance to wall: " + str(robot.distanceToWall) + " mm")


        while robot.isFollowingWall:
            robot.run()
            robot.update_sensors()

            log("Current distance to wall: " + str(robot.distanceToWall) + " | Starting Distance: to wall " + str(robot.wallFollowingDistance) , True)

            if robot.hasHitFrontWall:
                self.stop_behavior(robot, "Stopped follwoing wall because touched a wall to the front")

            if robot.hasHitLeftWall:
                self.stop_behavior(robot, "Stopped follwoing wall because touched a wall to the left")

            if robot.current_color == Color.RED:
                self.stop_behavior(robot, "Stopped Follwoing wall because of light detetcted")

            if (robot.distanceToWall < robot.wallFollowingDistance - 30) or (robot.distanceToWall > robot.wallFollowingDistance + 30):
                self.stop_behavior(robot, "Stopped follwoing wall because not near wall...")

        
class Wander(RobotBehavior):
    """Default behavior of robot. Performs these actions until 
    the robot sensors detect somtheing. Priorit of 3.
    """
    def __init__(self):
        super().__init__(3)
        self.start_time = None
        self.timeout_duration = 8
    

    def run(self, robot):
        self.wander(robot)

    def wander(self, robot):
        """
        Robot just moves forward until sensors 
        detect that a behavior should happen
        """
        robot.isWandering = True
        self.start_time = time.time()
        robot.isWandering = True

        while robot.isWandering:
            robot.run()
            robot.update_sensors()

            # Check if the timeout duration has elapsed
            if time.time() - self.start_time > self.timeout_duration:
                self.stop_behavior(robot, "Timeout reached while wandering... Assuming the Robot is stuck...")
                self.recalibrate_front(robot)

            if robot.hasHitFrontWall:
                self.stop_behavior(robot, "Stopped Wandering because touched wall in front...")

            if robot.hasHitLeftWall:
                self.stop_behavior(robot, "Stopped Wandering because touched a wall to the left...")

            if robot.distanceToWall < MIN_WALL_DISTANCE:
                self.stop_behavior(robot, "Stopped Wandering because close to wall...")

            if robot.current_color == Color.RED:
                self.stop_behavior(robot, "Stopped Wandering because light/fire detetcetd...")

    def stop_behavior(self, robot, msg):
        robot.stop()
        robot.isWandering = False
        log(msg)

    def recalibrate_front(self, robot) -> None:
        """
        Given that the robot ran into a wall,
        backup the robot 200 mm and turn it 90 degrees.
        """
        random_angle = randint(45, 180)
        robot.move(-200)
        robot.turn(random_angle)
