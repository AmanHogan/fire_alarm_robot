"""responsible for coordinating the behaviors that the robot will take given the sensor data"""
import heapq
import math
from globals import *

class RobotBehavior:
    def __init__(self, priority):
        self.priority = priority

    def run(self):
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
        print("Touched a Wall!!!!")
        robot.backup()

class FireDetection(RobotBehavior):
    """Coordinates behaviors if robot sees a bright enough light.
    Priority of 1
    """
    def __init__(self):
        super().__init__(1)

    def run(self):
        print("Detected a Fire!!!")
        
class WallFollowing(RobotBehavior):
    """Coordinates behaviors if robot gets close enough to the wall.
    Priority of 2.
    """
    def __init__(self):
        super().__init__(2)

    def run(self, robot):
        print("Following Wall...")
        self.parallel_to_wall(robot)
    
    def parallel_to_wall(self, robot):
        angle_to_turn = math.degrees(math.atan(ROBOT_LENGTH / robot.distanceToWall))
        robot.turn(angle_to_turn)
        print(angle_to_turn)
        robot.follow_wall()
        print("Finished Following wall...")


class Wander(RobotBehavior):
    """Default behavior of robot. Performs these actions until 
    the robot sensors detect somtheing. Priorit of 3.
    """
    def __init__(self):
        super().__init__(3)

    def run(self, robot):
        print("Wandering...")
        robot.wander()


