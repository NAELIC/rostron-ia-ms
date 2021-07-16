from rostron_utils.world import World
import math
from math import sin, cos, pi
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from rostron_interfaces.msg import Robot, Robots
from rostron_interfaces.msg import Order, Hardware

from rclpy.action.server import ServerGoalHandle


import time

from rostron_navigation.primitive.move_to import MoveToStrategie, R_Math

class GoalKeeper(Node):
    def __init__(self, id, distance_goal=0.4):
        super().__init__("goalkeeper")
        self.id = id
        self.distance_goal = distance_goal
        self.goal_center = (4.6,0.0)
        World().init(self)
    
    def goal_pose(self,pose,theta):
        """ Returns a pose close to the point and aligned with the angle theta"""
        return (pose[0]+self.distance_goal*cos(theta), pose[1]+self.distance_goal*sin(theta))

    def update_goal_keeper(self, goal_handle: ServerGoalHandle):
        """Create a new goalkeeper pose according to ball position, and move to it"""
        while(True):
            ball_position=(World().ball.position.x,World().ball.position.y)
            vector_angle= (ball_position[0]-self.goal_center[0],
                            ball_position[1]-self.goal_center[1])
            vector_axis= (1,0) # right vector
            orientation_goal = R_Math().angle_between(vector_axis, vector_angle)
            goal_pose = self.goal_pose(self.goal_center, orientation_goal)
            robot = MoveToStrategie(self.id)
            robot.move_to(goal_pose,orientation_goal)

def main(args=None):
    rclpy.init(args=args)
    robot=GoalKeeper(0)
    while(True):
        robot.update_goal_keeper()

if __name__ == '__main__':
    main()