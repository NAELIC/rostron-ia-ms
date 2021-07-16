from rostron_utils.world import World
import math
from math import sin, cos, pi

from rclpy.node import Node
from rostron_interfaces.action import MoveTo

from rostron_navigation.primitive.move_to import MoveToStrategie, R_Math
from rclpy.action.server import ServerGoalHandle

class Dribbler(Node):
    """
    A améliorer :
    * Gestion de la puissance du spin power ??
    * Gerer le cas lorsqu'il faut dribbler dans le sens opposé strict de la balle
     (il faut éviter la balle)
    """
    def __init__(self, id):
        super().__init__("striker")
        self.id = id
        self.ball_position=(0.0,0.0)
        self.distance_ball = 0.2
        World().init(self)

    def update_pose_ball(self):
        """Update the values ​​of the ball position"""
        self.ball_position = (World().ball.position.x,World().ball.position.y)

    def first_pose(self,pose,theta):
        """ Returns a pose behind a position and aligned with the angle theta"""
        return (pose[0]+self.distance_ball*cos(theta-math.pi),
                pose[1]+self.distance_ball*sin(theta-math.pi))

    def goal_pose(self,pose,theta):
        """ Returns a pose close to the point and aligned with the angle theta"""
        return (pose[0]+self.distance_ball*cos(theta),
                pose[1]+self.distance_ball*sin(theta))

    def move_to_ball_and_dribble(self, goal_handle: ServerGoalHandle):
        self.update_pose_ball()

        goal : MoveTo.Goal = goal_handle.request
        # initial params #
        pose=(goal.position.x,goal.position.y)
        ##################

        robot = MoveToStrategie(self.id)
        ball_position=(World().ball.position.x,World().ball.position.y)
        vector_angle= (pose[0]-ball_position[0],
                        pose[1]-ball_position[1])
        vector_axis= (1,0) # right vector
        theta = R_Math().angle_between(vector_axis, vector_angle)
        robot.move_to(self.first_pose(ball_position,theta),theta)
        self.distance_ball=0.1
        robot.move_by([self.first_pose(ball_position, theta),self.ball_position,
                       self.goal_pose(ball_position, theta)],theta,True)
        robot.move_to(pose,theta,False,True)

        goal_handle.succeed()
        result = MoveTo.Result()
        return result