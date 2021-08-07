from rclpy.node import Node
from rostron_utils.world import World
from rostron_utils.angle_radian import AngleRadian
import math
from math import sin, cos, pi
from rostron_interfaces.action import MoveTo

from rostron_navigation.primitive.move_to import MoveToStrategie
from rclpy.action.server import ServerGoalHandle

class Striker(Node):
    """
    A améliorer :
    * Gérer la puissance du kick en fonction de la distance demandée
    * Gerer le cas lorsqu'il faut tirer dans le sens opposé strict de la balle
      (il faut éviter la balle)
    * Bug lorsqu'on passe du moveTo au kick : le dribbler (spin power de Hardware) s'arrete 
      et decolle la balle du robot => empeche le tir car balle décollé (probleme en simu)
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
        return (pose[0]+self.distance_ball*cos(theta), pose[1]+self.distance_ball*sin(theta))

    def move_to_ball_and_kick(self, goal_handle: ServerGoalHandle):
        self.update_pose_ball()
        
        # initial params #
        pose=(-4.6,0.0)
        kick_type=1
        ##################

        robot = MoveToStrategie(self.id)
        ball_position=(World().ball.position.x,World().ball.position.y)
        vector_angle= (pose[0]-ball_position[0],
                        pose[1]-ball_position[1])
        vector_axis= (1,0) # right vector
        theta = AngleRadian.angle_between(vector_axis, vector_angle)
        robot.move_to(self.first_pose(ball_position,theta),theta)
        self.distance_ball=0.1
        robot.move_by([self.first_pose(ball_position, theta),self.ball_position,
                       self.goal_pose(ball_position, theta)],theta, True)
        
        # Move to pose after the ball without dribbling to keep the ball close to kicker
        robot.move_by([self.goal_pose(ball_position, theta)],theta) 
        robot.kick(kick_type)

        goal_handle.succeed()
        result = MoveTo.Result()
        return result
