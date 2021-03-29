from .strategies import Strategies

from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World

import numpy as np


class Wall(Strategies):

    def __init__(self, robot_id):
        super().__init__()
        self.goTo1 = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id[0], 1)

                self.goTo2 = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id[1], 1)
                    self.goTo3 = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id[2], 1)
    
        

    def order_robot(self, x, y, theta):
        msg1 = PoseStamped()
        msg1.header.frame_id = 'map'
        msg1.pose.position.x = x
        msg1.pose.position.y = y
        msg1.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goTo1.publish(msg1)
        msg2 = PoseStamped()
        msg2.header.frame_id = 'map'
        msg2.pose.position.x = x
        msg2.pose.position.y = y-0.5
        msg2.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goTo1.publish(msg2)
        msg3 = PoseStamped()
        msg3.header.frame_id = 'map'
        msg3.pose.position.x = x
        msg3.pose.position.y = y+0.5
        msg3.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goTo1.publish(msg3)



    def vector2angle(self, v):
        norm = np.linalg.norm(v)
        if norm == 0.0:
            return 0.0

        v_norm = v / norm
        angle = np.arccos(v_norm[0])
        if v[1] < 0:
            angle = -angle
        return angle

    def update(self):
        ball = np.array((World().ball.position.x, World().ball.position.y))
        robot = np.array((World().allies[0].pose.position.x, World().allies[0].pose.position.y))
        target = np.array((4.5, 0))

        target_ball = target - ball
        target_ball = target_ball / np.linalg.norm(target_ball)

        orientation = self.vector2angle(ball - robot)

        position = target - (target_ball * 0.4)

        print(orientation)

        self.order_robot(position[0], position[1], orientation)

        return False

