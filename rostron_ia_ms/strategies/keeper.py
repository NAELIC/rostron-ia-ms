from .strategies import Strategies

from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World

import numpy as np


class Keeper(Strategies):

    def __init__(self, robot_id):
        super().__init__()
        self.goTo = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id, 1)

    def order_robot(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goTo.publish(msg)

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
