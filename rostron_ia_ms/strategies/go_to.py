from .strategies import Strategies
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World
from rostron_interfaces.msg import Hardware

import math


class GoTo(Strategies):
    def __init__(self, robot_id, x, y, theta):
        super().__init__()
        self.x = x
        self.y = y
        self.theta = theta
        self.goTo = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id, 1)

    def order_robot(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        if World().on_positive_half:
            msg.pose.position.x = -x
            msg.pose.position.y = -y
            msg.pose.orientation = self.yaw_to_quaternion(
                math.fmod(theta + math.pi, 2 * math.pi))
        else:
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation = self.yaw_to_quaternion(theta)
        self.goTo.publish(msg)

    def update(self):
        self.order_robot(self.x, self.y, self.theta)
        return True
