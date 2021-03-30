from .strategies import Strategies
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World
from rostron_interfaces.msg import Hardware

import math
import time 

class Striker(Strategies):
    state_ = 0

    def __init__(self, robot_id):
        super().__init__()
        self.robot_id = robot_id
        self.goTo = World().node_.create_publisher(
            PoseStamped, 'robot_%d/goal_pose' % robot_id, 1)
        self.hardware = World().node_.create_publisher(
            Hardware, 'robot_%d/hardware_order' % robot_id, 1)
        

    def order_robot(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        if World().on_positive_half:
            print('change')
            msg.pose.position.x = -x
            msg.pose.position.y = -y
            msg.pose.orientation = self.yaw_to_quaternion(
                math.fmod(theta + math.pi, 2 * math.pi))
        else:
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation = self.yaw_to_quaternion(theta)
        self.goTo.publish(msg)

    def publish_kick(self, kick_type, kick_power, spin_power):
        msg = Hardware()
        msg.kick_type = kick_type
        msg.kick_power = kick_power
        msg.spin_power = spin_power

        self.hardware.publish(msg)

    def update(self):
        ball = np.array((World().ball.position.x, World().ball.position.y))
        robot = np.array(
            (World().allies[self.robot_id].pose.position.x, World().allies[self.robot_id].pose.position.y))
        goal_ennemy = np.array((4.5, 0))

        goal_ball =  goal_ennemy - ball 

        goal_ball = goal_ball / np.linalg.norm(goal_ball)
        orientation = math.atan2(-goal_ball[1], -goal_ball[0])
        target = [ball[0] - 0.5 * goal_ball[0], ball[1] - 0.5 * goal_ball[1]]
        # World().node_.get_logger().info('pass')

        if self.state_ == 0:
            self.order_robot(target[0], target[1], orientation)
            self.state_ += 1
            self.time = World().node_.get_clock().now().to_msg().sec
        elif self.state_ == 1:
            x = (target[0] - robot[0]) ** 2
            y = (target[1] - robot[1]) ** 2

            dist = np.sqrt(x + y)
            t =  World().node_.get_clock().now().to_msg().sec - self.time

            if dist - 0.03 < 0:
                World().node_.get_logger().info('pass')
                self.state_ = self.state_ + 1
            elif t > 5:
                self.order_robot(target[0], target[1], orientation)
                self.time = World().node_.get_clock().now().to_msg().sec



        elif self.state_ == 2:
            self.publish_kick(Hardware.FLAT_KICK, 1.0, 750.0)
            self.order_robot(ball[0] + 0.1 * goal_ball[0],
                             ball[1] + 0.1 * goal_ball[1], orientation)
            self.state_ = self.state_ + 1
            self.time = World().node_.get_clock().now().to_msg().sec
        else :
            World().node_.get_logger().info('finish')
            t =  World().node_.get_clock().now().to_msg().sec - self.time
            World().node_.get_logger().info('%f' % t)
            if t > 2:
                self.state_ = 0

        return False
