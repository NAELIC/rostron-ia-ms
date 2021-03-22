import rclpy
from rclpy.node import Node
from rostron_interfaces.msg import Robots, Ball

from .utils.world import World

from .strategies.move_to import MoveTo

from geometry_msgs.msg import Point


class TestStrategie(Node):
    def __init__(self):
        super().__init__('test_strategies')
        self.declare_parameter('team', 'yellow')
        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value
        self.timer_ = self.create_timer(3, self.callback)

        self.subscription = self.create_subscription(
            Robots,
            '/%s/allies' % self.team_,
            World().update_allies,
            10)

        self.subscription = self.create_subscription(
            Robots,
            '/%s/opponents' % self.team_,
            World().update_opponents,
            10)
        self.subscription = self.create_subscription(
            Ball,
            '/%s/ball' % self.team_,
            World().update_ball,
            10)

        p = Point()
        self.strategy_1 = MoveTo(
            robot_id=0, position=p, orientation=3.14, node=self)

    def callback(self):
        while self.strategy_1.update():
            pass


def main(args=None):
    rclpy.init(args=args)

    test = TestStrategie()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
