from rostron_interfaces.msg import Robots, Ball, Referee
from rclpy.node import Node


class SingletonMeta(type):
    """
    The Singleton class can be implemented in different ways in Python. Some
    possible methods include: base class, decorator, metaclass. We will use the
    metaclass because it is best suited for this purpose.
    """

    _instances = {}

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class World(metaclass=SingletonMeta):
    def init(self, node: Node, team : str):
        self.node_ = node

        node.create_subscription(
            Robots,
            '/%s/allies' % team,
            World().update_allies,
            10)
        node.create_subscription(
            Robots,
            '/%s/opponents' % team,
            World().update_opponents,
            10)

        node.create_subscription(
            Ball,
            '/%s/ball' % team,
            World().update_ball,
            10)

        node.create_subscription(
            Referee,
            '/%s/gc' % team,
            World().update_gc,
            10)

    def update_allies(self, msg: Robots):
        self.allies_ = msg.robots

    def update_opponents(self, msg: Robots):
        self.allies_ = msg.robots

    def update_ball(self, msg: Ball):
        self.ball_ = msg

    def update_gc(self, msg: Referee):
        self.node_.get_logger().info('receive gc')
        self.gc_ = msg
