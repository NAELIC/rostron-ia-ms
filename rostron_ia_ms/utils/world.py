from rostron_interfaces.msg import Robots, Ball, Referee
from rclpy.node import Node
from rclpy.subscription import Subscription

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
    node_ : Node = None
    gc: Referee = None
    ball: Ball = None
    allies = []
    opponents = []

    p_ball_: Subscription = None
    p_allies_: Subscription = None
    p_opponents_: Subscription = None
    p_gc_: Subscription = None

    
    def init(self, node: Node, team : str):
        self.node_ = node

        # Robots
        self.p_allies_ = node.create_subscription(
            Robots,
            '/%s/allies' % team,
            self.update_allies,
            10)
        self.p_opponents_ = node.create_subscription(
            Robots,
            '/%s/opponents' % team,
            self.update_opponents,
            10)

        # Ball
        self.p_ball_ = node.create_subscription(
            Ball,
            '/%s/ball' % team,
            self.update_ball,
            10)

        # GameController
        self.p_gc_ = node.create_subscription(
            Referee,
            '/%s/gc' % team,
            self.update_gc,
            10)
    
    def update_allies(self, msg: Robots):
        self.allies = msg.robots

    def update_opponents(self, msg: Robots):
        self.opponents = msg.robots

    def update_ball(self, msg: Ball):
        self.ball = msg

    def update_gc(self, msg: Referee):
        self.gc = msg

