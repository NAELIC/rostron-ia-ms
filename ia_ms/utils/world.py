from rostron_interfaces.msg import Robots, Ball

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
    
    def update_allies(self, msg: Robots):
        self.allies_ = msg.robots

    def update_opponents(self, msg: Robots):
        self.allies_ = msg.robots

    def update_ball(self, msg: Ball):
        self.ball_ = msg