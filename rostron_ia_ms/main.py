# from rostron_ia_ms.strategies.striker import Striker
from rostron_ia_ms.strategies.go_to import GoTo
from rostron_ia_ms.strategies.control import Control
import rclpy
from rclpy.node import Node
from .utils.task import Task

# TODO : Move with the new rostron_utils
from .utils.world import World

class IANode(Node):
    tasks = []

    def __init__(self):
        super().__init__("IANode")
        self.create_timer(0.16, self.update)

        self.declare_parameter('yellow', True)
        self.is_yellow = self.get_parameter('yellow').get_parameter_value().bool_value

        World().init(self, self.is_yellow)
        # print(thisRobot.pose.position.x)
        # thisRobot = World().allies[0]


        # self.tasks.append(GoTo(0, thisRobot.pose.position.x-0.5, 0.0, 0.0))
        # self.tasks.append(Control(0, 0, 0, 3.14))
        # self.tasks.append(Striker(0))
        # self.create_service() Manager
        # self.create_service() Strategies

    def update(self):
        for t in self.tasks:
            t.update()

def main():
    rclpy.init()
    node = IANode()
    rclpy.spin(node)
    rclpy.shutdown()