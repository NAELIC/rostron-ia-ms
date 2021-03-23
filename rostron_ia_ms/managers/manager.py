import rclpy
from rclpy.node import Node

from rostron_ia_ms.utils.world import World


class Manager(Node):
    def __init__(self, name: str):
        super().__init__(name + 'manager')

        self.declare_parameter('team', 'yellow')
        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value
        World().init(self, self.team_)
