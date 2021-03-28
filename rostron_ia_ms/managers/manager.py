import rclpy
from rclpy.node import Node

from rostron_ia_ms.utils.world import World


class Manager(Node):
    def __init__(self, name: str):
        super().__init__(name + '_manager')

        self.declare_parameter('team', 'yellow')
        self.team_ = self.get_parameter(
            'team').get_parameter_value().string_value

        self.declare_parameter('yellow', True)
        self.is_yellow = self.get_parameter(
            'yellow').get_parameter_value().bool_value

        World().init(self, self.is_yellow)
