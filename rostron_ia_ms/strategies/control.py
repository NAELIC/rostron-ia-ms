import math
from .strategies import Strategies

from rostron_ia_ms.utils.world import World
from rostron_interfaces.msg import Commands, Command, Hardware, Robot

import numpy as np
import math

class Control(Strategies):
    def __init__(self, id: float, x: float, y: float, theta: float) -> None:
        self.x = x
        self.id = id
        self.y = y
        self.theta =  theta
        self.publisher_ = World().node_.create_publisher(Commands, 'commands', 1)

    def command(self, x, y):
        commands = Commands()
        command = Command()
        command.id = self.id
        hardware = Hardware()
        hardware.kick_power = 0.0
        hardware.kick_type = Hardware.NO_KICK
        command.hardware = hardware
        command.velocity.linear.x = x
        command.velocity.linear.y = y
        command.velocity.angular.z = 0.0
        commands.commands.append(command)
        self.publisher_.publish(commands)
        
    def update(self):
        robot : Robot = World().allies[self.id]
        rob_target_x = robot.pose.position.x - self.x
        rob_target_y = robot.pose.position.y - self.y

        self.command(rob_target_x, rob_target_y)

        
