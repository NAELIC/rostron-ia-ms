from abc import ABC, abstractmethod
from math import sin, cos, pi
from ..utils.world import World
from geometry_msgs.msg import PoseStamped, Quaternion

class Strategies(ABC):
    
    @abstractmethod
    def update(self):
        pass
    
    # def euler_to_quaternion(self, roll, pitch, yaw):
    #     qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
    #         cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    #     qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
    #         sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    #     qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
    #         sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    #     qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
    #         sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    #     return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    # Todo(@Etienne) : Move this own class ?
    def yaw_to_quaternion(self, yaw):
        qx = cos(yaw/2) - sin(yaw/2)
        qy = cos(yaw/2) + sin(yaw/2)
        qz = sin(yaw/2) - cos(yaw/2)
        qw = cos(yaw/2) + sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
