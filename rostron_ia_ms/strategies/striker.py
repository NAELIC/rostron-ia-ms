from .strategies import Strategies

from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World

class Striker(Strategies):
    state_ = 0

    def __init__(self, robot_id):
        super().__init__()
        self.goTo = World().node_.create_publisher(
            PoseStamped, '/yellow/robot_%d/goal_pose' % robot_id, 1)

    def update(self):
        if World().ball.position.x:

            if self.state_ == 0:
                self.goTo()
            elif self.state_ == 1:
                pass
            else:
                pass

        
            return False
        else:
            return False

