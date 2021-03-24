from .strategies import Strategies

from geometry_msgs.msg import Point, PoseStamped
from rostron_ia_ms.utils.world import World

class GoToBall(Strategies):

    def __init__(self, robot_id):
        super().__init__()
        self.goTo = World().node_.create_publisher(
            PoseStamped, '/yellow/robot_%d/goal_pose' % robot_id, 1)

    def update(self):
        if World().ball.position.x:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = World().ball.position.x
            msg.pose.position.y = World().ball.position.y
            msg.pose.orientation = self.yaw_to_quaternion(0.0)
            self.goTo.publish(msg)
        
            return False
        else:
            return False

