from .strategies import Strategies

from geometry_msgs.msg import Point, PoseStamped


class MoveTo(Strategies):

    def __init__(self, robot_id, position : Point, orientation, node):
        super().__init__()
        self.goTo = node.create_publisher(
            PoseStamped, '/yellow/robot_%d/goal_pose' % robot_id, 1)

    def update(self):
        self.world.ball_
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.orientation = self.euler_to_quaternion(0.0, 0.0, 0.0)
        self.goTo.publish(msg)
        
        return True
