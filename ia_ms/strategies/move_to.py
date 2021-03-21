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
        msg.pose.position.x = self.world.ball_.position.x
        msg.pose.position.y = self.world.ball_.position.y
        msg.pose.orientation = self.yaw_to_quaternion(0.0)
        self.goTo.publish(msg)
        
        return True
