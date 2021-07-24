import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from rostron_interfaces.action import MoveTo

class MoveToActionClient(Node):
    def __init__(self):
        super().__init__('move_to_action_client')
        self._action_client = ActionClient(self, MoveTo, 'yellow/move_to')

    def move_to(self, id, position, orientation):
        goal_msg = MoveTo.Goal()
        goal_msg._id = id

        pos_msg = Point()
        pos_msg.x= position[0]
        pos_msg.y= position[1]
        goal_msg.position = pos_msg

        goal_msg.orientation = orientation
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveToActionClient()
    future = action_client.move_to(3,(0.0,2.0),0.0)
    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()
