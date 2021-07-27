from rostron_utils.world import World
from rostron_utils.angle_radian import AngleRadian
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from rostron_interfaces.action import MoveTo

class MoveToActionClient(Node):
    def __init__(self):
        super().__init__('move_to_action_client')
        self._action_client = ActionClient(self, MoveTo, 'yellow/move_to')
        World().init(self)

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


class GoalKeeper(Node):
    def __init__(self, id, distance_goal=0.4):
        super().__init__("goalkeeper")
        self.id = id
        self.distance_goal = distance_goal
        self.goal_center = (4.6,0.0)
        World().init(self)
        self.ball_position = (0.0,0.0) # Initial pose
        self.goal_already_placed = False
        self.robot = MoveToActionClient()
    
    def goal_pose(self,pose,theta):
        """ Returns a pose close to the point and aligned with the angle theta"""
        return (pose[0]+self.distance_goal*cos(theta), pose[1]+self.distance_goal*sin(theta))
    
    def update_position(self):
        """Create a new goalkeeper pose according to ball position, and move to it"""
        while True:
            if self.goal_already_placed : 
                rclpy.spin_once(self)
                self.ball_position=(World().ball.position.x,World().ball.position.y)
            else :
                self.goal_already_placed= True
                self.ball_position=(0.0,0.0)
            self.ball_position=(World().ball.position.x,World().ball.position.y)
            vector_angle= (self.ball_position[0]-self.goal_center[0],
                            self.ball_position[1]-self.goal_center[1])
            vector_axis= (1,0) # right vector
            orientation_goal = AngleRadian().angle_between(vector_axis, vector_angle)
            goal_pose = self.goal_pose(self.goal_center, orientation_goal)
            
            action = self.robot.move_to(self.id, goal_pose, orientation_goal)
            rclpy.spin_until_future_complete(self.robot,action)
        

def main(args=None):
    rclpy.init(args=args)
    goalkeeper=GoalKeeper(5) # id = 5
    goalkeeper.update_position()

if __name__ == '__main__':
    main()