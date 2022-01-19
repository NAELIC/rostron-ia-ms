from rostron_ia_ms.strategies.strategies import Strategies
from rostron_ia_ms.utils.world import World
from rclpy.action import ActionClient
from rostron_interfaces.action import Behavior

import json

def create_move_to(x, y, theta):
    msg = Behavior.Goal()

    msg.name = "move_to"
    msg.params = json.dumps({"x": x, "y": y, "theta": theta})

    return msg

class GoTo(Strategies):
    def __init__(self, id: float, x: float, y: float, theta: float) -> None:
        self.client_ = ActionClient(
            World().node_, Behavior, f"robot_{id}/behavior")

        self.client_.wait_for_server()
        self.client_.send_goal_async(create_move_to(x, y, theta))

    def update(self):
        pass
