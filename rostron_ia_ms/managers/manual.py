from .manager import Manager
import rclpy

from rostron_ia_ms.utils.world import World

from rostron_ia_ms.strategies.go_to_ball import GoToBall

class Manual(Manager):
    def __init__(self):
        super().__init__('dummy')
        World().init(self, self.team_)
        self.timer_ = self.create_timer(0.16, self.update)
        self.strategies = [GoToBall(0)]

    def update(self):
        # World._gc.sel
        while len(self.strategies) > 0:
            for num, strategie in enumerate(self.strategies):
                if strategie.update():
                    self.strategies.pop(num)


def main(args=None):
    rclpy.init(args=args)

    test = Manual()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
