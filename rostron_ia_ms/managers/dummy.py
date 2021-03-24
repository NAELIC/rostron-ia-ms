from .manager import Manager
import rclpy
from rostron_ia_ms.utils.world import World


class Dummy(Manager):
    def __init__(self):
        super().__init__('dummy')
        World().init(self, self.team_)
        self.timer_ = self.create_timer(0.16, self.update)

    def update(self):
        # World._gc.sel
        if World().opponents:
            self.get_logger().info("%f" % World().opponents[0].pose.position.x)


def main(args=None):
    rclpy.init(args=args)

    test = Dummy()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
