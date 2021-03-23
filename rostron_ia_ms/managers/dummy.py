from .manager import Manager
import rclpy

class Dummy(Manager):
    def __init__(self):
        super().__init__('dummy')

def main(args=None):
    rclpy.init(args=args)

    test = Dummy()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
