import rclpy

from .game_state import GameStateManager, State
from std_msgs.msg import Bool

from rostron_ia_ms.strategies.go_to import GoTo


class ExempleGC(GameStateManager):

    strategies = []

    def __init__(self):
        super().__init__('exemple')
        self.timer_ = self.create_timer(0.16, self.update)
        self.halt = self.create_publisher(Bool, 'halt', 1)

    def update(self):
        super().update()

    ################################################
    #                    HALT                      #
    ################################################

    def start_halt(self):
        self.get_logger().info('[START] HALT')
        msg = Bool()
        msg.data = True
        self.halt.publish(msg)
        pass

    def halt(self):
        self.get_logger().info('[CONTINUE] HALT')
        msg = Bool()
        msg.data = True
        self.halt.publish(msg)
        pass

    def stop_halt(self):
        self.get_logger().info('[STOP] HALT')
        msg = Bool()
        msg.data = False
        for _ in range(5):
            self.halt.publish(msg)
        pass

    ################################################
    #                    STOP                      #
    ################################################

    def start_stop(self):
        self.get_logger().info('[START] STOP')
        self.strategies = [
            GoTo(0, -3.80, 0, 0),
            GoTo(1, -0.7, 0, 0),
            GoTo(2, -1.5, 1.12, 0),
            GoTo(3, -1.5, -1.12, 0),
            GoTo(4, -2.5, 0.7, 0),
            GoTo(5, -2.5, -0.7, 0)
        ]
        pass

    def stop(self):
        self.get_logger().info('[CONTINUE] STOP')
        for strategie in self.strategies:
            strategie.update()

    def stop_stop(self):
        self.get_logger().info('[STOP] STOP')
        pass

    ################################################
    #               KICKOFF OPPONENT               #
    ################################################

    def start_kickoff_opponent(self):
        self.get_logger().info('[START] KICKOFF OPPONENT')
        self.strategies = [
            GoTo(0, -3.80, 0, 0),
            GoTo(1, -0.7, 0, 0),
            GoTo(2, -1.5, 1.12, 0),
            GoTo(3, -1.5, -1.12, 0),
            GoTo(4, -2.5, 0.7, 0),
            GoTo(5, -2.5, -0.7, 0)
        ]
        pass

    def kickoff_opponent(self):
        self.get_logger().info('[CONTINUE] KICKOFF OPPONENT')
        for strategie in self.strategies:
            strategie.update()
        pass

    def stop_kickoff_opponent(self):
        self.get_logger().info('[STOP] KICKOFF OPPONENT')
        pass

    ################################################
    #                 KICKOFF ALLY                 #
    ################################################

    def start_kickoff_ally(self):
        self.get_logger().info('[START] KICKOFF ALLY')
        self.strategies = [
            GoTo(0, -3.80, 0, 0),
            GoTo(1, -0.3, 0, 0),
            GoTo(2, -1.5, 1.12, 0),
            GoTo(3, -1.5, -1.12, 0),
            GoTo(4, -2.5, 0.7, 0),
            GoTo(5, -2.5, -0.7, 0)
        ]
        pass

    def kickoff_ally(self):
        self.get_logger().info('[CONTINUE] KICKOFF ALLY')
        for strategie in self.strategies:
            strategie.update()
        pass

    def stop_kickoff_ally(self):
        self.get_logger().info('[STOP] KICKOFF ALLY')
        pass

    ################################################
    #               PENALTY OPPONENT               #
    ################################################

    def start_penalty_opponent(self):
        self.get_logger().info('[START] PENALTY OPPONENT')
        pass

    def penalty_opponent(self):
        self.get_logger().info('[CONTINUE] PENALTY OPPONENT')
        for strategie in self.strategies:
            strategie.update()
        pass

    def stop_penalty_opponent(self):
        self.get_logger().info('[STOP] PENALTY OPPONENT')
        pass

    ################################################
    #                 PENALTY ALLY                 #
    ################################################

    def start_penalty_ally(self):
        self.get_logger().info('[START] PENALTY ALLY')
        pass

    def penalty_ally(self):
        self.get_logger().info('[CONTINUE] PENALTY ALLY')
        for strategie in self.strategies:
            strategie.update()
        pass

    def stop_penalty_ally(self):
        self.get_logger().info('[STOP] PENALTY ALLY')
        pass

    ################################################
    #                NORMAL START                 #
    ################################################

    def start_normal_start(self):
        self.get_logger().info('[START] NORMAL START')
        pass

    def normal_start(self):
        self.get_logger().info('[CONTINUE] NORMAL START')
        for strategie in self.strategies:
            strategie.update()
        pass

    def stop_normal_start(self):
        self.get_logger().info('[STOP] NORMAL START')
        pass

    ################################################
    #                 FORCE START                  #
    ################################################

    def start_force_start(self):
        self.get_logger().info('[START] FORCE START')
        self.internal_state_ = State.RUNNING
        self.start_running()
        pass

    def force_start(self):
        self.get_logger().info('[CONTINUE] FORCE START')
        self.internal_state_ = State.RUNNING
        self.start_running()
        pass

    def stop_force_start(self):
        self.get_logger().info('[STOP] FORCE START')
        self.internal_state_ = State.RUNNING
        self.start_running()
        pass

    ################################################
    #                   RUNNING                    #
    ################################################

    def start_running(self):
        self.get_logger().info('[START] RUNNING')
        pass

    def running(self):
        self.get_logger().info('[CONTINUE] RUNNING')
        pass

    def stop_running(self):
        self.get_logger().info('[STOP] RUNNING')
        pass

    ################################################
    #               DIRECT FREE ALLY               #
    ################################################

    def start_direct_free_ally(self):
        self.get_logger().info('[START] DIRECT FREE ALLY')
        pass

    def direct_free_ally(self):
        self.get_logger().info('[CONTINUE] DIRECT FREE ALLY')
        pass

    def stop_direct_free_ally(self):
        self.get_logger().info('[STOP] DIRECT FREE ALLY')
        pass

    ################################################
    #             DIRECT FREE OPPONENT             #
    ################################################

    def start_direct_free_opponent(self):
        self.get_logger().info('[START] DIRECT FREE OPPONENT')
        pass

    def direct_free_opponent(self):
        self.get_logger().info('[CONTINUE] DIRECT FREE ALLY')
        pass

    def stop_direct_free_opponent(self):
        self.get_logger().info('[STOP] DIRECT FREE ALLY')
        pass

    ################################################
    #               INDIRECT FREE ALLY             #
    ################################################

    def start_indirect_free_ally(self):
        self.get_logger().info('[START] INDIRECT FREE ALLY')
        pass

    def indirect_free_ally(self):
        self.get_logger().info('[CONTINUE] INDIRECT FREE ALLY')
        pass

    def stop_indirect_free_ally(self):
        self.get_logger().info('[STOP] INDIRECT FREE ALLY')
        pass

    ################################################
    #              INDIRECT FREE OPPONENT          #
    ################################################

    def start_indirect_free_opponent(self):
        self.get_logger().info('[START] INDIRECT FREE OPPONENT')
        pass

    def indirect_free_opponent(self):
        self.get_logger().info('[CONTINUE] INDIRECT FREE OPPONENT')
        pass

    def stop_indirect_free_opponent(self):
        self.get_logger().info('[STOP] INDIRECT FREE OPPONENT')
        pass

    ################################################
    #                   TIMEOUT                    #
    ################################################

    def start_timeout(self):
        self.get_logger().info('[START] TIMEOUT')
        pass

    def timeout(self):
        self.get_logger().info('[CONTINUE] TIMEOUT')
        pass

    def stop_timeout(self):
        self.get_logger().info('[STOP] TIMEOUT')
        pass


def main(args=None):
    rclpy.init(args=args)

    test = ExempleGC()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
