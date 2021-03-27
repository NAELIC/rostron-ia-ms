import rclpy

from .game_state import GameStateManager


class ExempleGC(GameStateManager):
    def __init__(self):
        super().__init__('exemple')
        self.timer_ = self.create_timer(0.16, self.update)

    def update(self):
        super().update()

    ################################################
    #                    HALT                      #
    ################################################

    def start_halt(self):
        self.get_logger().info('[START] HALT')
        pass

    def halt(self):
        self.get_logger().info('[CONTINUE] HALT')
        pass

    def stop_halt(self):
        self.get_logger().info('[STOP] HALT')
        pass

    ################################################
    #                    STOP                      #
    ################################################

    def start_stop(self):
        self.get_logger().info('[START] STOP')
        pass

    def stop(self):
        self.get_logger().info('[CONTINUE] STOP')
        pass

    def stop_stop(self):
        self.get_logger().info('[STOP] STOP')
        pass

    ################################################
    #               KICKOFF OPPONENT               #
    ################################################

    def start_kickoff_opponent(self):
        self.get_logger().info('[START] KICKOFF OPPONENT')
        pass

    def kickoff_opponent(self):
        self.get_logger().info('[CONTINUE] KICKOFF OPPONENT')
        pass

    def stop_kickoff_opponent(self):
        self.get_logger().info('[STOP] KICKOFF OPPONENT')
        pass

    ################################################
    #                 KICKOFF ALLY                 #
    ################################################

    def start_kickoff_ally(self):
        self.get_logger().info('[START] KICKOFF ALLY')
        pass

    def kickoff_ally(self):
        self.get_logger().info('[CONTINUE] KICKOFF ALLY')
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
        pass

    def stop_normal_start(self):
        self.get_logger().info('[STOP] NORMAL START')
        pass

    ################################################
    #                 FORCE START                  #
    ################################################

    def start_force_start(self):
        self.get_logger().info('[START] FORCE START')
        pass

    def force_start(self):
        self.get_logger().info('[CONTINUE] FORCE START')
        pass

    def stop_force_start(self):
        self.get_logger().info('[STOP] FORCE START')
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
