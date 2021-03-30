from abc import ABC, abstractmethod
from .manager import Manager
from rostron_ia_ms.utils.world import World

from enum import Enum


class Command(Enum):
    HALT = 0
    STOP = 1
    NORMAL_START = 2
    FORCE_START = 3
    PREPARE_KICKOFF_YELLOW = 4
    PREPARE_KICKOFF_BLUE = 5
    PREPARE_PENALTY_YELLOW = 6
    PREPARE_PENALTY_BLUE = 7
    DIRECT_FREE_YELLOW = 8
    DIRECT_FREE_BLUE = 9
    INDIRECT_FREE_YELLOW = 10
    INDIRECT_FREE_BLUE = 11
    TIMEOUT_YELLOW = 12
    TIMEOUT_BLUE = 13
    BALL_PLACEMENT_YELLOW = 16
    BALL_PLACEMENT_BLUE = 17


class State(Enum):
    HALT = 0
    STOP = 1
    KICKOFF_ALLY = 2
    KICKOFF_OPPONENT = 3
    NORMAL_START = 4
    FORCE_START = 5
    RUNNING = 6
    PENALTY_ALLY = 7
    PENALTY_OPPONENT = 8
    DIRECT_FREE_ALLY = 9
    DIRECT_FREE_OPPONENT = 10
    INDIRECT_FREE_ALLY = 11
    INDIRECT_FREE_OPPONENT = 12
    TIMEOUT = 13


class GameStateManager(Manager, ABC):
    last_gc_receive = None
    internal_state_ = State.HALT

    def __init__(self, name: str):
        super().__init__(name)
        # self.start_halt()

    def state_change(self) -> bool:
        '''
            See if the state is changed since the last update.
        '''
        if World().gc.command is None or self.last_gc_receive == World().gc.command:
            return False
        else:
            self.last_gc_receive = World().gc.command
            return True

    def update(self):
        if not(World().ready()):
            self.get_logger().info('[WAITING] : All main topic not receive')
            return

        if self.state_change():
            self.callback_stop()
            self.update_state()
        self.callback_state()

    def callback_stop(self):
        if self.internal_state_ == State.HALT:
            self.stop_halt()
        elif self.internal_state_ == State.STOP:
            self.stop_stop()
        elif self.internal_state_ == State.KICKOFF_ALLY:
            self.stop_kickoff_ally()
        elif self.internal_state_ == State.KICKOFF_OPPONENT:
            self.stop_kickoff_opponent()
        elif self.internal_state_ == State.NORMAL_START:
            self.stop_normal_start()
        elif self.internal_state_ == State.FORCE_START:
            self.stop_force_start()
        elif self.internal_state_ == State.RUNNING:
            self.stop_running()
        elif self.internal_state_ == State.PENALTY_ALLY:
            self.stop_penalty_ally()
        elif self.internal_state_ == State.PENALTY_OPPONENT:
            self.stop_penalty_opponent()
        elif self.internal_state_ == State.DIRECT_FREE_ALLY:
            self.stop_direct_free_ally()
        elif self.internal_state_ == State.DIRECT_FREE_OPPONENT:
            self.stop_direct_free_opponent()
        elif self.internal_state_ == State.INDIRECT_FREE_ALLY:
            self.stop_indirect_free_ally()
        elif self.internal_state_ == State.INDIRECT_FREE_OPPONENT:
            self.stop_indirect_free_opponent()
        elif self.internal_state_ == State.TIMEOUT:
            self.stop_timeout()
        else:
            self.get_logger().warn('[STOP] Command not implemented : %d' % self.internal_state_)

    def update_state(self):
        if World().gc.command == Command.HALT.value:
            self.get_logger().info('State changed : HALT')
            self.internal_state_ = State.HALT
            self.start_halt()
        elif World().gc.command == Command.STOP.value:
            self.get_logger().info('State changed : STOP')
            self.internal_state_ = State.STOP
            self.start_stop()
        elif World().gc.command == Command.PREPARE_KICKOFF_BLUE.value:
            self.get_logger().info('State changed : PREPARE_KICKOFF_BLUE')
            if self.is_yellow:
                self.internal_state_ = State.KICKOFF_OPPONENT
                self.start_kickoff_opponent()
            else:
                self.internal_state_ == State.KICKOFF_ALLY
                self.start_kickoff_ally()
        elif World().gc.command == Command.PREPARE_KICKOFF_YELLOW.value:
            self.get_logger().info('State changed : PREPARE_KICKOFF_YELLOW')
            if self.is_yellow:
                self.internal_state_ = State.KICKOFF_ALLY
                self.start_kickoff_ally()
            else:
                self.internal_state_ == State.KICKOFF_OPPONENT
                self.start_kickoff_opponent()
        elif World().gc.command == Command.NORMAL_START.value:
            self.get_logger().info('State changed : NORMAL_START')
            self.internal_state_ = State.NORMAL_START
            self.start_normal_start()
        elif World().gc.command == Command.FORCE_START.value:
            self.get_logger().info('State changed : FORCE_START')
            self.internal_state_ = State.NORMAL_START
            self.start_force_start()
        elif World().gc.command == Command.PREPARE_PENALTY_BLUE.value:
            self.get_logger().info('State changed : PREPARE_PENALTY_BLUE')
            if self.is_yellow:
                self.internal_state_ = State.PENALTY_OPPONENT
                self.start_penalty_opponent()
            else:
                self.internal_state_ = State.PENALTY_ALLY
                self.start_penalty_ally()
        elif World().gc.command == Command.PREPARE_PENALTY_YELLOW.value:
            self.get_logger().info('State changed : PREPARE_PENALTY_YELLOW')
            if self.is_yellow:
                self.internal_state_ = State.PENALTY_ALLY
                self.start_penalty_ally()
            else:
                self.internal_state_ = State.PENALTY_OPPONENT
                self.start_penalty_opponent()
        elif World().gc.command == Command.DIRECT_FREE_BLUE.value:
            self.get_logger().info('State changed : DIRECT_FREE_BLUE')
            if self.is_yellow:
                self.internal_state_ = State.DIRECT_FREE_OPPONENT
                self.start_direct_free_opponent()
            else:
                self.internal_state_ = State.DIRECT_FREE_ALLY
                self.start_direct_free_ally()
        elif World().gc.command == Command.DIRECT_FREE_YELLOW.value:
            self.get_logger().info('State changed : DIRECT_FREE_YELLOW')
            if self.is_yellow:
                self.internal_state_ = State.DIRECT_FREE_ALLY
                self.start_direct_free_ally()
            else:
                self.internal_state_ = State.DIRECT_FREE_OPPONENT
                self.start_direct_free_opponent()
        elif World().gc.command == Command.INDIRECT_FREE_BLUE.value:
            self.get_logger().info('State changed : INDIRECT_FREE_BLUE')
            if self.is_yellow:
                self.internal_state_ = State.INDIRECT_FREE_OPPONENT
                self.start_indirect_free_opponent()
            else:
                self.internal_state_ = State.INDIRECT_FREE_ALLY
                self.start_indirect_free_ally()
        elif World().gc.command == Command.INDIRECT_FREE_YELLOW.value:
            self.get_logger().info('State changed : INDIRECT_FREE_YELLOW')
            if self.is_yellow:
                self.internal_state_ = State.INDIRECT_FREE_ALLY
                self.start_indirect_free_ally()
            else:
                self.internal_state_ = State.INDIRECT_FREE_OPPONENT
                self.start_indirect_free_opponent()
        elif World().gc.command == Command.TIMEOUT_BLUE.value or World().gc.command == Command.TIMEOUT_YELLOW.value:
            self.internal_state_ = State.TIMEOUT
            self.start_timeout()
        else:
            self.get_logger().warn(
                '[START] Command not implemented : %d' % World().gc.command)

    def callback_state(self):
        if self.internal_state_ == State.HALT:
            self.halt()
        elif self.internal_state_ == State.STOP:
            self.stop()
        elif self.internal_state_ == State.KICKOFF_ALLY:
            self.kickoff_ally()
        elif self.internal_state_ == State.KICKOFF_OPPONENT:
            self.kickoff_opponent()
        elif self.internal_state_ == State.NORMAL_START:
            self.normal_start()
        elif self.internal_state_ == State.FORCE_START:
            self.force_start()
        elif self.internal_state_ == State.RUNNING:
            self.running()
        elif self.internal_state_ == State.PENALTY_ALLY:
            self.penalty_ally()
        elif self.internal_state_ == State.PENALTY_OPPONENT:
            self.penalty_opponent()
        elif self.internal_state_ == State.DIRECT_FREE_ALLY:
            self.direct_free_ally()
        elif self.internal_state_ == State.DIRECT_FREE_OPPONENT:
            self.direct_free_opponent()
        elif self.internal_state_ == State.INDIRECT_FREE_ALLY:
            self.indirect_free_ally()
        elif self.internal_state_ == State.INDIRECT_FREE_OPPONENT:
            self.indirect_free_opponent()
        elif self.internal_state_ == State.TIMEOUT:
            self.timeout()
        else:
            self.get_logger().warn('[CALLBACK] Command not implemented : %d' % self.internal_state_)

    # This methods behind needs to be implemented for a match

    ################################################
    #                    HALT                      #
    ################################################
    @abstractmethod
    def start_halt(self):
        pass

    @abstractmethod
    def halt(self):
        pass

    @abstractmethod
    def stop_halt(self):
        pass

    ################################################
    #                    STOP                      #
    ################################################
    @abstractmethod
    def start_stop(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def stop_stop(self):
        pass

    ################################################
    #               KICKOFF OPPONENT               #
    ################################################
    @abstractmethod
    def start_kickoff_opponent(self):
        pass

    @abstractmethod
    def kickoff_opponent(self):
        pass

    @abstractmethod
    def stop_kickoff_opponent(self):
        pass

    ################################################
    #                 KICKOFF ALLY                 #
    ################################################
    @abstractmethod
    def start_kickoff_ally(self):
        pass

    @abstractmethod
    def kickoff_ally(self):
        pass

    @abstractmethod
    def stop_kickoff_ally(self):
        pass

    ################################################
    #               PENALTY OPPONENT               #
    ################################################
    @abstractmethod
    def start_penalty_opponent(self):
        pass

    @abstractmethod
    def penalty_opponent(self):
        pass

    @abstractmethod
    def stop_penalty_opponent(self):
        pass

    ################################################
    #                 PENALTY ALLY                 #
    ################################################
    @abstractmethod
    def start_penalty_ally(self):
        pass

    @abstractmethod
    def penalty_ally(self):
        pass

    @abstractmethod
    def stop_penalty_ally(self):
        pass

    ################################################
    #                NORMAL START                 #
    ################################################
    @abstractmethod
    def start_normal_start(self):
        pass

    @abstractmethod
    def normal_start(self):
        pass

    @abstractmethod
    def stop_normal_start(self):
        pass

    ################################################
    #                 FORCE START                  #
    ################################################
    @abstractmethod
    def start_force_start(self):
        pass

    @abstractmethod
    def force_start(self):
        pass

    @abstractmethod
    def stop_force_start(self):
        pass

    ################################################
    #                   RUNNING                    #
    ################################################
    @abstractmethod
    def start_running(self):
        pass

    @abstractmethod
    def running(self):
        pass

    @abstractmethod
    def stop_running(self):
        pass

    ################################################
    #               DIRECT FREE ALLY               #
    ################################################
    @abstractmethod
    def start_direct_free_ally(self):
        pass

    @abstractmethod
    def direct_free_ally(self):
        pass

    @abstractmethod
    def stop_direct_free_ally(self):
        pass

    ################################################
    #             DIRECT FREE OPPONENT             #
    ################################################
    @abstractmethod
    def start_direct_free_opponent(self):
        pass

    @abstractmethod
    def direct_free_opponent(self):
        pass

    @abstractmethod
    def stop_direct_free_opponent(self):
        pass

    ################################################
    #               INDIRECT FREE ALLY             #
    ################################################
    @abstractmethod
    def start_indirect_free_ally(self):
        pass

    @abstractmethod
    def indirect_free_ally(self):
        pass

    @abstractmethod
    def stop_indirect_free_ally(self):
        pass

    ################################################
    #              INDIRECT FREE OPPONENT          #
    ################################################
    @abstractmethod
    def start_indirect_free_opponent(self):
        pass

    @abstractmethod
    def indirect_free_opponent(self):
        pass

    @abstractmethod
    def stop_indirect_free_opponent(self):
        pass

    ################################################
    #                   TIMEOUT                    #
    ################################################
    @abstractmethod
    def start_timeout(self):
        pass

    @abstractmethod
    def timeout(self):
        pass

    @abstractmethod
    def stop_timeout(self):
        pass
