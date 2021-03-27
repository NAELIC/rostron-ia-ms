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
    HALT = 1
    STOP = 2
    KICKOFF_ALLY = 3
    KICKOFF_OPPONENT = 4
    NORMAL_START = 5
    FORCE_START = 6
    RUNNING = 7
    PENALTY_ALLY = 8
    PENALTY_OPPONENT = 9

class GameStateManager(ABC, Manager):
    last_gc_receive = None
    internal_state_ = None

    def __init__(self, name: str):
        super().__init__(name)

    def state_change(self) -> bool:
        '''
            See if the state is changed since the last update.
        '''
        if self.last_gc_receive is not None and self.last_gc_receive == World().gc.command:
            return False
        else:
            self.last_gc_receive == World().gc.command
            return True

    def update(self):
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

    def update_state(self):
        if World().gc.command == Command.HALT:
            self.get_logger().info('State changed : HALT')
            self.internal_state_ == State.HALT
            self.start_halt()
        elif World().gc.command == Command.STOP:
            self.get_logger().info('State changed : STOP')
            self.internal_state_ == State.STOP
            self.start_stop()
        elif World().gc.command == Command.PREPARE_KICKOFF_BLUE:
            self.get_logger().info('State changed : PREPARE_KICKOFF_BLUE')
            if self.is_yellow:
                self.internal_state_ == State.KICKOFF_OPPONENT
                self.start_kickoff_opponent()
            else:
                self.internal_state_ == State.KICKOFF_ALLY
                self.start_kickoff_ally()
        elif World().gc.command == Command.PREPARE_KICKOFF_YELLOW:
            self.get_logger().info('State changed : PREPARE_KICKOFF_YELLOW')
            if self.is_yellow:
                self.internal_state_ == State.KICKOFF_ALLY
                self.start_kickoff_ally()
            else:
                self.internal_state_ == State.KICKOFF_OPPONENT
                self.start_kickoff_opponent()
        elif World().gc.command == Command.NORMAL_START:
            self.get_logger().info('State changed : NORMAL_START')
            self.internal_state_ == State.NORMAL_START
            self.start_normal_start()
        elif World().gc.command == Command.FORCE_START:
            self.get_logger().info('State changed : NORMAL_START')
            self.internal_state_ == State.NORMAL_START
            self.start_force_start()
        elif World().gc.command == Command.PREPARE_PENALTY_BLUE:
            self.get_logger().info('State changed : PREPARE_PENALTY_BLUE')
            if self.is_yellow:
                self.internal_state_ == State.PENALTY_OPPONENT
                self.start_penalty_opponent()
            else:
                self.internal_state_ == State.PENALTY_ALLY
                self.start_penalty_ally()
        elif World().gc.command == Command.PREPARE_PENALTY_YELLOW:
            self.get_logger().info('State changed : PREPARE_PENALTY_YELLOW')
            if self.is_yellow:
                self.internal_state_ == State.PENALTY_ALLY
                self.start_penalty_ally()
            else:
                self.internal_state_ == State.PENALTY_OPPONENT
                self.start_penalty_opponent()

    def callback_state(self):
        if self.internal_state_ == State.HALT:
            self.halt()
        elif self.internal_state_ == State.STOP:
            self.stop()
        elif self.internal_state_ == State.KICKOFF_ALLY:
            self.kickoff_ally()
        elif self.internal_state_ == State.KICKOFF_OPPONENT:
            self.kickoff_opponent()
        elif self.internal_state_ == State.FORCE_START:
            self.force_start()
        elif self.internal_state_ == State.RUNNING:
            self.running()
        elif self.internal_state_ == State.PENALTY_ALLY:
            self.penalty_ally()
        elif self.internal_state_ == State.PENALTY_OPPONENT:
            self.penalty_opponent()

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
