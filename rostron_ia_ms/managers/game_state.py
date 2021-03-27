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
    START = 2
    STOP = 3


class GameStateManager(ABC, Manager):
    last_gc_receive = None

    def __init__(self):
        super().__init__('manual')

    def new_command_receive(self):
        if last_gc_receive is not None and last_gc_receive == World.gc.command:
            return false
        else:
            return True

    def update(self):
        if self.new_command_receive():
            self.update_state()
        self.callback_state()

    def update_state(self):
        if World().gc.stage == Command.HALT:
            self.get_logger()
            internal_state_ == State.HALT
        elif World().gc.stage == Command.STOP:
            self.stop()

    def callback_state(self):
        if self.internal_state_ ==

    @abstractmethod
    def halt(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    def prepare_kickoff_blue():
