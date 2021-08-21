from abc import ABC, abstractmethod


class Task(ABC):
    @abstractmethod
    def update(self):
        pass
