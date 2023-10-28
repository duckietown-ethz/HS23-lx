from abc import ABC, abstractmethod
from threading import Thread
from typing import TypeVar, Generic

from ..exceptions import ComponentShutdown
from ..types import IQueue

InputType = TypeVar("InputType")
OutputType = TypeVar("OutputType")

__all__ = [
    "InputType",
    "OutputType",
    "Component",
]


class Component(Thread, ABC, Generic[InputType, OutputType]):

    def __init__(self):
        super().__init__()
        self._is_shutdown: bool = False

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    def stop(self):
        self._is_shutdown = True
        # stop all queues
        for attr_name in set(self.__dict__.keys()) - set(dir(self.__class__)):
            attr = getattr(self, attr_name)
            if isinstance(attr, IQueue):
                attr.stop()

    def run(self) -> None:
        try:
            self.worker()
        except ComponentShutdown:
            return
        except:
            raise

    @abstractmethod
    def worker(self):
        pass

    def __del__(self):
        self.stop()
