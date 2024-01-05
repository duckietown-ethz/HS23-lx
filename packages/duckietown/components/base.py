from abc import ABC, abstractmethod
from threading import Thread
from typing import TypeVar, Generic, Optional, List, Iterator

from ..exceptions import ComponentShutdown
from ..types import IQueue

InputType = TypeVar("InputType")
OutputType = TypeVar("OutputType")

__all__ = [
    "InputType",
    "OutputType",
    "Component",
    "IComponent"
]


class IComponent(Generic[InputType, OutputType], ABC):

    def __init__(self):
        self._is_shutdown: bool = False
        self._is_started: bool = False

    @property
    def id(self) -> str:
        return str(id(self))

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    @property
    def is_started(self) -> bool:
        return self._is_started

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def join(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def reset(self):
        pass

    @property
    @abstractmethod
    def queues(self) -> List[IQueue]:
        pass

    @abstractmethod
    def worker(self):
        pass

    def __del__(self):
        self.stop()


class Component(IComponent[InputType, OutputType], ABC):

    def __init__(self):
        super(Component, self).__init__()
        self._thread: Optional[Thread] = None

    def start(self):
        if self.is_started:
            raise RuntimeError("You cannot start a Component twice.")
        self.reset()
        self._thread = Thread(target=self.run)
        self._thread.start()
        self._is_started = True

    def join(self):
        if self.is_started:
            self._thread.join()

    def stop(self):
        self._is_shutdown = True
        # stop all queues
        for queue in self.queues:
            queue.stop()
        self._is_started = False

    def reset(self):
        if self.is_started:
            raise RuntimeError("You must shutdown a component before you can reset it")
        self._is_shutdown = False
        self._is_started = False
        self._thread = None
        for queue in self.queues:
            queue.reset()

    @property
    def queues(self) -> Iterator[IQueue]:
        # return all queues
        for attr_name in set(self.__dict__.keys()) - set(dir(self.__class__)):
            attr = getattr(self, attr_name)
            if isinstance(attr, IQueue):
                yield attr

    def run(self) -> None:
        try:
            self.worker()
        except ComponentShutdown:
            return
        except:
            raise
        finally:
            self._thread = None
