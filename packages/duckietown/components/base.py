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
    "IComponent"
]


class IComponent(Generic[InputType, OutputType], ABC):

    def __init__(self):
        self._is_shutdown: bool = False

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    @property
    def id(self) -> str:
        return str(id(self))

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def join(self):
        pass

    @abstractmethod
    def worker(self):
        pass

    def __del__(self):
        self.stop()



# TODO: investigate removing Thread as subclass of component and have the component define start() where a
#  new Thread is spawned if one is not available yet and follows it.
#  This avoids the 'RuntimeError: threads can only be started once' and be able to restart a Thread without
#  the need to recreate the component.


class Component(Thread, IComponent[InputType, OutputType], ABC):

    def __init__(self):
        super(Component, self).__init__()
        IComponent.__init__(self)

    @property
    def id(self) -> str:
        return self.name.lower()

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
