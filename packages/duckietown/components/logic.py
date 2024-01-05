
import dataclasses
import time
from typing import Optional, Any

from duckietown.types import Queue

from .base import Component
from ..types import Queue


@dataclasses.dataclass
class Behavior:
    start_time: float
    data: Any
    duration: Optional[float] = None
    notes: Any = None

    @property
    def expired(self) -> bool:
        if self.duration is None:
            return True
        return time.time() - self.start_time > self.duration

    def update(self, data: Any, duration: Optional[float] = None, notes: Any = None):
        self.start_time = time.time()
        self.data = data
        self.duration = duration
        self.notes = notes


class DrivingLogicComponent(Component[Any, Any]):
    """
    TODO

    Args:
        TODO

    """

    def __init__(self, stop_data: Any):
        super(DrivingLogicComponent, self).__init__()
        self.disabled: bool = False
        # store arguments
        self._stop_data: Any = stop_data
        # behavior
        self._behavior: Behavior = Behavior(time.time(), self._stop_data)
        # queues
        self.in_data: Queue[Any] = Queue()
        self.in_obstacle: Queue[bool] = Queue(repeat_last=True, initial=False)
        self.in_stopline: Queue[bool] = Queue(repeat_last=True, initial=False)
        self.out_stopped: Queue[bool] = Queue()
        self.out_data: Queue[Any] = Queue()

    def worker(self):
        stopped: bool = False
        while not self.is_shutdown:
            data: Any = self.in_data.get()

            if self.disabled:
                self.out_data.put(data)
                return

            if self._behavior.expired:
                obstacle: bool = self.in_obstacle.get()
                stopline: bool = self.in_stopline.get()

                if obstacle:
                    self._behavior.update(data=self._stop_data, duration=None, notes=self._behavior.notes)
                    stopped = True

                elif stopline and not self._behavior.notes == "stopline":
                    self._behavior.update(data=self._stop_data, duration=2.0, notes="stopline")
                    stopped = True

                else:
                    stopped = False
                    if self._behavior.notes == "stopline":
                        self._behavior.update(data=data, duration=2.0)
                    else:
                        self._behavior.update(data=data, duration=None)
            
            self.out_stopped.put(stopped)
            self.out_data.put(self._behavior.data)
