from functools import partial
from threading import Semaphore
from typing import Any, Tuple, List

from .base import Component, OutputType
from ..types import IQueue, Queue, CallbackQueue

__all__ = [
    "SynchronizerComponent"
]


class SynchronizerComponent(Component[Tuple[OutputType, ...], None]):

    def __init__(self, queues: Tuple[IQueue, ...]):
        super(SynchronizerComponent, self).__init__()
        self._queues: Tuple[IQueue, ...] = queues
        # internal state
        self._lock: Semaphore = Semaphore(1)
        self._data: List[Any] = [None] * len(self._queues)
        # use local callback queues to concentrate all the messages into a single place
        self._local: List[CallbackQueue] = [
            CallbackQueue(partial(self._collision_point, i), connect_to=q) for i, q in enumerate(self._queues)
        ]
        # the output queue
        self.out_data: IQueue[Tuple[Any, ...]] = Queue()

    def _collision_point(self, queue_idx: int, data: Any):
        with self._lock:
            self._data[queue_idx] = data
            # check whether it is NOT time to publish
            for v in self._data:
                if v is None:
                    return
            # publish
            self.out_data.put(tuple(self._data))
            # clear data
            self._data: List[Any] = [None] * len(self._queues)

    def start(self):
        pass

    def worker(self):
        pass
