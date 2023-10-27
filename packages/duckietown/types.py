import queue
from typing import Tuple, Dict, Union, TypeVar, Generic, Set, Any

import numpy as np

from .exceptions import ComponentShutdown

JPEGImage = bytes
BGRImage = np.ndarray
Color = Tuple[int, int, int]
ColorName = str
DetectedLines = Dict[str, list]
CameraParameters = Dict[str, Union[np.ndarray, int]]


T = TypeVar("T")
SHUTDOWN_DUMMY = object()


class Queue(Generic[T], queue.Queue):
    """
    Same as queue.Queue but with at most 1 item and a put() that replaces the old element in the queue if any.
    """

    def __init__(self, repeat_last: bool = False, initial: T = None):
        super(Queue, self).__init__(maxsize=1)
        # internal state
        self._repeat_last: bool = repeat_last
        self._last: T = None
        self._links: Set[Queue] = {self}
        self._is_shutdown: bool = False
        # initial value
        if initial is not None:
            self.put(initial)

    # noinspection PyMethodOverriding
    def get(self) -> T:
        if self._is_shutdown:
            raise ComponentShutdown()
        # get item
        if self._repeat_last and self._last is not None:
            try:
                item: Any = super(Queue, self).get(block=False)
            except queue.Empty:
                item: Any = self._last
        else:
            item: Any = super(Queue, self).get()
        # we pass the dummy to the queue to unlock them and make them exit
        if item is SHUTDOWN_DUMMY:
            raise ComponentShutdown()
        # ---
        self._last = item
        return item

    # noinspection PyMethodOverriding
    def put(self, value: T):
        for q in self._links:
            try:
                super(Queue, q).get(block=False)
            except queue.Empty:
                pass
            super(Queue, q).put(value)

    def forward_to(self, q: 'Queue'):
        self._links.add(q)

    def wants(self, q: 'Queue'):
        q._links.add(self)

    def stop(self):
        self._is_shutdown = True
        self.put(SHUTDOWN_DUMMY)
