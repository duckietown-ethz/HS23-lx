import dataclasses
import queue
from abc import abstractmethod, ABC
from typing import Tuple, Dict, Union, TypeVar, Generic, Set, Any, Callable

import numpy as np

from .exceptions import ComponentShutdown

JPEGImage = bytes
BGRImage = np.ndarray
RGB8Color = BGR8Color = Tuple[int, int, int]
RGBColor = BGRColor = Tuple[float, float, float]
RGBAColor = Tuple[float, float, float, float]
PWMSignal = float
Range = float
Ticks = int
ColorName = str
DetectedLines = Dict[str, list]
CameraParameters = Dict[str, Union[np.ndarray, int]]


T = TypeVar("T")
R = TypeVar("R")
SHUTDOWN_DUMMY = object()


class IQueue(Generic[T], ABC):

    def __init__(self):
        self._links: Set[IQueue] = {self}
        self._is_shutdown: bool = False

    @abstractmethod
    def get(self) -> T:
        pass

    @abstractmethod
    def put(self, value: T):
        pass

    @abstractmethod
    def reset(self):
        pass

    @property
    def anybody_interested(self) -> bool:
        return len(self._links) > 1

    def forward_to(self, q: 'IQueue'):
        self._links.add(q)

    def wants(self, q: 'IQueue'):
        q._links.add(self)

    def stop(self):
        self._is_shutdown = True
        self.put(SHUTDOWN_DUMMY)


class Queue(IQueue[T]):
    """
    Same as queue.Queue but with at most 1 item and a put() that replaces the old element in the queue if any.
    """

    def __init__(self, repeat_last: bool = False, initial: T = None):
        super(Queue, self).__init__()
        # internal state
        self._repeat_last: bool = repeat_last
        self._initial: T = initial
        self._last: T = None
        self._proxied: queue.Queue = queue.Queue(maxsize=1)
        # initial value
        if self._initial is not None:
            self.put(self._initial)

    # noinspection PyMethodOverriding
    def get(self) -> T:
        if self._is_shutdown:
            raise ComponentShutdown()
        # get item
        if self._repeat_last and self._last is not None:
            try:
                item: Any = self._proxied.get(block=False)
            except queue.Empty:
                item: Any = self._last
        else:
            item: Any = self._proxied.get()
        # we pass the dummy to the queue to unlock them and make them exit
        if item is SHUTDOWN_DUMMY:
            # re-add to the queue to wake others
            self.put(SHUTDOWN_DUMMY)
            raise ComponentShutdown()
        # ---
        self._last = item
        return item

    # noinspection PyMethodOverriding
    def put(self, value: T):
        # iterate over the linked queues
        for q in self._links:
            if q is self:
                # clear old and add new
                try:
                    self._proxied.get(block=False)
                except queue.Empty:
                    pass
                self._proxied.put(value)
            else:
                # we do not want to shut down other queues, just us
                if value is SHUTDOWN_DUMMY:
                    continue
                # delegate the other queue to figure out what to do
                q.put(value)

    def reset(self):
        self._is_shutdown = False
        # recreate queue
        self._proxied: queue.Queue = queue.Queue(maxsize=1)
        # initial value
        if self._initial is not None:
            self.put(self._initial)


class CallbackQueue(IQueue[T]):

    def __init__(self, callback: Callable[[T], None], connect_to: IQueue = None):
        super(CallbackQueue, self).__init__()
        self._callback: Callable[[T], None] = callback
        if connect_to is not None:
            self.wants(connect_to)

    def get(self) -> T:
        raise NotImplementedError(
            "You cannot call the method 'get()' on an instance of CallbackQueue. "
            "Messages are delivered to the given callback, use the class Queue instead."
        )

    def put(self, value: T):
        # we do not want to send the dummy through the callback
        if value is SHUTDOWN_DUMMY:
            return
        # iterate over the linked queues
        for q in self._links:
            if q is self:
                self._callback(value)
            else:
                q.put(value)

    def reset(self):
        self._is_shutdown = False


class LambdaQueue(IQueue[T], Generic[T, R]):

    def __init__(self, target: IQueue, get: Callable[[T], R] = None, put: Callable[[T], R] = None):
        super(LambdaQueue, self).__init__()
        passthrough: Callable[[T], R] = lambda x: x
        self._get_wrapper: Callable[[T], None] = get or passthrough
        self._put_wrapper: Callable[[T], None] = put or passthrough
        self._target: IQueue = target

    def get(self) -> T:
        return self._get_wrapper(self._target.get())

    def put(self, value: T):
        self._target.put(self._put_wrapper(value))

    def reset(self):
        self._is_shutdown = False


@dataclasses.dataclass
class LEDsPattern:
    front_left: RGBAColor
    front_right: RGBAColor
    rear_right: RGBAColor
    rear_left: RGBAColor
