import os
import time
from abc import abstractmethod
from threading import Condition
from typing import Optional, Any, Tuple, List, Callable

# noinspection PyUnresolvedReferences
from turbojpeg import TurboJPEG

from dt_duckiematrix_protocols.robot import DB21M
from dt_duckiematrix_protocols.robot.features.lights import LED, Lights
from dt_duckiematrix_protocols.robot.features.sensors import SensorAbs
from .base import OutputType, InputType, IComponent
from ..duckiematrix import Duckiematrix
from ..types import JPEGImage, BGRImage, IQueue, Queue, CallbackQueue, PWMSignal, LEDsPattern, RGBAColor, \
    Range, Ticks

__all__ = [
    "DuckiematrixCameraDriverComponent",
    "DuckiematrixTimeOfFlightDriverComponent",
    "DuckiematrixWheelEncoderDriverComponent",
    "DuckiematrixMotorsDriverComponent",
    "DuckiematrixLEDsDriverComponent"
]


class GenericDuckiematrixSubscriberComponent(IComponent[None, OutputType]):

    def __init__(self, vehicle_name: str):
        super(GenericDuckiematrixSubscriberComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        # TODO: we can figure this out from docker
        assert "DUCKIEMATRIX_ENGINE_HOST" in os.environ
        self._engine_hostname = os.environ["DUCKIEMATRIX_ENGINE_HOST"]
        # ---
        self._matrix = Duckiematrix.get_instance(self._engine_hostname)
        self._robot: DB21M = self._matrix.robots.DB21M(self._vehicle_name)
        self._callback: Callable[[Any], None] = lambda msg: self._out_data.put(self._msg_to_data(msg))
        # simulate Thread.join
        self._join: Condition = Condition()
        # queues
        self._out_data: IQueue[Any] = Queue()

    @staticmethod
    @abstractmethod
    def _msg_to_data(msg) -> Any:
        pass

    @property
    @abstractmethod
    def _sensor(self) -> SensorAbs:
        pass

    def start(self):
        self._sensor.attach(self._callback)

    def join(self, **kwargs) -> None:
        if self.is_shutdown:
            return
        with self._join:
            self._join.wait()

    def worker(self):
        pass

    def stop(self):
        self._is_shutdown = True
        try:
            self._sensor.detach(self._callback)
        except:
            pass
        # release all Threads that have joined this object
        with self._join:
            self._join.notify_all()


class GenericDuckiematrixPublisherComponent(IComponent[InputType, None]):

    def __init__(self, vehicle_name: str):
        super(GenericDuckiematrixPublisherComponent, self).__init__()
        self._vehicle_name: str = vehicle_name
        # TODO: we can figure this out from docker
        assert "DUCKIEMATRIX_ENGINE_HOST" in os.environ
        self._engine_hostname = os.environ["DUCKIEMATRIX_ENGINE_HOST"]
        # ---
        self._matrix = Duckiematrix.get_instance(self._engine_hostname)
        self._robot: DB21M = self._matrix.robots.DB21M(self._vehicle_name)
        # simulate Thread.join
        self._join: Condition = Condition()
        # queues
        self._in_data: IQueue[Any] = CallbackQueue(self.publish)

    def start(self) -> None:
        pass

    def join(self, **kwargs) -> None:
        if self.is_shutdown:
            return
        with self._join:
            self._join.wait()

    def worker(self):
        pass

    @abstractmethod
    def publish(self, data: Any, *, force: bool = False):
        pass

    def stop(self):
        self._is_shutdown = True
        # release all Threads that have joined this object
        with self._join:
            self._join.notify_all()


class DuckiematrixCameraDriverComponent(GenericDuckiematrixSubscriberComponent[BGRImage]):

    def __init__(self, vehicle_name: str):
        super(DuckiematrixCameraDriverComponent, self).__init__(vehicle_name)
        # JPEG decoder
        self._jpeg_decoder = TurboJPEG()

    @property
    def _sensor(self) -> SensorAbs:
        return self._robot.camera

    @property
    def out_bgr(self) -> IQueue[BGRImage]:
        return self._out_data

    def _msg_to_data(self, msg) -> BGRImage:
        jpeg: JPEGImage = msg.as_uint8()
        return self._jpeg_decoder.decode(jpeg)


class DuckiematrixTimeOfFlightDriverComponent(GenericDuckiematrixSubscriberComponent[Range]):

    MAX_RANGE: float = 1.2

    @property
    def _sensor(self) -> SensorAbs:
        # TODO: fix this in the protocols
        return self._robot._time_of_flight

    @property
    def out_range(self) -> IQueue[Range]:
        return self._out_data

    @staticmethod
    def _msg_to_data(msg) -> Optional[Range]:
        range: float = msg.range
        return None if range >= DuckiematrixTimeOfFlightDriverComponent.MAX_RANGE else range


class DuckiematrixWheelEncoderDriverComponent(GenericDuckiematrixSubscriberComponent[Range]):

    RESOLUTION: float = 135

    def __init__(self, vehicle_name: str, side: str):
        if side not in ["left", "right"]:
            raise ValueError(f"Side '{side}' not recognized. Valid choices are ['left', 'right'].")
        self.side: str = side
        super(DuckiematrixWheelEncoderDriverComponent, self).__init__(vehicle_name)
        # queues
        self.out_rotation: IQueue[float] = Queue()

    @property
    def _sensor(self) -> SensorAbs:
        return getattr(self._robot.wheels, self.side).encoder

    @property
    def out_ticks(self) -> IQueue[Ticks]:
        return self._out_data

    @property
    def resolution(self) -> Optional[float]:
        return DuckiematrixWheelEncoderDriverComponent.RESOLUTION

    def _msg_to_data(self, msg) -> Optional[Range]:
        ticks: int = msg.ticks
        rotation: float = ticks / self.resolution
        self.out_rotation.put(rotation)
        return ticks


class DuckiematrixLEDsDriverComponent(GenericDuckiematrixPublisherComponent[LEDsPattern]):

    OFF: RGBAColor = (0, 0, 0, 0)
    IDLE: LEDsPattern = LEDsPattern(
        # white on the front
        front_left=(1, 1, 1, 0.1),
        front_right=(1, 1, 1, 0.1),
        # red on the back
        rear_right=(1, 0, 0, 0.2),
        rear_left=(1, 0, 0, 0.2),
    )

    @property
    def in_pattern(self) -> IQueue[LEDsPattern]:
        return self._in_data

    def publish(self, data: LEDsPattern, *, force: bool = False):
        if self.is_shutdown and not force:
            return
        lights: Lights = self._robot.lights
        colors: List[RGBAColor] = [
            data.front_left, data.rear_left, self.OFF, data.rear_right, data.front_right
        ]
        leds: List[LED] = [
            lights.light0, lights.light1, lights.light2, lights.light3, lights.light4,
        ]
        for led, color in zip(leds, colors):
            self._set_light(led, color)

    @staticmethod
    def _set_light(light: LED, color: RGBAColor):
        light.color.r = color[0]
        light.color.g = color[1]
        light.color.b = color[2]
        light.color.a = color[3]

    def stop(self):
        self.publish(self.IDLE, force=True)
        super(DuckiematrixLEDsDriverComponent, self).stop()


class DuckiematrixMotorsDriverComponent(GenericDuckiematrixPublisherComponent[Tuple[PWMSignal, PWMSignal]]):

    OFF: float = 0.0

    def __init__(self, vehicle_name: str):
        super(DuckiematrixMotorsDriverComponent, self).__init__(vehicle_name)
        # queues
        self.out_command_time: Queue[float] = Queue()

    @property
    def in_pwml_pwmr(self) -> IQueue[Tuple[PWMSignal, PWMSignal]]:
        return self._in_data

    def publish(self, data: Tuple[PWMSignal, PWMSignal], *, force: bool = False):
        if self.is_shutdown and not force:
            return
        left, right = data
        self._robot.drive_pwm(left, right)
        self.out_command_time.put(time.time())

    def stop(self):
        self.publish((self.OFF, self.OFF), force=True)
        super(DuckiematrixMotorsDriverComponent, self).stop()
