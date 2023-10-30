import os
from abc import ABC
from typing import Optional, Tuple

from .base import OutputType, InputType, IComponent
from .duckiematrix import \
    DuckiematrixCameraDriverComponent, \
    DuckiematrixTimeOfFlightDriverComponent, \
    DuckiematrixWheelEncoderDriverComponent, \
    DuckiematrixMotorsDriverComponent, \
    DuckiematrixLEDsDriverComponent
from .ros import \
    ROSCameraDriverComponent, \
    ROSTimeOfFlightDriverComponent, \
    ROSWheelEncoderDriverComponent, \
    ROSMotorsDriverComponent, \
    ROSLEDsDriverComponent
from ..types import BGRImage, IQueue, PWMSignal, LEDsPattern, Range, Ticks, Queue

__all__ = [
    "CameraDriverComponent",
    "TimeOfFlightDriverComponent",
    "WheelEncoderDriverComponent",
    "LEDsDriverComponent",
    "MotorsDriverComponent",
]


class GenericProxiedComponent(IComponent[InputType, OutputType], ABC):

    def __init__(self):
        super(GenericProxiedComponent, self).__init__()
        self._proxy: Optional[IComponent] = None

    def start(self):
        self._proxy.start()

    def stop(self):
        self._proxy.stop()

    def join(self):
        self._proxy.join()

    def worker(self):
        self._proxy.worker()


class GenericSubscriberComponent(GenericProxiedComponent[None, OutputType], ABC):
    pass


class GenericPublisherComponent(GenericProxiedComponent[InputType, None], ABC):
    pass


class CameraDriverComponent(GenericSubscriberComponent[BGRImage]):

    def __init__(self, vehicle_name: str, **kwargs):
        super(CameraDriverComponent, self).__init__()
        if "DUCKIEMATRIX_ENGINE_HOST" in os.environ:
            print("Using Duckiematrix...")
            # use duckiematrix
            self._proxy = DuckiematrixCameraDriverComponent(vehicle_name)
        else:
            print("Using ROS...")
            # use ROS
            self._proxy = ROSCameraDriverComponent(vehicle_name, **kwargs)

    @property
    def out_bgr(self) -> IQueue[BGRImage]:
        return self._proxy.out_bgr


class TimeOfFlightDriverComponent(GenericSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, **kwargs):
        super(TimeOfFlightDriverComponent, self).__init__()
        if "DUCKIEMATRIX_ENGINE_HOST" in os.environ:
            print("Using Duckiematrix...")
            # use duckiematrix
            self._proxy = DuckiematrixTimeOfFlightDriverComponent(vehicle_name)
        else:
            print("Using ROS...")
            # use ROS
            self._proxy = ROSTimeOfFlightDriverComponent(vehicle_name, **kwargs)

    @property
    def out_range(self) -> IQueue[Range]:
        return self._proxy.out_range


class WheelEncoderDriverComponent(GenericSubscriberComponent[Range]):

    def __init__(self, vehicle_name: str, **kwargs):
        super(WheelEncoderDriverComponent, self).__init__()
        if "DUCKIEMATRIX_ENGINE_HOST" in os.environ:
            print("Using Duckiematrix...")
            # use duckiematrix
            self._proxy = DuckiematrixWheelEncoderDriverComponent(vehicle_name, **kwargs)
        else:
            print("Using ROS...")
            # use ROS
            self._proxy = ROSWheelEncoderDriverComponent(vehicle_name, **kwargs)

    @property
    def out_ticks(self) -> IQueue[Ticks]:
        return self._proxy.out_ticks

    @property
    def resolution(self) -> Optional[float]:
        return self._proxy.resolution


class LEDsDriverComponent(GenericPublisherComponent[LEDsPattern]):

    def __init__(self, vehicle_name: str):
        super(LEDsDriverComponent, self).__init__()
        if "DUCKIEMATRIX_ENGINE_HOST" in os.environ:
            print("Using Duckiematrix...")
            # use duckiematrix
            self._proxy = DuckiematrixLEDsDriverComponent(vehicle_name)
        else:
            print("Using ROS...")
            # use ROS
            self._proxy = ROSLEDsDriverComponent(vehicle_name)

    @property
    def in_pattern(self) -> IQueue[LEDsPattern]:
        return self._proxy.in_pattern


class MotorsDriverComponent(GenericPublisherComponent[Tuple[PWMSignal, PWMSignal]]):

    def __init__(self, vehicle_name: str):
        super(MotorsDriverComponent, self).__init__()
        if "DUCKIEMATRIX_ENGINE_HOST" in os.environ:
            print("Using Duckiematrix...")
            # use duckiematrix
            self._proxy = DuckiematrixMotorsDriverComponent(vehicle_name)
        else:
            print("Using ROS...")
            # use ROS
            self._proxy = ROSMotorsDriverComponent(vehicle_name)

    @property
    def in_pwml_pwmr(self) -> IQueue[Tuple[PWMSignal, PWMSignal]]:
        return self._proxy.in_pwml_pwmr

    @property
    def out_command_time(self) -> IQueue[float]:
        return self._proxy.out_command_time
