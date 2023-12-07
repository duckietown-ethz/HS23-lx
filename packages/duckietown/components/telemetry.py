import time
from abc import ABC
from typing import Type, TypeVar

import requests

from duckietown.components.base import Component
from duckietown.telemetry import BatteryTelemetry
from duckietown.telemetry import (
    GenericTelemetryAbs,
    CPUTelemetry,
    GPUTelemetry,
    MemoryTelemetry,
    DiskTelemetry,
)
from duckietown.types import IQueue, Queue


T = TypeVar("T", bound=GenericTelemetryAbs)


class BatteryTelemetryComponent(Component[None, BatteryTelemetry]):
    def __init__(self, vehicle_name: str, frequency: float):
        super(BatteryTelemetryComponent, self).__init__()
        assert frequency > 0
        # define url
        self._url: str = f"http://{vehicle_name}.local/health/battery"
        self._frequency: float = frequency
        # queues
        self.out_data: IQueue = Queue()

    def worker(self):
        while not self.is_shutdown:
            # call API
            response = requests.get(self._url)
            data: dict = response.json()
            # instantiate BatteryTelemetry
            telemetry: BatteryTelemetry = BatteryTelemetry(**data["battery"])
            # send data out
            self.out_data.put(telemetry)
            # wait
            time.sleep(1.0 / self._frequency)


class GenericComputeTelemetryComponentAbs(Component[None, T], ABC):
    def __init__(
        self,
        TelemetryClass: Type[GenericTelemetryAbs],
        category: str,
        vehicle_name: str,
        frequency: float,
    ):
        super(GenericComputeTelemetryComponentAbs, self).__init__()
        assert frequency > 0
        self._category: str = category
        self._TelemetryClass: Type[GenericTelemetryAbs] = TelemetryClass
        # define url
        self._url: str = f"http://{vehicle_name}.local/health/{self._category}"
        self._frequency: float = frequency
        # queues
        self.out_data: IQueue = Queue()

    def worker(self):
        while not self.is_shutdown:
            # call API
            response = requests.get(self._url)
            data: dict = response.json()
            # instantiate Telemetry object
            telemetry: GenericTelemetryAbs = self._TelemetryClass.from_dict(
                data[self._category]
            )
            # send data out
            self.out_data.put(telemetry)
            # wait
            time.sleep(1.0 / self._frequency)


class CPUTelemetryComponent(GenericComputeTelemetryComponentAbs[CPUTelemetry]):
    def __init__(self, vehicle_name: str, frequency: float):
        super(CPUTelemetryComponent, self).__init__(
            CPUTelemetry, "cpu", vehicle_name, frequency
        )


class GPUTelemetryComponent(GenericComputeTelemetryComponentAbs[GPUTelemetry]):
    def __init__(self, vehicle_name: str, frequency: float):
        super(GPUTelemetryComponent, self).__init__(
            GPUTelemetry, "gpu", vehicle_name, frequency
        )


class MemoryTelemetryComponent(GenericComputeTelemetryComponentAbs[MemoryTelemetry]):
    def __init__(self, vehicle_name: str, frequency: float):
        super(MemoryTelemetryComponent, self).__init__(
            MemoryTelemetry, "memory", vehicle_name, frequency
        )


class DiskTelemetryComponent(GenericComputeTelemetryComponentAbs[DiskTelemetry]):
    def __init__(self, vehicle_name: str, frequency: float):
        super(DiskTelemetryComponent, self).__init__(
            DiskTelemetry, "disk", vehicle_name, frequency
        )
